#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2021 University of Oxford, 2024 CNRS
##
## This file is part of Cockpit.
##
## Cockpit is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## Cockpit is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with Cockpit.  If not, see <http://www.gnu.org/licenses/>.


"""This device module is a reinterpretation of boulderSLM in order to work with the newer implementation of
microscope's SLM"""

import decimal
import time
from itertools import groupby

import numpy as np
import Pyro4
import wx

import cockpit.gui.dialogs.getNumberDialog
import cockpit.gui.guiUtils
import cockpit.handlers.executor
import cockpit.util
from cockpit import events
from cockpit.devices import device
from cockpit.gui.device import (
    EnableButton,
    DEFAULT_SIZE
)

TWO_PI = 2.0 * np.pi


class SIM_SLM(device.Device):
    """A microscope SLM device implementing 3D SIM logic and GUI.

    Remotely, the SLM must be running a sim_slm device server.

    Sample config entry:

    .. code:: ini

        [slm]
        type: cockpit.devices.sim_slm.SIM_SLM
        uri: PYRO:sim_slm@slmhost:8000
        pixelPitch: 15.0
        diffractionAngle: 0.45
        modulationFactors: 488: 190
                           561: 180
                           647: 170
        triggerSource: NAME_OF_EXECUTOR_DEVICE
        triggerLine: 2
        settlingTime: 10

    """

    _config_types = {
        "settlingTime": float,
        "triggerLine": int,
    }

    def __init__(self, name, config={}):
        super().__init__(name, config)

        # General device properties.
        self.connection = None
        self.position = None
        self.wasPowered = None
        self.slmTimeout = 10
        self.slmRetryLimit = 3
        self.shape = None
        self.pixelPitch = None

        # SIM-specific properties.
        self.diffractionAngle = None
        self.modulationFactors = {}
        self.simPhaseOffset = 0.0
        self.simAngleOffset = TWO_PI / 5.0
        self.numPhases = 5
        self.numAngles = 3

        self._kk = None
        self._ll = None
        self._patterns = None
        self._wavelengths = None
        self.sequenceParameters = []

        # GUI properties.
        self.menuItems = None

    def initialize(self):
        if self.uri:
            uri = self.uri
        else:
            uri = "PYRO:pyroSLM@%s:%d" % (self.ipAddress, self.port)
        self.connection = Pyro4.Proxy(uri)

        self.diffractionAngle = float(
            self.config.get("diffractionangle", None)
        )

        for vdef in self.config.get("modulationfactors", "").split("\n"):
            if vdef == "":
                continue
            w, f = vdef.strip("\n").split(":")
            self.modulationFactors[int(w)] = int(f)

        if not self.modulationFactors:
            raise Warning("No modulation factors defined in config.")

        self.shape = self.connection.get_shape()
        self._kk, self._ll = np.meshgrid(
            np.arange(self.shape[0]),
            np.arange(self.shape[1]),
        )

        try:
            self.pixelPitch = self.connection.get_pixel_pitch()
        except AttributeError:
            self.pixelPitch = float(self.config.get("pixelpitch", None))

        if not self.pixelPitch:
            raise Warning("No pixel pitch defined in config.")

    def onExit(self) -> None:
        if self.connection is not None:
            self.connection.disable()
            self.connection._pyroRelease()
        super().onExit()

    def finalizeInitialization(self):
        # A mapping of context-menu entries to functions.
        # Define in tuples - easier to read and reorder.
        self.menuItems = [
            ("Generate SIM sequence", self.testSIMSequence),
            ("SIM diff. angle", self.setDiffractionAngle),
            ("SIM modulation factor", self.setModulationFactors),
        ]

    def getIsEnabled(self):
        return self.connection.get_is_enabled()

    def setEnabled(self, state):
        """Enable or disable the SLM."""
        if state:
            self.connection.enable()
        else:
            self.connection.disable()

    def cycleToPosition(self, targetPosition):
        pos = self.getCurrentPosition()
        delta = (targetPosition - pos) + (targetPosition < pos) * len(
            self.sequenceParameters
        )
        for _ in range(delta):
            self.handler.triggerNow()
            time.sleep(0.01)

    def executeTable(self, table, startIndex, stopIndex, numReps, repDuration):
        # Found a table entry with a simple index. Trigger until that index
        # is reached.
        for t, h, args in table[startIndex:stopIndex]:
            events.publish(
                events.UPDATE_STATUS_LIGHT,
                "device waiting",
                "SLM moving to index %d" % args,
            )
            self.cycleToPosition(args)

    def examineActions(self, table):
        # Extract pattern parameters from the table.
        # patternParms is a list of tuples (angle, phase, wavelength)
        patternParams = [row[2] for row in table if row[1] is self.handler]
        if not patternParams:
            # SLM is not used in this experiment.
            return

        # Remove consecutive duplicates and position resets.
        reducedParams = [
            p[0] for p in groupby(patternParams) if type(p[0]) is tuple
        ]
        # Find the repeating unit in the sequence.
        sequenceLength = len(reducedParams)
        for length in range(2, len(reducedParams) // 2):
            if reducedParams[:length] == reducedParams[length:2 * length]:
                sequenceLength = length
                break
        sequence = reducedParams[:sequenceLength]

        self.compute3DSIMSequence(sequence)
        self.sendPatterns()

        # Track sequence index set by last set of triggers.
        lastIndex = 0
        for i, (t, handler, action) in enumerate(table.actions):
            if handler is not self.handler:
                # Nothing to do
                continue
            # Action specifies a target frame in the sequence.
            # Remove original event.
            table[i] = None
            # How many triggers?
            if type(action) is tuple and action != sequence[lastIndex]:
                # Next pattern does not match last, so step one pattern.
                numTriggers = 1
            elif type(action) is int:
                if action >= lastIndex:
                    numTriggers = action - lastIndex
                else:
                    numTriggers = sequenceLength - lastIndex - action
            else:
                numTriggers = 0
            """
            Used to calculate time to execute triggers and settle here, 
            then push back all later events, but that leads to very long
            delays before the experiment starts. For now, comment out
            this code, and rely on a fixed time passed back to the action
            table generator (i.e. experiment class).

            # How long will the triggers take?
            # Time between triggers must be > table.toggleTime.
            ## Shift later table entries to allow for triggers and settling.
            table.shiftActionsBack(time, dt)
            for trig in range(numTriggers):
                t = table.addToggle(t, triggerHandler)
                t += table.toggleTime
            """
            for _ in range(numTriggers):
                t = table.addToggle(t, self.handler)
                t += table.toggleTime

            lastIndex += numTriggers
            if lastIndex >= sequenceLength:
                lastIndex = lastIndex % sequenceLength
        table.clearBadEntries()
        # Store the parameters used to generate the sequence.
        self.sequenceParameters = sequence
        self.connection.run_queue()
        # Fire several triggers to ensure that the sequence is loaded.
        for _ in range(12):
            self.handler.triggerNow()
            time.sleep(0.01)
        # Ensure that we're at position 0.
        self.cycleToPosition(0)
        self.position = self.getCurrentPosition()

    def getCurrentPosition(self):
        return self.connection.get_pattern_idx()

    def getHandlers(self):
        trigsource = self.config.get("triggersource", None)
        trigline = self.config.get("triggerline", None)
        dt = decimal.Decimal(self.config.get("settlingtime", 10))
        self.handler = cockpit.handlers.executor.DelegateTrigger(
            "slm",
            "slm group",
            True,
            {
                "examineActions": self.examineActions,
                "getMovementTime": lambda *args: dt,
                "executeTable": self.executeTable,
                "setEnabled": self.setEnabled,
                "getIsEnabled": self.getIsEnabled,
            },
        )
        if trigline is not None and trigsource is not None:
            self.handler.delegateTo(trigsource, trigline, 0, dt)
        return [self.handler]

    # SIM-specific methods
    def compute3DSIMSequence(self, anglePhaseWavelength):
        """Generate a SIM sequence from a list of parameters.
        angle_phase_wavelength is a list where each element is a tuple of the
        form (angle_number, phase_number, wavelength).
        """
        num_phases = 0
        num_angles = 0
        wavelengths = []
        for angle, phase, wavelength in anglePhaseWavelength:
            num_phases = max(num_phases, phase + 1)
            num_angles = max(num_angles, angle + 1)
            if wavelength not in wavelengths:
                wavelengths.append(wavelength)

        phases = [
            self.simPhaseOffset + n * TWO_PI / num_phases
            for n in range(num_phases)
        ]
        angles = [
            self.simAngleOffset + n * TWO_PI / num_angles
            for n in range(num_angles)
        ]

        # Calculate line pitches for each wavelength, once.
        # d = m * wavelength / np.sin theta
        # 1/1000 since wavelength in nm, pixel pitch in microns.
        pitches = {
            w: w / (1000.0 * np.sin(self.diffractionAngle * TWO_PI / 360.0))
            for w in wavelengths
        }

        patterns = np.zeros(
            (len(anglePhaseWavelength), *self.shape),
            dtype=np.float32,
        )
        wavelengthSeq = []
        for i, (angle, phase, wavelength) in enumerate(anglePhaseWavelength):
            # retardation for equal powers in 0 and combined +/-1 orders
            modulation = self.modulationFactors[wavelength] / 360.0

            pp = pitches[wavelength] / self.pixelPitch
            th = angles[angle]
            ph = phases[phase]
            # Create a stripe float pattern
            patterns[i] = (
                (0.5 * modulation)
                + (0.5 * modulation)
                * np.cos(
                    ph
                    + TWO_PI
                    * (np.cos(th) * self._kk + np.sin(th) * self._ll)
                    / pp
                )
            ).astype(np.float32)
            # Lose two LSBs and pass through the LUT for given wavelength.
            wavelengthSeq.append(wavelength)

        self.sequenceParameters = anglePhaseWavelength
        self._patterns = patterns
        self._wavelengths = wavelengthSeq

    ### UI functions ###
    def makeUI(self, parent):
        panel = wx.Panel(parent, style=wx.BORDER_RAISED)
        panel.SetDoubleBuffered(True)
        panel.Sizer = wx.BoxSizer(wx.VERTICAL)
        powerButton = EnableButton(panel, self.handler)
        panel.Sizer.Add(powerButton, 0, wx.EXPAND)
        triggerButton = wx.Button(panel, label="step")
        triggerButton.Bind(wx.EVT_BUTTON, lambda evt: self.onStep(evt))
        panel.Sizer.Add(triggerButton, 0, wx.EXPAND)
        # Add a status info display.
        statusDisplay = wx.StaticText(
            parent=panel,
            style=wx.ALIGN_CENTRE | wx.ST_NO_AUTORESIZE,
            size=(DEFAULT_SIZE[0], 3 * DEFAULT_SIZE[1]),
        )
        statusDisplay.SetFont(statusDisplay.GetFont().Smaller())
        statusDisplay.Bind(
            wx.EVT_TIMER, lambda event: self.updateStatusDisplay(event)
        )
        panel.Sizer.Add(statusDisplay)
        # Set up a timer to update value displays.
        self.updateTimer = wx.Timer(statusDisplay)
        self.updateTimer.Start(1000)
        # Changed my mind. SIM diffraction angle is an advanced parameter,
        # so it now lives in a right-click menu rather than on a button.
        panel.Bind(wx.EVT_CONTEXT_MENU, self.onRightMouse)
        # Controls other than powerButton only enabled when SLM is enabled.
        triggerButton.Disable()
        statusDisplay.Disable()
        powerButton.manageStateOf((triggerButton, statusDisplay))
        return panel

    def onStep(self, event):
        self.connection.trigger()

    def sendPatterns(self):
        if self._patterns is not None:
            self.connection.queue_patterns(self._patterns, self._wavelengths)

    def updateStatusDisplay(self, event):
        # Get the display object. It seems there is variation between
        # wx versions. With some versions, the display is obtained by
        #    event.GetEventObject().
        # With others, it is
        #    event.GetEventObject().GetOwner()
        display = event.GetEventObject()
        if not hasattr(display, "SetLabel"):
            display = display.GetOwner()
        self.position = self.getCurrentPosition()
        if self.position is None:
            display.SetLabel("No queue\nrunning")
        elif not self.sequenceParameters:
            display.SetLabel("No sequence\ngenerated.\nPlease set one.")
        else:
            parms = self.sequenceParameters[self.position]
            display.SetLabel("angle:\t%s\nphase:\t%s\nwavel.:\t%s" % parms)

    def onPrepareForExperiment(self, *args):
        self.position = self.getCurrentPosition()
        self.wasPowered = self.getIsEnabled()

    def cleanupAfterExperiment(self, *args):
        if not self.wasPowered:
            self.setEnabled(False)

    def performSubscriptions(self):
        # events.subscribe(events.USER_ABORT, self.onAbort)
        events.subscribe(
            events.PREPARE_FOR_EXPERIMENT, self.onPrepareForExperiment
        )
        events.subscribe(
            events.CLEANUP_AFTER_EXPERIMENT, self.cleanupAfterExperiment
        )

    ### Context menu and handlers ###
    def menuCallback(self, index, item):
        func = self.menuItems[item]
        return func()

    def onRightMouse(self, event):
        menu = wx.Menu()
        for item, callback in self.menuItems:
            if item:
                menu_item = menu.Append(wx.ID_ANY, item)
                menu.Bind(wx.EVT_MENU, callback, menu_item)
            else:
                menu.AppendSeparator()
        cockpit.gui.guiUtils.placeMenuAtMouse(event.GetEventObject(), menu)

    def testSIMSequence(self, event):
        inputs = cockpit.gui.dialogs.getNumberDialog.getManyNumbersFromUser(
            None,
            "Generate a SIM sequence",
            [
                "wavelength",
                "total angles",
                "total phases",
                "order\n0 for a then ph\n1 for ph then a",
            ],
            (488, 3, 5, 0),
        )
        wavelength, angles, phases, order = [int(i) for i in inputs]
        if order == 0:
            params = [
                (theta, phi, wavelength)
                for phi in range(phases)
                for theta in range(angles)
            ]
        elif order == 1:
            params = [
                (theta, phi, wavelength)
                for theta in range(angles)
                for phi in range(phases)
            ]
        else:
            raise ValueError("Order must be 0 or 1.")

        self.compute3DSIMSequence(params)
        self.sendPatterns()
        self.connection.run_queue()

    def setDiffractionAngle(self, event):
        theta = self.diffractionAngle
        newTheta = float(
            cockpit.gui.dialogs.getNumberDialog.getNumberFromUser(
                None,
                "Set SIM diffraction angle",
                (
                    "Adjust diffraction angle to\nput spots at edge of pupil.\n"
                    "Current angle is %.2f°." % theta
                ),
                theta,
                atMouse=True,
            )
        )
        self.diffractionAngle = newTheta

    def setModulationFactors(self, event):
        modulation_factors = self.modulationFactors
        new_modulation_factors = (
            cockpit.gui.dialogs.getNumberDialog.getManyNumbersFromUser(
                None,
                "Set SIM modulation factors",
                [str(w) for w in modulation_factors.keys()],
                list(modulation_factors.values()),
                atMouse=True,
            )
        )
        new_modulation_factors = {
            int(wavelength): int(factor)
            for wavelength, factor in zip(
                modulation_factors.keys(), new_modulation_factors
            )
        }
        self.modulationFactors = new_modulation_factors
