#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2021 University of Oxford
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

## Copyright 2013, The Regents of University of California
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##
## 1. Redistributions of source code must retain the above copyright
##   notice, this list of conditions and the following disclaimer.
##
## 2. Redistributions in binary form must reproduce the above copyright
##   notice, this list of conditions and the following disclaimer in
##   the documentation and/or other materials provided with the
##   distribution.
##
## 3. Neither the name of the copyright holder nor the names of its
##   contributors may be used to endorse or promote products derived
##   from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
## FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
## COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
## INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
## BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
## LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
## CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
## LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
## ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.


import threading
import time

import wx

import cockpit.util.user_config
from cockpit import depot
from cockpit.experiment import z_stack
from cockpit.gui import gui_utils


## Provided so the UI knows what to call this experiment.
EXPERIMENT_NAME = "Example opto script"


class OptoExperiment(z_stack.ZStackExperiment):
    """This experiment is an example of a custom opto script.

    It is identical to a standard Z-stack experiment except that it turns on
    and off lights in a configurable timed sequence before running the stack
    Note that this class inherits from the ZStackExperiment class in
    experiment/z_stack.py; the generateActions() function in that class is what
    collects the actual data. We just modify the execute() function (inherited
    from the base Experiment class in experiment/experiment.py) to engage the
    lights as specified before each rep. See below this class for the
    ExperimentUI and OptoPanel classes that set up the UI for configuring the
    experiment settings.
    """

    def __init__(
        self, lightToSequence, lightToIsOnDuringAcquisition, **kwargs
    ):
        """Set up the opto experiment with its light sequences.

        Args:
            lightToSequence (dict): Maps light handler names to lists of
                [start, stop] times for illuminating them prior to running a
                standard Z-stack.
            lightToIsOnDuringAcquisition (dict): Maps light handler names to
                whether or not those lights should be left on while taking
                images in the Z-stack section of the experiment.
        """
        # Convert from light source names to light handlers.
        self.lightToSequence = {}
        for name, sequence in lightToSequence.items():
            handler = depot.getHandlerWithName(name)
            self.lightToSequence[handler] = sequence
        self.lightToIsOnDuringAcquisition = {}
        for name, isOn in lightToIsOnDuringAcquisition.items():
            handler = depot.getHandlerWithName(name)
            self.lightToIsOnDuringAcquisition[handler] = isOn
        # Call the ZStackExperiment constructor with all of our remaining
        # parameters, which are in the "kwargs" dictionary object.
        super().__init__(**kwargs)

    def execute(self):
        """This function overrides the base execute() function in the
        Experiment class."""
        # HACK: replace self.numReps since it's referred to in
        # Experiment.execute(), and we want to handle repeats at a higher
        # level.
        numReps = self.numReps
        self.numReps = 1
        for i in range(numReps):
            if self.shouldAbort:
                # User aborted experiment.
                break
            # Create a new thread for activating each light in parallel.
            # We could do this single-threaded, but it would require an
            # in-depth examination of self.lightToSequence to determine when
            # to turn on each light.
            threads = []
            for light, sequence in self.lightToSequence.items():
                newThread = threading.Thread(
                    target=self.shineLight, args=[light, sequence]
                )
                newThread.start()
                threads.append(newThread)
            # Wait for all of our threads to finish.
            for thread in threads:
                thread.join()

            # Pre-acquisition illumination is complete; move on to taking
            # images.
            # Turn on all lights that want to be left on during acquisition.
            activatedLights = []
            for light, shouldBeOn in self.lightToIsOnDuringAcquisition.items():
                if shouldBeOn:
                    light.setExposing(True)
                    activatedLights.append(light)
            # Invoke our base class' execute() function to collect data.
            result = super().execute()
            # Turn off the triggering lights.
            for light in activatedLights:
                light.setExposing(False)
        return result

    def shineLight(self, light, sequence):
        """Given a light source and a list of [start, stop] pairs when the
        light should be enabled/disabled, turn the light on and off according
        to the sequence."""
        curTime = 0
        for start, stop in sequence:
            # Sleep until we should start illumination.
            time.sleep(start - curTime)
            # Turn the light on.
            light.setExposing(True)
            # Sleep until we should turn the light off.
            time.sleep(stop - start)
            # Turn the light off.
            light.setExposing(False)
            curTime = stop


## A consistent name to use to refer to the experiment class itself.
EXPERIMENT_CLASS = OptoExperiment


class ExperimentUI(wx.Panel):
    """Generate the UI for special parameters used by this experiment."""

    def __init__(self, parent, configKey):
        super().__init__(parent=parent)

        self.configKey = configKey

        ## List of all light sources available for use.
        self.allLights = depot.getHandlersOfType(depot.LIGHT_TOGGLE)

        ## User config settings.
        self.settings = self.loadSettings()

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(
            wx.StaticText(self, -1, "Opto light exposure sequences:"),
            0,
            wx.ALL,
            5,
        )

        # Generate a header to label the controls in each OptoPanel. Sizes
        # for each element are derived from the OptoPanel's layout.
        headerSizer = wx.BoxSizer(wx.HORIZONTAL)
        for text, size in [
            ("Light", 60),
            ("", 80),
            ("", 80),
            ("Sequence", 200),
            ("Start", 40),
            ("End", 40),
            ("", 80),
            ("On during acquisition", 200),
        ]:
            headerSizer.Add(wx.StaticText(self, -1, text, size=(size, -1)))

        sizer.Add(headerSizer)

        ## Maps light handlers to OptoPanel instances.
        self.lightToPanel = {}
        defaultVals = []
        for light in self.allLights:
            sequence = self.settings["lightToSequence"].get(light.name, [])
            doesStartOn = self.settings["lightToIsOnDuringAcquisition"].get(
                light.name, False
            )
            panel = OptoPanel(self, light, sequence, doesStartOn)
            self.lightToPanel[light] = panel
            sizer.Add(panel)

        self.SetSizerAndFit(sizer)

    def getLightToSequence(self):
        """Return a mapping of light source names to lists of illumination
        sequences. e.g. {"650 LED": [[0, 20], [30, 50]]} to indicate that the
        650 LED should be on from 0 to 20 seconds, and from 30 to 50 seconds.

        We use handler names instead of handlers here so that they can be
        serialized (when saving experiment settings) cleanly.
        """
        return dict(
            [
                (l.name, self.lightToPanel[l].getSequence())
                for l in self.allLights
            ]
        )

    def getLightToIsOnDuringAcquisition(self):
        """Returns a mapping of light source names to whether or not those
        lights should be left on during image acquisition (independently of
        whether those lights are being used as traditional imaging lights).

        We use handler names instead of handlers here so that they can be
        serialized (when saving experiment settings) cleanly.
        """
        return dict(
            [
                (l.name, self.lightToPanel[l].getIsOnDuringAcquisition())
                for l in self.allLights
            ]
        )

    def augmentParams(self, params, shouldSave=True):
        """Given a parameters dict (parameter name to value) to hand to the
        experiment instance,
        augment them with our special parameters."""
        if shouldSave:
            self.saveSettings()
        params["lightToSequence"] = self.getLightToSequence()
        params["lightToIsOnDuringAcquisition"] = (
            self.getLightToIsOnDuringAcquisition()
        )
        return params

    def loadSettings(self):
        """Load the saved experiment settings, if any."""
        return cockpit.util.user_config.getValue(
            self.configKey + "optoExampleSettings",
            default={
                "lightToSequence": {},
                "lightToIsOnDuringAcquisition": {},
            },
        )

    def getSettingsDict(self):
        """Generate a dict of our settings."""
        return self.augmentParams({}, shouldSave=False)

    def saveSettings(self, settings=None):
        """Save the current experiment settings to config."""
        if settings is None:
            settings = self.getSettingsDict()
        cockpit.util.user_config.setValue(
            self.configKey + "optoExampleSettings", settings
        )


## This panel lets the user define a sequence of illumination sequences (on and
# off) for a specific light.

from cockpit.gui.gui_utils import FLOATVALIDATOR


class OptoPanel(wx.Panel):
    def __init__(self, parent, light, sequence, doesStartOn):
        super().__init__(parent)
        self.light = light

        ## List of [start, stop] lists, indicating times when the light
        # should be turned on or off.
        self.sequence = sequence

        sizer = wx.BoxSizer(wx.HORIZONTAL)
        sizer.Add(wx.StaticText(self, -1, light.name + ": ", size=(60, -1)))

        button = wx.Button(self, -1, "Clear", size=(80, -1))
        button.Bind(wx.EVT_BUTTON, self.onClear)
        button.SetToolTip(
            wx.ToolTip("Remove all entries for this light source")
        )
        sizer.Add(button)

        button = wx.Button(self, -1, "Delete last", size=(80, -1))
        button.Bind(wx.EVT_BUTTON, self.onDeleteLast)
        button.SetToolTip(
            wx.ToolTip("Remove the last entry for this light source")
        )
        sizer.Add(button)

        self.sequenceText = wx.TextCtrl(
            self, -1, size=(200, -1), style=wx.TE_READONLY
        )
        self.sequenceText.SetToolTip(
            wx.ToolTip(
                "Displays the sequence of on/off actions this light source will perform prior to starting the acquisition."
            )
        )
        sizer.Add(self.sequenceText)

        self.startText = wx.TextCtrl(self, -1, size=(40, -1))
        self.startText.SetToolTip(
            wx.ToolTip("Time, in seconds, at which to start illumination")
        )
        sizer.Add(self.startText)
        self.startText.SetValidator(FLOATVALIDATOR)
        self.startText.allowEmpty = True

        self.stopText = wx.TextCtrl(self, -1, size=(40, -1))
        self.startText.SetToolTip(
            wx.ToolTip("Time, in seconds, at which to stop illumination")
        )
        sizer.Add(self.stopText)
        self.stopText.SetValidator(FLOATVALIDATOR)
        self.stopText.allowEmpty = True

        button = wx.Button(self, -1, "Add", size=(80, -1))
        button.Bind(wx.EVT_BUTTON, self.onAdd)
        button.SetToolTip(
            wx.ToolTip("Add this start/stop pair to the sequence of actions.")
        )
        sizer.Add(button)

        self.isOnDuringAcquisition = wx.CheckBox(self)
        self.isOnDuringAcquisition.SetToolTip(
            wx.ToolTip(
                "If checked, this light source will continuously shine on the sample during acquisition."
            )
        )
        self.isOnDuringAcquisition.SetValue(doesStartOn)
        sizer.Add(self.isOnDuringAcquisition)

        self.SetSizerAndFit(sizer)
        self.updateDisplay()

    def onClear(self, event=None):
        """Clear all illumination settings."""
        self.sequence = []
        self.updateDisplay()

    def onDeleteLast(self, event=None):
        """Delete the last entry in the illumination settings."""
        self.sequence = self.sequence[:-1]
        self.updateDisplay()

    def onAdd(self, event=None):
        """Add a new entry to the illumination settings, merging entries as
        necessary."""
        start = gui_utils.tryParseNum(self.startText, float)
        stop = gui_utils.tryParseNum(self.stopText, float)
        if start >= stop:
            # No-op; easiest way for this to happen is for the stop text to
            # have no value.
            return
        # Go through our list of sequences and see if we need to merge them
        # together.
        self.sequence.append([start, stop])
        indexToDelete = len(self.sequence) - 1
        didChange = True
        while didChange:
            didChange = False
            for i, (altStart, altStop) in enumerate(self.sequence):
                # Check for overlap.
                if (start < altStart and stop > altStart) or (
                    stop > altStop and start < altStop
                ):
                    # Merge the two entries.
                    start = min(start, altStart)
                    stop = max(stop, altStop)
                    self.sequence[i] = [start, stop]
                    didChange = True
                    # Destroy the old sequence as it's been merged with
                    # this one.
                    del self.sequence[indexToDelete]
                    indexToDelete = i
                    break

        # Sort the sequence by start time.
        self.sequence.sort(key=lambda s: s[0])
        self.startText.SetValue("")
        self.stopText.SetValue("")
        self.updateDisplay()

    def updateDisplay(self):
        """Update our textual display of the illumination settings."""
        text = ", ".join(
            ["(%.2f, %.2f)" % (start, stop) for start, stop in self.sequence]
        )
        self.sequenceText.SetValue(text)

    def getSequence(self):
        """Simple accessor."""
        return self.sequence

    def getIsOnDuringAcquisition(self):
        """Simple accessor."""
        return self.isOnDuringAcquisition.GetValue()
