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

"""Boulder SLM."""

import decimal
import time
from itertools import groupby
from threading import Lock

import Pyro4
import wx

import cockpit.gui.device
import cockpit.gui.dialogs.get_number_dialog
import cockpit.gui.gui_utils
import cockpit.handlers.executor
import cockpit.util
from cockpit import events
from cockpit.devices import device


class _LastParameters:
    """A class to keep a record of last SIM parmeters using async calls."""

    def __init__(self, slm):
        self.slm = slm
        self._params = None
        self._result = None
        self._lock = Lock()

    @property
    def params(self):
        """Return the last recorded SIM parameters."""
        if self._result and self._result.ready:
            # Updated parameters are available.
            with self._lock:
                try:
                    self._params = self._result.value
                except:
                    pass
                finally:
                    self._result = None
        return self._params

    @params.setter
    def params(self, value):
        """Set recorded parameters explicitly."""
        with self._lock:
            self._result = None
            self._params = value

    def update(self):
        """Dispatch async call to update record from hardware."""
        if self.slm.asproxy:
            self._result = self.slm.asproxy.get_sim_sequence()


class BoulderSLM(device.Device):
    """Boulder SLM device.

    Sample config entry:

    .. code:: ini

        [slm]
        type: cockpit.devices.boulder_slm.BoulderSLM
        uri: PYRO:pyroSLM@slmhost:8000
        triggerSource: NAME_OF_EXECUTOR_DEVICE
        triggerLine: 2

    """

    _config_types = {
        "settlingtime": float,
        "triggerLine": int,
    }

    def __init__(self, name, config={}):
        super().__init__(name, config)
        self.connection = None
        self.asproxy = None
        self.position = None
        self.was_powered = None
        self.slm_timeout = 10
        self.slm_retry_limit = 3
        self.last = _LastParameters(self)

    def initialize(self):
        if self.uri:
            uri = self.uri
        else:
            uri = "PYRO:pyroSLM@%s:%d" % (self.ip_address, self.port)
        self.connection = Pyro4.Proxy(uri)
        self.asproxy = Pyro4.Proxy(uri)
        self.asproxy._pyroAsync()
        # If there's a diffraction angle in the config, set it on the remote.
        angle = self.config.get("diffractionangle", None)
        if angle:
            self.connection.set_sim_diffraction_angle(angle)

    def on_exit(self) -> None:
        for proxy in [self.connection, self.asproxy]:
            if proxy is not None:
                proxy._pyroRelease()
        super().on_exit()

    def get_is_enabled(self):
        return self.connection.get_is_enabled()

    def set_enabled(self, state):
        """Enable or disable the SLM."""
        if state:
            # Enable.
            if self.last.params == self.connection.get_sim_sequence():
                # Hardware and software sequences match
                target_position = self.get_current_position()
            else:
                target_position = 0
            # Enable the hardware.
            self.connection.run()
            # Often, after calling connection.run(), the SLM pattern and the image
            # index reported are not synchronised until a few triggers have been
            # sent, so we need to compensate for this. We do the best we can,
            # but any other trigger-device activity during this call can mean that
            # we miss the target frame by +/- 1.
            for i in range(3):
                self.handler.trigger_now()
                time.sleep(0.01)
            # Cycle to the target position.
            self.cycle_to_position(target_position)
        else:
            # Disable the SLM.
            self.connection.stop()

    def cycle_to_position(self, target_position):
        pos = self.get_current_position()
        delta = (target_position - pos) + (target_position < pos) * len(
            self.last.params
        )
        for i in range(delta):
            self.handler.trigger_now()
            time.sleep(0.01)

    def execute_table(
        self, table, start_index, stop_index, num_reps, rep_duration
    ):
        # Found a table entry with a simple index. Trigger until that index
        # is reached.
        for t, h, args in table[start_index:stop_index]:
            events.publish(
                events.UPDATE_STATUS_LIGHT,
                "device waiting",
                "SLM moving to index %d" % args,
            )
            self.cycle_to_position(args)

    def examine_actions(self, table):
        # Extract pattern parameters from the table.
        # pattern_parms is a list of tuples (angle, phase, wavelength)
        pattern_params = [row[2] for row in table if row[1] is self.handler]
        if not pattern_params:
            # SLM is not used in this experiment.
            return

        # Remove consecutive duplicates and position resets.
        reduced_params = [
            p[0] for p in groupby(pattern_params) if type(p[0]) is tuple
        ]
        # Find the repeating unit in the sequence.
        sequence_length = len(reduced_params)
        for length in range(2, len(reduced_params) // 2):
            if reduced_params[0:length] == reduced_params[length : 2 * length]:
                sequence_length = length
                break
        sequence = reduced_params[0:sequence_length]
        ## Tell the SLM to prepare the pattern sequence.
        async_result = self.asproxy.set_sim_sequence(sequence)

        # Track sequence index set by last set of triggers.
        last_index = 0
        for i, (t, handler, action) in enumerate(table.actions):
            if handler is not self.handler:
                # Nothing to do
                continue
            # Action specifies a target frame in the sequence.
            # Remove original event.
            table[i] = None
            # How many triggers?
            if type(action) is tuple and action != sequence[last_index]:
                # Next pattern does not match last, so step one pattern.
                num_triggers = 1
            elif type(action) is int:
                if action >= last_index:
                    num_triggers = action - last_index
                else:
                    num_triggers = sequence_length - last_index - action
            else:
                num_triggers = 0
            """
            Used to calculate time to execute triggers and settle here, 
            then push back all later events, but that leads to very long
            delays before the experiment starts. For now, comment out
            this code, and rely on a fixed time passed back to the action
            table generator (i.e. experiment class).

            # How long will the triggers take?
            # Time between triggers must be > table.toggleTime.
            ## Shift later table entries to allow for triggers and settling.
            table.shift_actions_back(time, dt)
            for trig in range(num_triggers):
                t = table.add_toggle(t, triggerHandler)
                t += table.toggleTime
            """
            for trig in range(num_triggers):
                t = table.add_toggle(t, self.handler)
                t += table.toggle_time

            last_index += num_triggers
            if last_index >= sequence_length:
                last_index = last_index % sequence_length
        table.clear_bad_entries()
        # Wait until SLM has finished generating and loading patterns.
        self.wait(async_result, "SLM is generating pattern sequence.")
        # Store the parameters used to generate the sequence.
        self.last.params = sequence
        self.connection.run()
        # Fire several triggers to ensure that the sequence is loaded.
        for i in range(12):
            self.handler.trigger_now()
            time.sleep(0.01)
        # Ensure that we're at position 0.
        self.cycle_to_position(0)
        self.position = self.get_current_position()

    def get_current_position(self):
        return self.connection.get_sequence_index()

    def get_handlers(self):
        trigsource = self.config.get("triggersource", None)
        trigline = self.config.get("triggerline", None)
        dt = decimal.Decimal(self.config.get("settlingtime", 10))
        result = []
        self.handler = cockpit.handlers.executor.DelegateTrigger(
            "slm",
            "slm group",
            True,
            {
                "examine_actions": self.examine_actions,
                "get_movement_time": lambda *args: dt,
                "execute_table": self.execute_table,
                "set_enabled": self.set_enabled,
                "get_is_enabled": self.get_is_enabled,
            },
        )
        self.handler.delegate_to(trigsource, trigline, 0, dt)
        result.append(self.handler)
        return result

    def make_ui(self, parent):
        """UI functions"""
        panel = wx.Panel(parent, style=wx.BORDER_RAISED)
        panel.SetDoubleBuffered(True)
        panel.Sizer = wx.BoxSizer(wx.VERTICAL)
        power_button = cockpit.gui.device.EnableButton(panel, self.handler)
        panel.Sizer.Add(power_button, 0, wx.EXPAND)
        trigger_button = wx.Button(panel, label="step")
        trigger_button.Bind(
            wx.EVT_BUTTON, lambda evt: self.handler.trigger_now()
        )
        panel.Sizer.Add(trigger_button, 0, wx.EXPAND)
        # Add a position display.
        pos_display = cockpit.gui.device.MultilineDisplay(
            parent=panel, numLines=3
        )
        pos_display.Bind(
            wx.EVT_TIMER, lambda event: self.update_position_display(event)
        )
        panel.Sizer.Add(pos_display)
        # Set up a timer to update value displays.
        self.update_timer = wx.Timer(pos_display)
        self.update_timer.Start(1000)
        # ensure that we bind the window destroy event to stop timer
        self.Bind(wx.EVT_WINDOW_DESTROY, self.on_destroy)
        # Changed my mind. SIM diffraction angle is an advanced parameter,
        # so it now lives in a right-click menu rather than on a button.
        panel.Bind(wx.EVT_CONTEXT_MENU, self.on_right_mouse)
        # Controls other than power_button only enabled when SLM is enabled.
        trigger_button.Disable()
        pos_display.Disable()
        power_button.manageStateOf((trigger_button, pos_display))
        return panel

    def on_destroy(self, event: wx.WindowDestroyEvent) -> None:
        """ensure that the timer is stopped on window destroy."""
        self.update_timer.Stop()
        event.Skip()

    def update_position_display(self, event):
        base_str = "angle:\t%s\nphase:\t%s\nwavel.:\t%s"
        # Get the display object. It seems there is variation between
        # wx versions. With some versions, the display is obtained by
        #    event.GetEventObject().
        # With others, it is
        #    event.GetEventObject().GetOwner()
        display = event.GetEventObject()
        if not hasattr(display, "SetLabel"):
            display = display.GetOwner()
        self.position = self.get_current_position()
        try:
            parms = self.last.params[self.position]
        except (IndexError, TypeError):
            # SLM parms updated since last position fetched, or lastParms is None.
            self.last.update()
            parms = None
        if parms:
            display.SetLabel(base_str % parms)

    def on_prepare_for_experiment(self, *args):
        self.position = self.get_current_position()
        self.was_powered = self.get_is_enabled()

    def cleanup_after_experiment(self, *args):
        if not self.was_powered:
            self.set_enabled(False)

    def perform_subscriptions(self):
        # events.subscribe(events.USER_ABORT, self.on_abort)
        events.subscribe(
            events.PREPARE_FOR_EXPERIMENT, self.on_prepare_for_experiment
        )
        events.subscribe(
            events.CLEANUP_AFTER_EXPERIMENT, self.cleanup_after_experiment
        )

    def wait(self, async_result, message):
        # Wait unti the SLM has finished an aynchronous task.
        status = wx.ProgressDialog(
            parent=wx.GetApp().MainWindow,
            title="Waiting for SLM",
            message=message,
        )
        status.Show()
        slm_fail_count = 0
        slm_fail = False
        while not async_result.wait(timeout=self.slm_timeout) and not slm_fail:
            slm_fail_count += 1
            if slm_fail_count >= self.slm_retry_limit:
                slm_fail = True
        status.Destroy()
        if slm_fail:
            raise Exception("SLM timeout.")

    def on_right_mouse(self, event):
        menu = wx.Menu()
        for label, method in [
            ("Generate SIM sequence", self.test_sim_sequence),
            ("SIM diff. angle", self.set_diffraction_angle),
        ]:
            menu_item = menu.Append(wx.ID_ANY, item=label)
            menu.Bind(wx.EVT_MENU, lambda evt: method, menu_item)
        cockpit.gui.gui_utils.place_menu_at_mouse(event.GetEventObject(), menu)

    def test_sim_sequence(self):
        inputs = cockpit.gui.dialogs.get_number_dialog.get_many_numbers_from_user(
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
        ## Tell the SLM to prepare the pattern sequence.
        async_result = self.asproxy.set_sim_sequence(params)
        self.wait(async_result, "SLM is generating pattern sequence.")
        self.last.update()

    def set_diffraction_angle(self):
        try:
            theta = self.connection.get_sim_diffraction_angle()
        except:
            raise Exception("Could not communicate with SLM service.")
        new_theta = float(
            cockpit.gui.dialogs.get_number_dialog.getNumberFromUser(
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
        self.connection.set_sim_diffraction_angle(new_theta)
