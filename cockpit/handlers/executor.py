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

import collections.abc
import functools
import operator
import time
import typing
from numbers import Number

import matplotlib.pyplot as plt
import wx
from microscope import ElectronicShutteringMode

from cockpit import depot, events, util
from cockpit.experiment import experiment
from cockpit.experiment.action_table import ActionTable
from cockpit.handlers.device import DeviceHandler
from cockpit.handlers.generic_positioner import GenericPositionerHandler


class ExecutorHandler(DeviceHandler):
    """This handler is responsible for executing portions of experiments."""

    def __init__(self, name, group_name, callbacks, digital_lines=None, analog_lines=None):
        """Set up the executor handler.

        callbacks must include the following:

        - examine_actions(name, table): Perform any necessary validation or
          modification of the experiment's ActionTable.
        - execute_table(name, table, start_index, stop_index): Actually perform
          actions through the specified lines in the ActionTable.

        Args:
            name (str): Handler name.
            group_name (str): Handler and device group name.
            callbacks (dict): Callbacks, as above.
            digital_lines (int): Optional, number of digital lines.
            analog_lines (int): Optional, number of analog lines.
        """
        # Note that even though this device is directly involved in running
        # experiments, it is never itself a part of an experiment, so
        # we pass False for is_eligible_for_experiments here.
        super().__init__(name, group_name, False, callbacks, depot.EXECUTOR)
        # Base class contains empty dicts used by mixins so that methods like
        # get_num_runnable_lines can be implemented here for all mixin combos. This
        # works just great, but is probably a horrible abuse of OOP. It might be
        # cleaner to have a single list of clients.
        self.digital_clients = {}
        self.analog_clients = {}
        # Number of digital and analog lines.
        self._digital_lines = digital_lines
        self._analog_lines = analog_lines
        if not isinstance(self, DigitalMixin):
            self.register_digital = self._raise_no_digital_exception
            self.get_digital = self._raise_no_digital_exception
            self.set_digital = self._raise_no_digital_exception
            self.read_digital = self._raise_no_digital_exception
            self.write_digital = self._raise_no_digital_exception
            self.trigger_digital = self._raise_no_digital_exception
        if not isinstance(self, AnalogMixin):
            self.register_analog = self._raise_no_analog_exception
            self.set_analog = self._raise_no_analog_exception
            self.get_analog = self._raise_no_analog_exception
            self.set_analog_client = self._raise_no_analog_exception
            self.get_analog_client = self._raise_no_analog_exception
        events.subscribe(
            events.PREPARE_FOR_EXPERIMENT, self.on_prepare_for_experiment
        )
        events.subscribe(
            events.CLEANUP_AFTER_EXPERIMENT, self.cleanup_after_experiment
        )

    def examine_actions(self, table):
        return self.callbacks["examine_actions"](table)

    def get_num_runnable_lines(self, table, index):
        """Return number of lines this handler can run."""
        count = 0
        for time, handler, parameter in table[index:]:
            # Check for analog and digital devices we control.
            if (
                handler is not self
                and handler not in self.digital_clients
                and handler not in self.analog_clients
            ):
                # Found a device we don't control.
                break
            count += 1
        return count

    def _raise_no_digital_exception(self, *args, **kwargs):
        raise Exception("Digital lines not supported.")

    def _raise_no_analog_exception(self, *args, **kwargs):
        raise Exception("Analog lines not supported.")

    def execute_table(self, table, start_index, stop_index, num_reps, rep_duration):
        """Run a portion of a table describing the actions to perform in a
        given experiment.

        Args:
            table (ActionTable): An ActionTable instance.
            start_index (int): Index of the first entry in the table to run.
            stop_index (int): Index of the entry before which we stop (i.e. it
                is not performed).
            num_reps (int): Number of times to iterate the execution.
            rep_duration (float): Amount of time to wait between reps, or None
                for no wait time.
        """
        # The actions between start_index and stop_index may include actions for
        # this handler, or for this handler's clients. All actions are
        # ultimately carried out by this handler, so we need to parse the
        # table to replace client actions, resulting in a table of
        # (time, (analogStage, digitalState)).
        if isinstance(self, DigitalMixin):
            digital_state = self.read_digital()
        else:
            digital_state = None
        if isinstance(self, AnalogMixin):
            analog_state = [self.get_analog_line(line) for line in range(self._analog_lines)]
        else:
            analog_state = None

        actions = []

        t_prev = None
        h_prev = None
        args_prev = None

        for i in range(start_index, stop_index):
            t, h, args = table[i]
            if h in self.analog_clients:
                # update analog state
                line_handler = self.analog_clients[h]
                if isinstance(args, collections.abc.Iterable):
                    # Using an indexed position
                    pos = line_handler.indexed_position(*args)
                else:
                    pos = args
                analog_state[line_handler.line] = line_handler.pos_to_native(pos)
            elif h in self.digital_clients:
                # set/clear appropriate bit
                change = 1 << self.digital_clients[h]
                # args contains new bit state
                if args:
                    digital_state |= change
                else:
                    digital_state = digital_state & (2 ** self._digital_lines - 1) - (change)

            # Check for simultaneous actions.
            if t_prev is not None and t == t_prev:
                if h not in h_prev:
                    # Update last action to merge actions at same timepoint.
                    actions[-1] = (t, (digital_state, analog_state[:]))
                    # Add handler and args to list for next check.
                    h_prev.append(h)
                    args_prev.append(args)
                elif args == args_prev[h_prev.index(h)]:
                    # Just a duplicate entry
                    continue
                else:
                    # Simultaneous, different actions with same handler.
                    raise Exception(
                        "Simultaneous actions with same hander, %s." % h
                    )
            else:
                # Append new action.
                actions.append((t, (digital_state, analog_state[:])))
                # Reinitialise h_prev and args_prev for next check.
                h_prev, args_prev = [h], [args]
                t_prev = t

        events.publish(
            events.UPDATE_STATUS_LIGHT,
            "device waiting",
            "Waiting for %s to finish" % self.name,
        )

        return self.callbacks["execute_table"](
            actions, 0, len(actions), num_reps, rep_duration
        )

    def show_debug_window(self):
        """Debugging function: display ExecutorOutputWindow."""
        # Ensure only a single instance of the window.
        global _window_instance
        window = globals().get("_window_instance")
        if window:
            try:
                window.Raise()
                return None
            except:
                pass
        # If we get this far, we need to create a new window.
        _window_instance = ExecutorDebugWindow(
            self, parent=wx.GetApp().MainWindow
        )
        _window_instance.Show()

    def on_prepare_for_experiment(self, experiment):
        # This smells sketchy, but does exactly what we need: run
        # the method on all mixins contributing to a hybrid class.
        # Could do achieve the same by having mixins append to a list
        # of actions to call on certain events.
        for c in self.__class__.__mro__[1:]:
            if hasattr(c, "_on_prepare_for_experiment"):
                c._on_prepare_for_experiment(self)

    def cleanup_after_experiment(self, is_cleanup_final=True):
        # See comments in on_prepare_for_experiment
        for c in self.__class__.__mro__[1:]:
            if hasattr(c, "_cleanup_after_experiment"):
                c._cleanup_after_experiment(self, is_cleanup_final)


class DigitalMixin:
    """Digital handler mixin."""

    def register_digital(self, client, line):
        """Register a client device that is connected to one of our lines.
        returns a virtual TriggerProxy."""
        h = TriggerProxy(client.name, self)
        self.digital_clients[h] = int(line)
        # If the client not a DelegateTrigger executor, all of its actions
        # are handled here.
        if not isinstance(client, DelegateTrigger):
            self.digital_clients[client] = int(line)
        return h

    def set_digital(self, line, state):
        """Set or clear a single line."""
        if line is None:
            return
        line = int(line)
        if self.callbacks.get("set_digital", None):
            self.callbacks["set_digital"](line, state)
        else:
            old_state = self.read_digital()
            if state:
                new_state = old_state | 1 << line
            else:
                new_state = old_state & (2**self._digital_lines - 1) - (1 << line)
            self.write_digital(new_state)

    def write_digital(self, state):
        self.callbacks["write_digital"](state)

    def read_digital(self):
        return self.callbacks["read_digital"]()

    def trigger_digital(self, client, dt=0.01):
        """Trigger a client line now."""
        line = self.digital_clients.get(client, None)
        if line:
            self.set_digital(line, True)
            time.sleep(dt)
            self.set_digital(line, False)

    @property
    def active_lights(self):
        return list(
            filter(
                lambda h: h.device_type == depot.LIGHT_TOGGLE
                          and h.get_is_enabled(),
                self.digital_clients,
            )
        )

    @property
    def active_cameras(self):
        return list(
            filter(
                lambda h: h.device_type == depot.CAMERA and h.get_is_enabled(),
                self.digital_clients,
            )
        )

    def take_image(self):
        if not self.digital_clients:
            # No triggered devices registered.
            return
        cam_lines = sum(
            [1 << self.digital_clients[cam] for cam in self.active_cameras]
        )

        # We want to know if there are cameras using rolling shutter. If so, cameras must be triggered in advance
        exposure_start_time = [
            cam.get_time_between_exposures()
            for cam in self.active_cameras
            if cam.get_shuttering_mode() == ElectronicShutteringMode.ROLLING
        ]
        exposure_start_time = max(exposure_start_time, default=0)

        if cam_lines == 0:
            # No cameras to be triggered.
            return

        light_time_pairs = []
        for light in self.active_lights:
            light_line = 1 << self.digital_clients[light]
            light_time = light.get_exposure_time()
            light_time_pairs.append((light_line, light_time))

        # Sort by exposure time
        light_time_pairs.sort(key=lambda item: item[1])

        # Generate a sequence of (time, digital state)
        # TODO: currently uses bulb exposure; should support other modes.
        if light_time_pairs:
            # Start by all active cameras and lights.
            state = cam_lines | functools.reduce(
                operator.ior, list(zip(*light_time_pairs))[0]
            )
            if exposure_start_time != 0:
                seq = [(0, cam_lines)]
            seq.append((exposure_start_time, state))
            # Switch off each light as its exposure time expires. If exposure times are equal, keep only the last one.
            for light_line, light_time in light_time_pairs:
                state -= light_line
                if (exposure_start_time + light_time) == seq[-1][0]:
                    seq[-1] = (exposure_start_time + light_time, state)
                else:
                    seq.append((exposure_start_time + light_time, state))
        else:
            # No lights. Just trigger the cameras.
            seq = [(0, cam_lines)]

        # If there is an ambient light enabled, extend exposure as
        # necessary (see issue #669).
        ambient = depot.get_handler_with_name("Ambient")
        if ambient is not None and ambient.get_is_enabled():
            t = ambient.get_exposure_time()
            if t > seq[-1][0]:
                seq.append((ambient.get_exposure_time(), 0))

        # Switch all lights and cameras off.
        seq.append((seq[-1][0] + 1, 0))
        if self.callbacks.get("runSequence", None):
            self.callbacks["runSequence"](seq)
        else:
            self.soft_sequence(seq)

    def write_with_mask(self, mask, state):
        initial = self.read_digital()
        final = (initial & ~mask) | state
        self.write_digital(final)

    @util.threads.call_in_new_thread
    def soft_sequence(self, seq):
        # Mask of the bits that we toggle
        mask = functools.reduce(operator.ior, list(zip(*seq))[1])
        entry_state = self.read_digital()
        t_last = 0
        for t, state in seq:
            if t != t_last:
                time.sleep((t - t_last) / 1000.0)
                t_last = t
            self.write_with_mask(mask, state)
        self.write_digital(entry_state)


class AnalogMixin:
    """Analog handler mixin.

    Consider output 'level' in volts, amps or ADUS, and input
    'position' in experimental units (e.g. um or deg).
    level = gain * (offset + position)
    gain is in units of volts, amps or ADUS per experimental unit.
    offset is in experimental units.
    """

    def register_analog(
        self, client, line, offset=0, gain=1, movement_time_func=None
    ):
        """Register a client device that is connected to one of our lines.

        Returns an AnalogLineHandler for that line.
        """
        h = AnalogLineHandler(
            client.name,
            self.name + " analogs",
            self,
            int(line),
            offset,
            gain,
            movement_time_func,
        )
        # May reference the client by whatever we were passed or its new handler
        self.analog_clients[client] = h
        self.analog_clients[h] = h
        return h

    def set_analog_line(self, line, level):
        """Set analog output of line to level."""
        self.callbacks["set_analog"](line, level)

    def get_analog_line(self, line):
        """Get level of analog line."""
        return self.callbacks["get_analog"](line)

    def _on_prepare_for_experiment(self):
        for client in self.analog_clients:
            self.analog_clients[client].save_position()

    def _cleanup_after_experiment(self, is_cleanup_final=True):
        if is_cleanup_final:
            for client in self.analog_clients:
                self.analog_clients[client].restore_position()


class AnalogLineHandler(GenericPositionerHandler):
    def __init__(
        self, name, group_name, analog_source, line, offset, gain, movement_time_func
    ):
        """A type of GenericPositioner for analog outputs.

        Handles absolute and indexed positions in action table. absolute:
        time, handler, float or int indexed:    time, handler, (index,
        wavelength or None or 'default')
        """
        super().__init__(name, group_name, True, {})
        # Indexed positions. Can be a dict if wavelength-independent, or
        # a mapping of wavelengths (as floats or ints) to lists of same length.
        self.positions = []
        # Scaling parameters
        self.gain = gain
        self.offset = offset
        # Line, required when executing table.
        self.line = line
        # Saved position
        self._saved_pos = None
        # Set up callbacks used by GenericPositionHandler methods.
        self.callbacks = {}
        self.callbacks["move_absolute"] = lambda pos: analog_source.set_analog_line(
            line, self.pos_to_native(pos)
        )
        self.callbacks["get_position"] = lambda: self.native_to_pos(
            analog_source.get_analog_line(line)
        )
        ## TODO - consider if we want to fallback to number or zero, or raise an exception here.
        if callable(movement_time_func):
            self.callbacks["get_movement_time"] = movement_time_func
        elif isinstance(movement_time_func, Number):
            self.callbacks["get_movement_time"] = lambda *args: (
                movement_time_func,
                0,
            )
        else:
            self.callbacks["get_movement_time"] = lambda *args: (0, 0)
        super().__init__(name, group_name, True, self.callbacks)

    def save_position(self):
        self._saved_pos = self.get_position()

    def restore_position(self):
        self.move_absolute(self._saved_pos)

    def move_relative(self, delta):
        self.callbacks["move_absolute"](self.callbacks["get_position"]() + delta)

    def pos_to_native(self, pos):
        return self.gain * (self.offset + pos)

    def native_to_pos(self, native):
        return (native / self.gain) - self.offset

    def indexed_position(self, index, wavelength=None):
        pos = None
        if isinstance(wavelength, Number) and isinstance(self.positions, dict):
            wls = [
                int(wl)
                for wl in self.positions
                if wl and isinstance(int(wl), Number)
            ]
            wl = min(wls, key=lambda w: abs(w - wavelength))
            ps = self.positions[str(wl)]
        elif isinstance(self.positions, dict):
            if None in self.positions:
                ps = self.positions[None]
            elif "default" in self.positions:
                ps = self.positions["default"]
            else:
                raise Exception(
                    "No wavelength specified, and no default in indexed positions."
                )
        else:
            ps = self.positions
        return ps[index]


class DigitalExecutorHandler(DigitalMixin, ExecutorHandler):
    pass


class AnalogExecutorHandler(AnalogMixin, ExecutorHandler):
    pass


class AnalogDigitalExecutorHandler(AnalogMixin, DigitalMixin, ExecutorHandler):
    pass


def plot_action_table_profile(
    action_table: ActionTable,
    handlers: typing.Optional[typing.List[DeviceHandler]] = None,
) -> None:
    """Plot the timing profile of an action table like an oscilloscope display.

    Args:
        action_table: the action table to plot.
        handlers: if not None, only plot actions for the given
            handlers.
    """
    # We first construct a more usable table
    table = {}

    def action_type(action):
        if isinstance(action, bool):
            return "digital"
        elif isinstance(action, (float, int)):
            return "analog"
        else:
            return None

    for event in action_table.actions:
        if event is None:
            continue
        time, handler, action = event
        if handlers is None or handler in handlers:
            if handler.name in table:
                # The handler already exists in the table
                if table[handler.name][2] == "digital":
                    # if digital have a point with last state at this time.
                    table[handler.name][0].append(time)
                    table[handler.name][1].append(table[handler.name][1][-1])
                table[handler.name][0].append(time)
                table[handler.name][1].append(action)
            else:
                table[handler.name] = ([time], [action], action_type(action))

    # Generate the actual plot
    fig, axs = plt.subplots(len(table), 1, sharex=True)
    fig.subplots_adjust(hspace=0)  # Remove horizontal space between axes
    for i, (key, data) in enumerate(table.items()):
        if data[2] == "analog":
            axs[i].plot(data[0], data[1])
        elif data[2] == "digital":
            axs[i].step(data[0], data[1], where="post")
            axs[i].set_yticks([0, 1])
            axs[i].set_ylim(-0.1, 1.1)
        else:
            continue
        axs[i].set_xlabel("Time", fontsize=12)
        axs[i].set_ylabel(key, fontsize=12)
        axs[i].grid(True, which="both", axis="x")

    plt.show(block=False)


class ExecutorDebugWindow(wx.Frame):
    """This debugging window allows manipulation of analog and digital lines."""

    def __init__(self, handler, parent, *args, **kwargs):
        title = handler.name + " Executor control lines"
        kwargs["style"] = (
            wx.SYSTEM_MENU | wx.CAPTION | wx.CLOSE_BOX | wx.CLIP_CHILDREN
        )
        super().__init__(parent, title=title, *args, **kwargs)
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)

        ## Maps buttons to their lines.
        self.buttonToLine = {}

        if handler._digital_lines is not None:
            # Digital controls
            nr_cols = 8
            nr_rows = (handler._digital_lines + nr_cols - 1) // nr_cols
            button_sizer = wx.GridSizer(nr_rows, nr_cols, 1, 1)
            for line in range(handler._digital_lines):
                clients = [
                    k.name
                    for k, v in handler.digital_clients.items()
                    if v == line
                ]
                if clients:
                    label = "\n".join(clients)
                else:
                    label = str(line)
                button = wx.ToggleButton(panel, wx.ID_ANY, label)
                button.Bind(
                    wx.EVT_TOGGLEBUTTON,
                    lambda evt, line=line: handler.set_digital(
                        line, evt.EventObject.Value
                    ),
                )
                button_sizer.Add(button, 1, wx.EXPAND)
            main_sizer.Add(button_sizer)

            # Analog controls
            # These controls deal with hardware units, i.e. probably ADUs.
            analog_sizer = wx.BoxSizer(wx.HORIZONTAL)
            for line in range(handler._analog_lines):
                analog_sizer.Add(wx.StaticText(panel, -1, "output %d:" % line))
                control = wx.TextCtrl(
                    panel, -1, size=(60, -1), style=wx.TE_PROCESS_ENTER
                )
                control.Bind(
                    wx.EVT_TEXT_ENTER,
                    lambda evt, line=line, ctrl=control: handler.set_analog_line(
                        line, float(ctrl.GetValue())
                    ),
                )
                # If dealing with ADUs, float should perhaps be int,
                # but rely on device to set correct type.
                analog_sizer.Add(control, 0, wx.RIGHT, 20)

            btn = wx.Button(panel, label="Display last experiment")
            btn.SetToolTip(
                wx.ToolTip(
                    "Plot the last experiment like an oscilloscope display."
                )
            )
            btn.Bind(wx.EVT_BUTTON, self._on_display_last_experiment)
            analog_sizer.Add(btn, 0, wx.RIGHT, 20)
            main_sizer.Add(analog_sizer)

        panel.SetSizerAndFit(main_sizer)
        self.SetClientSize(panel.GetSize())

    def _on_display_last_experiment(self, evt: wx.CommandEvent) -> None:
        del evt
        if experiment.last_experiment is None:
            wx.MessageBox("No experiment has been done yet.")
        else:
            plot_action_table_profile(experiment.last_experiment.table)


class SimpleExecutor(DeviceHandler):
    """A class for a handler that can perform actions in an experiment, but
    doesn't have any analog or digital capabilities."""

    def __init__(self, name, group_name, is_eligible_for_experiments, callbacks):
        super().__init__(
            name,
            group_name,
            is_eligible_for_experiments,
            callbacks,
            depot.EXECUTOR,
        )
        for cb_name, cb in callbacks.items():
            if cb_name == "execute_table":
                continue
            if not callable(cb):
                cb = lambda *args, **kwargs: cb
            self.__setattr__(cb_name, cb)

    def execute_table(self, table, start_index, stop_index, num_reps, rep_duration):
        """Execute whatever the device does and publish an event on completion."""
        self.callbacks["execute_table"](
            table, start_index, stop_index, num_reps, rep_duration
        )
        events.publish(events.EXPERIMENT_EXECUTION)

    def get_num_runnable_lines(self, table, index):
        """Return number of lines this handler can run."""
        count = 0
        if not self.is_eligible_for_experiments:
            return 0
        for time, handler, parameter in table[index:]:
            if handler is not self:
                break
            count += 1
        return count


class TriggerProxy(DeviceHandler):
    """A trigger handler to allow discrimination of trigger and triggered
    device in the action table."""

    def __init__(self, name, trig_source):
        super().__init__(
            name + " trigger", name + " group", False, {}, depot.GENERIC_DEVICE
        )
        self.triggerNow = lambda: trig_source.trigger_digital(self)


## This handler can examine and modify an action table, but delegates
# running it to some other ExecutorHandler.
# The DelegateTrigger instance should be created with any non-trigger actions
# handled in callback named 'execute_table', and then connected to a trigger
# source with the delegate_to method.


class DelegateTrigger(SimpleExecutor):
    def __init__(self, name, group_name, is_eligible_for_experiments, callbacks):
        """def __init__(self, name, group_name, trig_source, trig_line,
        examine_actions, movementTime=0):."""
        super().__init__(name, group_name, is_eligible_for_experiments, callbacks)
        self._trigger = None
        self._trigger_time = 0
        self._response_time = 0
        self.trigger_now = lambda: None

    def delegate_to(self, trig_source, trig_line, trig_time=0, response_time=0):
        """Delegate trigger actions to some trig_source."""
        if isinstance(trig_source, str):
            trig_source = depot.get_handler(trig_source, depot.EXECUTOR)
        self._trigger = trig_source.register_digital(self, trig_line)
        self.trigger_now = self._trigger.triggerNow
        self._trigger_time = trig_time
        self._response_time = response_time

    def add_toggle(self, time, table):
        """Add a toggle event to the action table.

        Return time of last action, and response time before ready after
        trigger.
        """
        dt = max(self._trigger_time, table.toggle_time)
        table.add_action(time, self._trigger, True)
        table.add_action(time + dt, self._trigger, False)
        return time + dt, self._response_time
