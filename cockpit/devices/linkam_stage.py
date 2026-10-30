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

"""Linkam stages."""

import datetime
import operator
import threading
import time

import Pyro4
import wx

import cockpit.gui.device
import cockpit.gui.gui_utils
import cockpit.handlers.stage_positioner
import cockpit.util.threads
from cockpit import events
from cockpit.devices.device import Device
from cockpit.devices.microscope_device import MicroscopeBase
from cockpit.util import value_logger


DEFAULT_LIMITS = ((0, 0), (11000, 3000))
LOGGING_PERIOD = 30


class RefillTimerPanel(wx.Panel):
    def __init__(self, *args, **kwargs):
        self._refill_func = None

        label_text = kwargs.pop("label", "")
        kwargs["style"] = kwargs.get("style", 0) | wx.BORDER_SIMPLE
        super().__init__(*args, **kwargs)
        self.Sizer = wx.BoxSizer(wx.VERTICAL)
        label = wx.StaticText(
            self, wx.ID_ANY, label=label_text, style=wx.ALIGN_CENTRE_HORIZONTAL
        )
        # Create controls using label=self.format to set correct width.
        self.filling = wx.StaticText(
            self,
            wx.ID_ANY,
            label=self.format(None),
            style=wx.ALIGN_CENTRE_HORIZONTAL | wx.ST_NO_AUTORESIZE,
        )
        self.previous = wx.StaticText(
            self, wx.ID_ANY, label=self.format(None), style=wx.ST_NO_AUTORESIZE
        )
        self.current = wx.StaticText(
            self, wx.ID_ANY, label=self.format(None), style=wx.ST_NO_AUTORESIZE
        )
        [
            self.Sizer.Add(o, flag=wx.ALL | wx.EXPAND, border=2)
            for o in (label, self.previous, self.current, self.filling)
        ]
        self.Bind(wx.EVT_CONTEXT_MENU, self.on_context_menu)
        self.ToolTip = wx.ToolTip(
            "dt: last cycle time.\nt+: time since last refill\nRight click to refill."
        )
        [c.Unbind(wx.EVT_MOTION) for c in self.Children]

    def set_refill_func(self, f):
        self._refill_func = f

    def on_context_menu(self, evt):
        menu = wx.Menu()
        menu.Append(1, "Start refill")
        self.Bind(wx.EVT_MENU, lambda e: self._refill_func(), id=1)
        cockpit.gui.gui_utils.place_menu_at_mouse(self, menu)

    def format(self, dt, prefix=""):
        prefix = "{:3.3} ".format(prefix)
        if dt is None:
            return prefix + "XX --:--:--"
        elif isinstance(dt, datetime.timedelta):
            dt = dt.total_seconds()
        mm, ss = divmod(dt, 60)
        hh, mm = divmod(mm, 60)
        return prefix + ("%.2d:%.2d:%.2d" % (hh, mm, ss))

    def do_update(self, refill):
        # Default colours for current timer display. May be modified before
        # being set at the end of this call.
        bg = wx.SystemSettings.GetColour(wx.SYS_COLOUR_WINDOW)
        fg = wx.SystemSettings.GetColour(wx.SYS_COLOUR_WINDOWTEXT)
        if refill is None:
            self.previous.SetLabel(self.format(None, "dt"))
            self.current.SetLabel(self.format(None, "t+"))
        else:
            # Counters
            t_last = refill.get("last")  # May be None
            prev = refill.get(
                "between_last"
            ).total_seconds()  # Always datetime.timedelta
            self.previous.SetLabel(self.format(prev, "dt"))
            if t_last is None:
                self.current.SetLabel(self.format(None, "t+"))
            else:
                t = (datetime.datetime.now() - t_last).total_seconds()
                self.current.SetLabel(self.format(t, "t+"))
                if prev > 0:
                    if (prev - t) <= 60:
                        # 1 minute left
                        bg = wx.Colour("red")
                        fg = wx.Colour("white")
                    elif (prev - t) < 300:
                        # 5 minutes left
                        bg = wx.Colour("yellow")
                        fg = wx.Colour("black")
            # Refill indicator
            if refill.get("refilling", False):
                self.filling.SetLabel("REFILLING")
                bg = wx.Colour("red")
                fg = wx.Colour("white")
            else:
                self.filling.SetLabel(" ")
        if bg != self.GetBackgroundColour():
            self.SetBackgroundColour(bg)
            for c in self.GetChildren():
                c.SetForegroundColour(fg)
            self.Refresh()


class LinkamStage(MicroscopeBase, Device):
    """Cockpit-side module for Linkam stages.

    Tested with CMS196.  config uses following parameters:

    ``type``
        Needs to be ``cockpit.devices.linkam_stage.LinkamStage``.
    ``uri``
        URI of device server.

    """

    _temperature_names = ("bridge", "dewar", "chamber", "base")
    _refill_names = ("sample", "external")

    def __init__(self, name, config={}):
        super().__init__(name, config)
        ## Connection to the XY stage controller (serial.Serial instance).
        self._proxy = Pyro4.Proxy(config.get("uri"))
        ## Lock around sending commands to the XY stage controller.
        self.xy_lock = threading.Lock()
        ## Cached copy of the stage's position.
        self.position_cache = (None, None)
        ## Target positions for movement in X and Y.
        self.motion_targets = [None, None]
        ## Flag to show that send_position_updates is running.
        self.sending_position_updates = False
        ## Status dict updated by remote.
        self.status = {}
        ## Keys for status items that should be logged
        self.logger = value_logger.ValueLogger(
            name, keys=list(map("t_".__add__, self._temperature_names))
        )
        try:
            xlim = self._proxy.get_value_limits("MotorSetpointX")
            ylim = self._proxy.get_value_limits("MotorSetpointY")
        except:
            xlim, ylim = zip(*DEFAULT_LIMITS)
        # _proxy may return (0,0) if it can't query the hardware.
        if not any(xlim):
            xlim, _ = zip(*DEFAULT_LIMITS)
        if not any(ylim):
            _, ylim = zip(*DEFAULT_LIMITS)
        self.hardlimits = tuple(zip(xlim, ylim))
        self.softlimits = self.hardlimits

        events.subscribe(events.USER_ABORT, self.on_abort)

    def finalize_initialization(self):
        """Finalize device initialization."""
        self.status_thread = threading.Thread(
            target=self.poll_status, name="Linkam-status"
        )
        events.subscribe(events.COCKPIT_INIT_COMPLETE, self.status_thread.start)

    def poll_status(self):
        """Fetch the status from the remote and update the UI.

        Formerly, the remote periodically pushed status to a cockpit
        server. This caused frequent Pyro timeout errors when cockpit
        was busy doing other things.
        """
        last_temps = [None]
        last_time = 0

        while True:
            time.sleep(1)
            try:
                status = self._proxy.get_status()
            except Pyro4.errors.ConnectionClosedError:
                # Some dumb Pyro bug.
                continue

            if status.get("connected", False):
                self.status.update(status)
                self.send_position_updates()
                t_now = time.time()
                if t_now - last_time > LOGGING_PERIOD:
                    new_temps = [status.get(k) for k in self.logger.keys]
                    if not all(map(operator.eq, new_temps, last_temps)):
                        self.logger.log(new_temps)
                        last_temps = new_temps
                    last_time = t_now
            else:
                self.status["connected"] = False
            self.update_ui()

    def initialize(self):
        """Initialize the device."""
        super().initialize()
        self.get_position(should_use_cache=False)
        self.update_settings()

    def on_abort(self, *args):
        """Actions to do in the event of an abort."""
        pass

    def get_handlers(self):
        """Generate and return device handlers."""
        result = []
        # zip(*limits) transforms ((x0,y0),(x1,y1)) to ((x0,x1),(y0,y1))
        for axis, (min_pos, max_pos) in enumerate(zip(*self.softlimits)):
            result.append(
                cockpit.handlers.stage_positioner.PositionerHandler(
                    "%d linkam mover" % axis,
                    "%d stage motion" % axis,
                    False,
                    {
                        "move_absolute": self.move_absolute,
                        "move_relative": self.move_relative,
                        "get_position": self.get_position,
                    },
                    axis,
                    (min_pos, max_pos),  # hard limits
                    (min_pos, max_pos),  # soft limits
                )
            )
        return result

    def make_ui(self, parent):
        """Make cockpit user interface elements."""
        ## A list of value displays for temperatures.
        # Panel, sizer and a device label.
        self.panel = wx.Panel(parent, style=wx.BORDER_RAISED)
        self.panel.SetDoubleBuffered(True)
        panel = self.panel
        panel.Sizer = wx.BoxSizer(wx.HORIZONTAL)
        left_sizer = wx.BoxSizer(wx.VERTICAL)
        right_sizer = wx.BoxSizer(wx.VERTICAL)
        panel.Sizer.Add(left_sizer)
        panel.Sizer.Add(right_sizer)

        self.elements = {}
        light_button = wx.ToggleButton(panel, wx.ID_ANY, "light")
        light_button.Bind(
            wx.EVT_TOGGLEBUTTON,
            lambda evt: self._proxy.set_light(evt.EventObject.Value),
        )
        self.elements["light"] = light_button
        left_sizer.Add(light_button, flag=wx.EXPAND)
        condensor_button = wx.ToggleButton(panel, wx.ID_ANY, "condensor")
        condensor_button.Bind(
            wx.EVT_TOGGLEBUTTON,
            lambda evt: self._proxy.set_condensor(evt.EventObject.Value),
        )
        left_sizer.Add(condensor_button, flag=wx.EXPAND)
        ## Generate the value displays.
        for d in self._temperature_names:
            self.elements[d] = cockpit.gui.device.ValueDisplay(
                parent=panel,
                label=d,
                value=0.0,
                format_str="%.1f",
                unit_str="°C",
            )
            left_sizer.Add(self.elements[d])
        # Settings button
        adv_button = wx.Button(parent=self.panel, label="settings")
        adv_button.Bind(wx.EVT_LEFT_UP, self.show_settings)
        left_sizer.Add(adv_button, flag=wx.EXPAND)
        # Refill timers
        for r in self._refill_names:
            self.elements[r] = RefillTimerPanel(panel, wx.ID_ANY, label=r)
            right_sizer.Add(
                self.elements[r], flag=wx.LEFT | wx.RIGHT | wx.BOTTOM, border=4
            )
            if r == "sample":
                self.elements[r].set_refill_func(self._proxy.refill_chamber)
            elif r == "external":
                self.elements[r].set_refill_func(self._proxy.refill_dewar)
        panel.Fit()
        return panel

    def move_absolute(self, axis, pos):
        """Move a stage axis to new position, pos."""
        pos = int(pos)
        if axis == 0:
            new_pos = (pos, None)
        elif axis == 1:
            new_pos = (None, pos)
        else:
            # Arguments were wrong. Just return, since raising an
            # exception can kill the mosaic.
            return
        with self.xy_lock:
            # moveToXY(x, y), where None indicates no change.
            self._proxy.move_to(*new_pos)
        self.motion_targets[axis] = pos
        self.send_position_updates()

    def move_relative(self, axis, delta):
        """Move stage to a position relative to the current position."""
        if delta:
            cur_pos = self.position_cache[axis]
            self.move_absolute(axis, cur_pos + delta)

    @cockpit.util.threads.call_in_new_thread
    def send_position_updates(self):
        """Send XY stage positions until it stops moving."""
        if self.sending_position_updates is True:
            # Already sending updates.
            return
        self.sending_position_updates = True
        moving = True
        # Send positions at least once.
        while moving:
            # Need this thread to sleep to give UI a chance to update.
            # Sleep at start of loop to allow stage time to respond to
            # move request so remote.isMoving() returns True.
            time.sleep(0.1)
            # Update position cache before publishing STAGE_MOVER so
            # that the UI gets the new position when it queries it.
            self.get_position(should_use_cache=False)
            for axis in [0, 1]:
                events.publish(events.STAGE_MOVER, axis)
            moving = self._proxy.is_moving()

        for axis in (0, 1):
            events.publish(events.STAGE_STOPPED, "%d linkam mover" % axis)
            self.motion_targets = [None, None]
        self.sending_position_updates = False
        return

    def get_position(self, axis=None, should_use_cache=True):
        """Return the position of one or both axes.

        If axis is None, return positions of both axes.
        Query the hardware if should_use_cache is False.
        """
        if not should_use_cache:
            # Occasionally, at the start on an experiment, a
            # ConnectionClosedError is thrown here. I think this is something
            # to do with Pyro reusing connections and those connections getting
            # closed on the remote due to frequent traffic in the status thread.
            # Workaround: retry a few times in the event of a ConnectionClosedError.
            success = False
            fail_count = 0
            while not success:
                try:
                    position = self._proxy.get_position()
                    success = True
                except Pyro4.errors.ConnectionClosedError:
                    if fail_count < 5:
                        fail_count += 1
                    else:
                        raise
                except:
                    raise
            self.position_cache = (position["X"], position["Y"])
        if axis is None:
            return self.position_cache
        else:
            return self.position_cache[axis]

    def update_ui(self):
        """Update user interface elements."""
        status = self.status
        if not status.get("connected", False):
            self.panel.Disable()
            return
        self.panel.Enable()
        # Temperatures
        for t in self._temperature_names:
            self.elements[t].update(self.status.get("t_" + t))
        self.elements["light"].SetValue(status.get("light", False))
        # Refills
        lines = []
        now = datetime.datetime.now()

        for r in self._refill_names:
            refill = status["refills"].get(r, None)
            self.elements[r].do_update(refill)

    def make_initial_publications(self):
        """Send initial device publications."""
        self.send_position_updates()
