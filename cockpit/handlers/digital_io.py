#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2021 University of Oxford
## Copyright (C) 2023 Ian Dobbie ian.dobbie@jhu.edu
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

import wx

import cockpit.gui
import cockpit.gui.device
import cockpit.util.threads
from cockpit import depot
from cockpit.handlers import device


class DigitalIOHandler(device.DeviceHandler):
    """A handler for Digital IO devices."""

    def __init__(
        self, name, group_name, is_eligible_for_experiments, callbacks
    ):
        super().__init__(
            name, group_name, is_eligible_for_experiments, callbacks, depot.DIO
        )
        self.is_enabled = False

    @property
    def paths(self):
        return self.callbacks["get_paths"]()

    def on_save_settings(self):
        paths = {}
        for key in self.path_name_to_button.keys():
            paths[key] = int(self.path_name_to_button[key].GetValue())
        outputs = list(map(int, self.callbacks["get_outputs"]()))
        io_state = [
            self.callbacks["get_io_state"](i) for i in range(len(outputs))
        ]
        return [paths, outputs, io_state]

    def on_load_settings(self, settings):
        paths, outputs, io_state = settings
        for key in paths.keys():
            self.path_name_to_button[key].SetValue(paths[key])
        for i, state in enumerate(io_state):
            # probably safer not to change IO state here
            # self.callbacks['set_io_state'](i,state)
            if state:
                self.callbacks["write_line"](i, outputs[i])

    def set_enabled(self, should_enable=True):
        try:
            self.is_enabled = self.callbacks["enable"](should_enable)
        except:
            self.is_enabled = False
            raise
        if self.is_enabled != should_enable:
            raise Exception("Problem enabling device with handler %s" % self)

    def get_is_enabled(self):
        """Return self.is_enabled."""
        return self.is_enabled

    def make_ui(self, parent):
        """UI functions"""
        self.panel = wx.Panel(parent)
        sizer = wx.BoxSizer(wx.VERTICAL)
        enable_button = cockpit.gui.device.EnableButton(
            self.panel, device_handler=self
        )
        sizer.Add(enable_button, 1, wx.EXPAND)
        self.path_name_to_button = {}
        for key in self.paths.keys():
            button = wx.ToggleButton(self.panel, wx.ID_ANY)
            button.SetLabel(key)
            button.Bind(
                wx.EVT_TOGGLEBUTTON, lambda evt, b=button: self.toggle_paths(b)
            )
            sizer.Add(button, 1, wx.EXPAND)
            self.path_name_to_button[key] = button
        self.panel.SetSizerAndFit(sizer)
        return self.panel

    def toggle_paths(self, button):
        path = button.Label
        if button.GetValue():
            # button is active so set the relevant DIO lines
            # take settings for this path
            settings = self.paths[path]
            # loop through DIO settings.
            for object in settings[0].keys():
                labels = self.callbacks["get_labels"]()
                line = labels.index(object)
                # loop through settings and set each named object to that state.
                self.callbacks["write_line"](line, settings[0][object])
            #                print(path,self.callbacks['get_outputs']())
            # Need some way to define exclusive and non-exclusive paths
            # assume they are exclusive for now.
            other_buttons = settings[1]
            for key in other_buttons.keys():
                self.path_name_to_button[key].SetValue(other_buttons[key])

    @cockpit.util.threads.call_in_main_thread
    def update_after_change(self, *args):
        #        # Accept *args so that can be called directly as a Pyro callback
        #        # or an event handler.
        pass


#        # need to update display if active.
# self.


def finalize_initialization(self):
    self.update_after_change()
