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

import wx

import cockpit.gui
import cockpit.util.threads
from cockpit import depot
from cockpit.handlers import device


class Filter:
    """An individual filter."""

    def __init__(self, position, *args):
        self.position = int(position)
        # args describes the filter.
        # The description can be one of
        #   label, value
        #   (label, value)
        #   label
        if isinstance(args[0], tuple):
            self.label = args[0][0]
            if len(args[0]) > 1:
                self.value = args[0][1]
            else:
                self.value = None
        else:
            self.label = args[0]
            if len(args) > 1:
                self.value = args[1]
            else:
                self.value = None

    def __repr__(self):
        if self.value:
            return "%d: %s, %s" % (self.position, self.label, self.value)
        else:
            return "%d: %s" % (self.position, self.label)


class FilterHandler(device.DeviceHandler):
    """A handler for emission and ND filter wheels."""

    def __init__(
        self,
        name,
        group_name,
        is_eligible_for_experiments,
        callbacks,
        cameras,
        lights,
    ):
        super().__init__(
            name,
            group_name,
            is_eligible_for_experiments,
            callbacks,
            depot.LIGHT_FILTER,
        )
        self.cameras = cameras or []
        self.lights = lights or []
        self.lastFilter = None

    @property
    def filters(self):
        return self.callbacks["getFilters"]()

    def on_save_settings(self):
        return self.callbacks["get_position"]()

    def on_load_settings(self, settings):
        filters = self.callbacks["make_selector"]()
        for f in filters:
            if f.position == settings:
                self.set_filter(f)

    def make_selector(self, parent):
        """UI functions"""
        self.ctrl = wx.Choice(parent)
        self.ctrl.Set(list(map(str, self.filters)))
        self.ctrl.Bind(
            wx.EVT_CHOICE,
            lambda evt: self.set_filter(self.filters[evt.Selection]),
        )
        self.add_watch("lastFilter", self.set_from_name)
        self.ctrl.Bind(wx.EVT_WINDOW_DESTROY, self.on_selector_destroy)
        self.ctrl.SetSelection(self.ctrl.FindString(str(self.lastFilter)))
        return self.ctrl

    def set_from_name(self, name):
        self.ctrl.SetSelection(self.ctrl.FindString(str(name)))

    def on_selector_destroy(self, event: wx.WindowDestroyEvent) -> None:
        self.remove_watch("lastFilter", self.set_from_name)
        event.Skip()

    def make_ui(self, parent):
        panel = wx.Panel(parent)
        panel.Sizer = wx.BoxSizer(wx.VERTICAL)
        panel.Sizer.Add(wx.StaticText(panel, label=self.name))
        panel.Sizer.Add(self.make_selector(panel), flag=wx.EXPAND)
        return panel

    def set_filter(self, filter):
        self.callbacks["setPosition"](
            filter.position, callback=self.update_after_move
        )

    def current_filter(self):
        position = self.callbacks["get_position"]()
        filters = self.callbacks["getFilters"]()
        for f in filters:
            if f.position == position:
                return f

    @cockpit.util.threads.call_in_main_thread
    def update_after_move(self, *args):
        # Accept *args so that can be called directly as a Pyro callback
        # or an event handler.
        self.lastFilter = self.current_filter()
        # Emission filters
        for camera in self.cameras:
            h = depot.get_handler(camera, depot.CAMERA)
            if h is not None:
                h.update_filter(self.lastFilter.label, self.lastFilter.value)
        # Excitation filters
        for h in self.lights:
            pass

    def finalize_initialization(self):
        self.update_after_move()
