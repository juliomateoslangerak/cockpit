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

import re

from cockpit import depot, events
from cockpit.devices import device
from cockpit.handlers.light_source import LightHandler


class ShutterDevice(device.Device):
    """Base shutter device with dummy methods for testing."""

    def __init__(self, name, config={}):
        super().__init__(name, config)
        lights = config.get("lights", None)
        if lights:
            self.lights = re.split(r"[,;: ]\s*", lights)
        else:
            self.lights = None

    def finalize_initialization(self):
        """Replace lights strings with their handlers."""
        handlers = []
        for name in self.lights:
            h = depot.get_handler(name, depot.LIGHT_TOGGLE)
            if h:
                handlers.append(h)
        self.lights = handlers
        # Register this shutter with the LightHandler class.
        LightHandler.add_shutter(self, self.lights)

    def initialize(self):
        pass

    def perform_subscriptions(self):
        events.subscribe(
            events.PREPARE_FOR_EXPERIMENT, self.on_prepare_for_experiment
        )
        events.subscribe(
            events.EXPERIMENT_COMPLETE, self.on_cleanup_after_experiment
        )
        events.subscribe(events.LIGHT_SOURCE_ENABLE, self.on_light_source_enable)

    def enable_trigger(self, enable=True):
        if enable:
            print("Shutter %s enabled." % self.name)
        else:
            print("Shutter %s disabled." % self.name)

    def on_light_source_enable(self, handler, enable):
        if enable and handler in self.lights:
            # One of our lights has been enabled.  Make sure that we respond to triggers.
            self.enable_trigger()
        elif not enable and not any([l.get_is_enabled() for l in self.lights]):
            # All of our lights are disabled.
            self.enable_trigger(False)

    def set_exposure_time(self, t):
        print("Shutter %s exposure time set to %s." % (self.name, t))

    def on_prepare_for_experiment(self, experiment):
        self.enable_trigger()

    def on_cleanup_after_experiment(self):
        pass
