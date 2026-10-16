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

import decimal

import cockpit.handlers.imager
import cockpit.interfaces.imager
import cockpit.util.colors
from cockpit import depot, events
from cockpit.handlers import device


## Available trigger modes for triggering the camera.
# Trigger at the end of an exposure; trigger before the exposure;
# trigger for the duration of the exposure.
TRIGGER_AFTER, TRIGGER_BEFORE, TRIGGER_DURATION, TRIGGER_SOFT = range(4)


class CameraHandler(device.DeviceHandler):
    r"""This handler is for cameras, of course.

    Cameras provide images to the microscope, and are assumed to be usable
    during experiments.

    Callbacks should fill in the following functions:
    - set_enabled(name, should_enable): Turn the camera "on" or "off".
    - get_image_size(name): Return a (width, height) tuple describing the size
      in pixels of the image the camera takes.
    - get_time_between_exposures(name, is_exact): Return the minimum time between
      exposures for this camera, in milliseconds. If is_exact is set, returns
      a decimal.Decimal instance.
    - set_exposure_time(name, time): Change the camera's exposure time to
      the specified value, in milliseconds.
    - get_exposure_time(name, is_exact): Returns the time in milliseconds that
      the camera is set to expose for when triggered. If is_exact is set,
      returns a decimal.Decimal instance.
    - get_shuttering_mode(name): Returns if the camera has a global or a rolling
      electronic shuttering mode.
    - set_roi(name, roi): Change the camera's region of interest to
      the specified region.
    - get_roi(name): Returns the current region of interest.
    - get_sensor_shape(name): Returns the camera's sensor shape.
    - prepare_for_experiment(name, experiment): Get the camera ready for an
      experiment.
    - Optional: get_min_exposure_time(name): returns the minimum exposure time
      the camera is capable of performing, in milliseconds. If not available,
      0ms is used.

    Args:
        exposure_mode (int): One of TRIGGER_AFTER, TRIGGER_BEFORE,
            TRIGGER_DURATION. The first two are for external-trigger cameras,
            which may be frame-transfer (trigger at end of exposure, and expose
            continuously) or not (trigger at beginning of exposure and expose
            for a pre-configured duration). The last two are for
            external-exposure cameras, which expose for as long as you tell
            them to, based on the TTL line.
        min_exposure_time (float): Minimum exposure duration, in milliseconds.
            Typically only applicable if doExperimentsExposeContinuously is
            True.
    """

    ## Shortcuts to decorators defined in parent class.
    reset_cache = device.DeviceHandler.reset_cache
    cached = device.DeviceHandler.cached

    def __init__(
        self,
        name,
        group_name,
        callbacks,
        exposure_mode,
        trig_handler=None,
        trig_line=None,
    ):
        # Note we assume that cameras are eligible for experiments.
        super().__init__(name, group_name, True, callbacks, depot.CAMERA)
        ## True if the camera is currently receiving images.
        self.is_enabled = False
        self._exposure_mode = exposure_mode
        self.wavelength = None
        self.dye = None
        # Set up trigger handling.
        if trig_handler and trig_line:
            h = trig_handler.registerDigital(self, trig_line)
            self.triggerNow = h.triggerNow
        else:
            softTrigger = self.callbacks.get("softTrigger", None)
            self.triggerNow = lambda: softTrigger
            if softTrigger:
                depot.addHandler(
                    cockpit.handlers.imager.ImagerHandler(
                        "%s imager" % name,
                        "imager",
                        {"takeImage": softTrigger},
                    )
                )

    def on_save_settings(self):
        return self.get_is_enabled()

    def on_load_settings(self, settings):
        # Only change state if we need to as this is slow.
        if self.get_is_enabled() != settings:
            self.toggle_state()

    @property
    def color(self):
        if self.wavelength is not None:
            return cockpit.util.colors.wavelength_to_color(self.wavelength, 0.8)
        else:
            return (127,) * 3

    @property
    def descriptive_name(self):
        if self.dye is not None:
            return "%s (%s)" % (self.name, self.dye)
        else:
            return self.name

    @property
    def exposure_mode(self):
        return self._exposure_mode

    @exposure_mode.setter
    def exposure_mode(self, trigger_type):
        """Set exposure mode."""
        self._exposure_mode = trigger_type

    def update_filter(self, dye, wavelength=None):
        """Update the filter for this camera."""
        self.dye = dye
        self.wavelength = wavelength
        events.publish(events.FILTER_CHANGE)

    @cockpit.interfaces.imager.pause_video
    @reset_cache
    def set_enabled(self, should_enable=True):
        """Invoke our callback, and let everyone know that a new camera is
        online."""
        try:
            self.is_enabled = self.callbacks["set_enabled"](
                self.name, should_enable
            )
        except:
            self.is_enabled = False
            raise
        if self.is_enabled != should_enable:
            raise Exception("Problem enabling device with handler %s" % self)
        # Subscribe / unsubscribe to the prepare-for-experiment event.
        func = [events.unsubscribe, events.subscribe][should_enable]
        func(events.PREPARE_FOR_EXPERIMENT, self.prepare_for_experiment)
        events.publish(events.CAMERA_ENABLE, self, self.is_enabled)

    def get_is_enabled(self):
        """Return self.is_enabled."""
        return self.is_enabled

    def get_image_size(self):
        """Return the size, in pixels, of images we generated."""
        return self.callbacks["get_image_size"](self.name)

    @cached
    def get_time_between_exposures(self, is_exact=False):
        """Return the amount of time, in milliseconds, that must pass after
        ending one exposure before another can be started.

        If is_exact is specified, then we return a decimal.Decimal value instead
        of a raw floating point value.
        """
        return self.callbacks["get_time_between_exposures"](self.name, is_exact)

    @cached
    def get_min_exposure_time(self, is_exact=False):
        """Return the minimum allowed exposure time, in milliseconds."""
        val = 0
        if "get_min_exposure_time" in self.callbacks:
            val = self.callbacks["get_min_exposure_time"](self.name)
        if is_exact:
            return decimal.Decimal(val)
        return val

    @reset_cache
    def set_exposure_time(self, time):
        """Set a new exposure time, in milliseconds."""
        return self.callbacks["set_exposure_time"](self.name, time)

    @cached
    def get_exposure_time(self, is_exact=False):
        """Return the camera's currently-set exposure time, in milliseconds.

        If is_exact is specified, then we return a decimal.Decimal value instead
        of a raw floating point value.
        """
        return self.callbacks["get_exposure_time"](self.name, is_exact)

    @cached
    def get_shuttering_mode(self):
        return self.callbacks["get_shuttering_mode"](self.name)

    @reset_cache
    def set_roi(self, roi):
        """Do any necessary preparation for the camera to participate in an."""
        return self.callbacks["set_roi"](self.name, roi)

    @cached
    def get_roi(self):
        return self.callbacks["get_roi"](self.name)

    @cached
    def get_sensor_shape(self):
        return self.callbacks["get_sensor_shape"](self.name)

    @reset_cache
    def prepare_for_experiment(self, experiment):
        """Do any necessary preparation for the camera to participate in an
        experiment."""
        return self.callbacks["prepare_for_experiment"](self.name, experiment)

    def get_exposure_mode(self):
        """Simple getter."""
        return self.exposure_mode
