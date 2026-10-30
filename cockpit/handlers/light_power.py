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

import logging

import cockpit.util.user_config
from cockpit import depot
from cockpit.handlers import device


_logger = logging.getLogger(__name__)


class LightPowerHandler(device.DeviceHandler):
    """Handler for light sources whose power can be controlled in software.

    Callbacks should fill in the following functions:
    - set_power(value): Set power level.
    - get_power(): Get current output power level.

    Args:
        current_power (float): Initial output power.
        is_enabled (bool): True iff the handler can be interacted with.
    """

    def __init__(
        self,
        name,
        group_name,
        callbacks,
        wavelength,
        current_power: float,
        is_enabled=True,
    ) -> None:
        # Validation:
        required = set(["get_power", "set_power"])
        missing = required.difference(callbacks)
        if missing:
            e = Exception(
                "%s %s missing callbacks: %s."
                % (self.__class__.__name__, name, " ".join(missing))
            )
            raise e

        super().__init__(name, group_name, False, callbacks, depot.LIGHT_POWER)
        self.wavelength = wavelength
        self.last_power = current_power
        self.power_set_point = None
        self.is_enabled = is_enabled

    def finalize_initialization(self):
        super().finalize_initialization()
        self._apply_user_config()

    def _apply_user_config(self):
        target_power = cockpit.util.user_config.get_value(
            self.name + "-light_power", default=0.01
        )
        try:
            self.set_power(target_power)
        except Exception as e:
            _logger.warning(
                "Failed to set prior power level %s for %s: %s",
                target_power,
                self.name,
                e,
            )

    def on_save_settings(self):
        return self.power_set_point

    def on_load_settings(self, settings):
        try:
            self.set_power(settings)
        except Exception as e:
            # Invalid power; just ignore it.
            print("Invalid power for %s: %s" % (self.name, settings))

    def set_enabled(self, is_enabled):
        """Toggle accessibility of the handler."""
        self.is_enabled = is_enabled

    def get_is_enabled(self):
        """Return True iff we're currently enabled (i.e. GUI is active)."""
        return self.is_enabled

    def get_power(self):
        """Fetch the current laser power."""
        return self.callbacks["get_power"]()

    def set_power(self, power):
        """Handle the user selecting a new power level."""
        if power < 0.0 or power > 1.0:
            raise RuntimeError(
                "Tried to set invalid power %f for light %s"
                % (power, self.name)
            )
        self.callbacks["set_power"](power)
        self.power_set_point = power
        cockpit.util.user_config.set_value(self.name + "-light_power", power)

    def get_wavelength(self):
        """Simple getter."""
        return self.wavelength

    def get_savefile_info(self):
        """Experiments should include the laser power."""
        return "%s: %.1f" % (self.name, self.last_power)
