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

import time
import typing

import cockpit.depot
import cockpit.devices.device
from cockpit.handlers.light_source import LightHandler


class SimpleLight(cockpit.devices.device.Device):
    """A simple light device.

    This class adds support for simple devices that may support
    external triggers and have no software interface to the hardware,
    ad therefore no power level control.  Sample config entry:

    .. code:: ini

        [led source]
        type: cockpit.devices.light.SimpleLight
        triggerSource: NAME_OF_EXECUTOR_DEVICE
        triggerLine: 1

    """

    def get_handlers(self):
        self.handlers = []
        trigsource = self.config.get("triggersource", None)
        trigline = self.config.get("triggerline", None)
        if trigsource:
            trighandler = cockpit.depot.get_handler(
                trigsource, cockpit.depot.EXECUTOR
            )
        else:
            trighandler = None
        self._exposure_time = 100
        self.handlers.append(
            LightHandler(
                self.name,
                self.name + " light source",
                {
                    "set_enabled": lambda name, on: time.sleep(0.5),
                    "set_exposure_time": lambda name, value: setattr(
                        self, "_exposure_time", value
                    ),
                    "get_exposure_time": lambda name: self._exposure_time,
                },
                self.config.get("wavelength", None),
                100,
                trighandler,
                trigline,
            )
        )

        return self.handlers


class AmbientLight(cockpit.devices.device.Device):
    """Ambient light source.

    Because exposure time is a property of the light source used to
    acquire an image and not of the camera, ``AmbientLight`` is a
    light source that enables specifying exposure times for images
    with no active illumination.

    """

    def __init__(self, name: str, config: typing.Mapping[str, str]) -> None:
        super().__init__(name, config)
        self._state = {"time": 100.0}
        callbacks = {
            "get_exposure_time": self._get_exposure_time,
            "set_exposure_time": self._set_exposure_time,
            "set_enabled": self._set_enabled,
        }  # type: typing.Dict[str, typing.Callable]
        self._handlers = [
            LightHandler(
                "Ambient",
                "ambient",
                callbacks,
                wavelength=0,
                exposure_time=100.0,
            )
        ]

    def get_handlers(self) -> typing.List[LightHandler]:
        return self._handlers

    def _get_exposure_time(self, name: str) -> float:
        return self._state["time"]

    def _set_exposure_time(self, name: str, value: float) -> None:
        self._state["time"] = value

    def _set_enabled(self, name: str, state: bool) -> None:
        # The ambient light source is always on, so we do nothing.  It
        # seems like there's no callback on the LightHandler to
        # actually check if the disabling/enabling worked (not that it
        # matters in this case).
        return
