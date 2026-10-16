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

from decimal import Decimal

from cockpit import depot
from cockpit.devices.device import Device
from cockpit.handlers import stage_positioner
from cockpit.interfaces.stage_mover import AXIS_MAP


class SimplePiezo(Device):
    """A simple piezo axis.

    Sample config entry:

    .. code:: ini

        [zPiezo]                 # name
        type: cockpit.devices.stage.SimplePiezo        # this class
        analogSource: asource    # an analogue source device name
        analogLine: 0            # the line on the analogue source
        offset: 0                # analogue units offset in experiment units, e.g. um
        gain: 262.144            # unit conversion gain, e.g. ADU per um
        min: 0                   # min axis range in experimental units
        range: 250               # axis range in experimental units

    """

    _config_types = {
        # Min, max and range are ints to prevent crashes where ints are expected
        # in UI code. We should fix this to be able to use floats.
        "range": int,
        "min": int,
        "max": int,
        "stepmin": int,
        "offset": float,
        "gain": float,
    }

    def __init__(self, name, config):
        super(SimplePiezo, self).__init__(name, config)

    def get_handlers(self):
        asource = self.config.get("analogsource", None)
        aline = self.config.get("analogline", None)
        a_handler = depot.getHandler(asource, depot.EXECUTOR)
        if a_handler is None:
            raise Exception("No control source.")
        axis = AXIS_MAP[self.config.get("axis", 2)]
        offset = self.config.get("offset", 0)
        gain = self.config.get("gain", 1)
        pos_min = self.config.get("min", None)
        pos_max = self.config.get("max", None)
        pos_range = self.config.get("range", None)
        have_min, have_max, have_range = [
            v is not None for v in [pos_min, pos_max, pos_range]
        ]
        if have_min and have_max:
            pass
        elif (have_min, have_max, have_range) == (True, False, True):
            pos_max = pos_min + pos_range
        elif (have_min, have_max, have_range) == (False, True, True):
            pos_min = pos_max - pos_range
        elif (have_min, have_max, have_range) == (False, False, True):
            # Assume range starts from zero.
            pos_min = 0
            pos_max = pos_range
        else:
            raise Exception(
                "No min, max or range specified for stage %s." % self.name
            )

        result = []
        # Create handler without movement callbacks.
        handler = stage_positioner.PositionerHandler(
            "%d %s" % (axis, self.name),
            "%d stage motion" % axis,
            True,
            {
                "getMovementTime": lambda x, start, delta: (
                    Decimal(0.05),
                    Decimal(0.05),
                )
            },
            axis,
            (pos_min, pos_max),
            (pos_min, pos_max),
        )

        # Connect handler to analogue source to populate movement callbacks.
        handler.connectToAnalogSource(a_handler, aline, offset, gain)

        result.append(handler)
        return result
