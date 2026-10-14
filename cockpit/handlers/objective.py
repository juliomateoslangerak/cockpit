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

import typing

import cockpit.depot
import cockpit.handlers.device


class ObjectiveHandler(cockpit.handlers.device.DeviceHandler):
    """Handler for a single objective.

    Args:
        name:
        group_name:
        pixel_size: how many microns wide a pixel using that objective
            appears to be.
        transform:
        offset:
        colour:
        lens_id:
    """

    def __init__(
        self,
        name: str,
        group_name: str,
        pixel_size: float,
        transform: typing.Tuple[int, int, int],
        offset: typing.Tuple[int, int, int],
        colour: typing.Tuple[float, float, float],
        lens_id: int,
    ) -> None:
        super().__init__(
            name,
            group_name,
            is_eligible_for_experiments=False,
            callbacks={},
            device_type=cockpit.depot.OBJECTIVE,
        )
        self.pixel_size = pixel_size
        self.transform = transform
        self.offset = offset
        self.colour = colour
        self.lens_id = lens_id
