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

from cockpit import depot
from cockpit.handlers import device


class GenericPositionerHandler(device.DeviceHandler):
    r"""This handler is for generic positioning devices.

    It moves along a single axis, and is not used for stage/sample
    positioning. Use the StagePositionerHandler for positioners that move
    the sample around.

    Callbacks should fill in the following functions:
    - moveAbsolute(name, position): Move to the given position, in whatever
      units are appropriate.
    - moveRelative(name, delta): Move by the specified delta, again in
      whatever units are appropriate.
    - getPosition(name): Get the current position.
    Additionally, if the device is eligible for experiments, it needs to
    have these functions:
    - getMovementTime(name, start, stop): return the movement time and
      stabilization time needed to go from <start> to <stop>.

    \todo Add motion limits.
    """

    ## Shortcuts to decorators defined in parent class.
    reset_cache = device.DeviceHandler.reset_cache
    cached = device.DeviceHandler.cached

    def __init__(self, name, groupName, isEligibleForExperiments, callbacks):
        super().__init__(
            name,
            groupName,
            isEligibleForExperiments,
            callbacks,
            depot.GENERIC_POSITIONER,
        )

    def moveAbsolute(self, pos):
        """Handle being told to move to a specific position."""
        self.callbacks["moveAbsolute"](pos)

    def moveRelative(self, delta):
        """Handle being told to move by a specific delta."""
        self.callbacks["moveRelative"](delta)

    def getPosition(self):
        """Retrieve the current position."""
        return self.callbacks["getPosition"]()

    def getMovementTime(self, start, stop):
        """Get the movement and stabilization time needed to perform the
        specified motion, in milliseconds."""
        # return self.callbacks['getMovementTime'](self.name, start, stop)
        return [
            decimal.Decimal(t)
            for t in self.callbacks["getMovementTime"](start, stop)
        ]

    @cached
    def getDeltaMovementTime(self, delta):
        return [
            decimal.Decimal(t)
            for t in self.callbacks["getMovementTime"](0.0, delta)
        ]
