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

import time

from cockpit import depot, events
from cockpit.handlers import device


class PositionerHandler(device.DeviceHandler):
    """This handler is for stage positioner devices.

    Callbacks should fill in the following functions:
    - move_absolute(axis, position): Move the axis to the given position,
      in microns.
    - move_relative(axis, delta): Move the axis by the specified delta,
      in microns.
    - get_position(axis): Get the position for the specified axis.
    Additionally, if the device is to be used in experiments, it must have:
    - get_movement_time(axis, start, end): Get the amount of time it takes to
      move from start to end and then stabilize.

    Args:
        axis (int): A numerical indicator of the axis (0 = X, 1 = Y,
            2 = Z).
        hard_limits (tuple): A (minPosition, maxPosition) tuple indicating
            the device's hard motion limits.
        soft_limits (tuple): Default soft motion limits for the device.
            Defaults to the hard limits.
    """

    ## Shortcuts to decorators defined in parent class.
    reset_cache = device.DeviceHandler.reset_cache
    cached = device.DeviceHandler.cached

    def __init__(
        self,
        name,
        group_name,
        is_eligible_for_experiments,
        callbacks,
        axis,
        hard_limits,
        soft_limits=None,
    ):
        super().__init__(
            name,
            group_name,
            is_eligible_for_experiments,
            callbacks,
            depot.STAGE_POSITIONER,
        )
        self.axis = axis
        self.hard_limits = hard_limits
        if soft_limits is None:
            soft_limits = hard_limits
        elif min(soft_limits) < min(hard_limits) or max(soft_limits) > max(
            hard_limits
        ):
            raise RuntimeError(
                "Soft limits {} for the PositionerHandler were outside the \
                               Range of hard limits {}".format(
                    soft_limits, hard_limits
                )
            )

        # Cast to a list since we may need to modify these later.
        self.soft_limits = list(soft_limits)

    def move_absolute(self, pos):
        """Handle being told to move to a specific position."""
        if self.soft_limits[0] <= pos <= self.soft_limits[1]:
            self.callbacks["move_absolute"](self.axis, pos)
        else:
            raise RuntimeError(
                "Tried to move %s " % (self.name)
                + "outside soft motion limits (target %.2f, limits [%.2f, %.2f])"
                % (pos, self.soft_limits[0], self.soft_limits[1])
            )

    def move_relative(self, delta):
        """Handle being told to move by a specific delta."""
        target = self.callbacks["get_position"](self.axis) + delta
        if self.soft_limits[0] <= target <= self.soft_limits[1]:
            self.callbacks["move_relative"](self.axis, delta)
        else:
            raise RuntimeError(
                "Tried to move %s " % (self.name)
                + "outside soft motion limits (target %.2f, limits [%.2f, %.2f])"
                % (target, self.soft_limits[0], self.soft_limits[1])
            )

    def get_position(self):
        """Retrieve the current position."""
        return self.callbacks["get_position"](self.axis)

    def get_hard_limits(self):
        """Simple getter."""
        return self.hard_limits

    def get_soft_limits(self):
        """Simple getter."""
        return self.soft_limits

    def set_soft_limit(self, value, isMax):
        """Set a soft limit, either min or max."""
        if isMax and value > self.hard_limits[1]:
            raise RuntimeError(
                "Attempted to set soft motion limit of %s, exceeding our hard motion limit of %s"
                % (value, self.hard_limits[1])
            )
        elif not isMax and value < self.hard_limits[0]:
            raise RuntimeError(
                "Attempted to set soft motion limit of %s, lower than our hard motion limit of %s"
                % (value, self.hard_limits[0])
            )
        self.soft_limits[int(isMax)] = value

    def get_movement_time(self, start, end):
        """Return the amount of time it'd take us to move the specified
        distance, and the amount of time needed to stabilize after reaching
        that point.

        Only called if this device is experiment-eligible.
        """
        if self.is_eligible_for_experiments:
            # if (start < self.soft_limits[0] or start > self.soft_limits[1] or
            #         end < self.soft_limits[0] or end > self.soft_limits[1]):
            #     raise RuntimeError("Experiment tries to move [%s] from %.2f to %.2f, outside motion limits (%.2f, %.2f)" % (self.name, start, end, self.soft_limits[0], self.soft_limits[1]))
            # return self.callbacks['get_movement_time'](self.axis, start, end)
            return self.get_delta_movement_time(end - start)
        raise RuntimeError(
            "Called get_movement_time on non-experiment-eligible positioner [%s]"
            % self.name
        )

    @cached
    def get_delta_movement_time(self, delta):
        return self.callbacks["get_movement_time"](self.axis, 0.0, delta)

    def connect_to_analog_source(self, source, line, offset, gain):
        """Register this handler with an analogue source."""
        h = source.register_analog(self, line, offset, gain)

        def wrap_move_func(f):
            """Movements are handled by the analogue handler, but need to wrap
            to publish position update and stage stop events."""

            def call(x, arg):
                f(arg)
                time.sleep(sum(map(float, self.get_movement_time(0, arg))))
                events.publish(events.STAGE_MOVER, x)
                events.publish(events.STAGE_STOPPED, self.name)

            return call

        self.callbacks["move_absolute"] = wrap_move_func(h.move_absolute)
        self.callbacks["move_relative"] = wrap_move_func(h.move_relative)

        # Sensorless devices will infer position from analogue output.
        # Those with sensors should have already specified this callback.
        if self.callbacks.get("get_position") is None:
            self.callbacks["get_position"] = lambda x: h.get_position()
