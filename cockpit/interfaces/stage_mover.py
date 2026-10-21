#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2021 Nicholas Hall
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

import math
import operator
import threading
import typing

import numpy
import wx

import cockpit.util.threads
from cockpit import depot, events
from cockpit.util import user_config


AxisLimits = typing.Tuple[float, float]
StageLimits = typing.Tuple[AxisLimits, AxisLimits, AxisLimits]


## Stage movement threshold (previously a hard-coded value).
# There can be problems if this doesn't match the corresponding threshold
# used by the stage device code.
# TODO:  This should be defined in only one place, either here,
# in the stage code, or in a config file.
STAGE_MIN_MOVEMENT = 0.3
# Map possible axis identifiers to canonical integer identifiers.
AXIS_MAP = {}
AXIS_MAP.update({k: 0 for k in "0xX"})
AXIS_MAP.update({k: 1 for k in "1yY"})
AXIS_MAP.update({k: 2 for k in "2zZ"})
AXIS_MAP.update({k: k for k in [0, 1, 2]})

## This module handles general stage motion: "go to this position", "move by
# this delta", "remember this position", "go to this remembered position",
# etc. The cockpit deals with this module instead of speaking direction to
# stage device handlers.
# Several of the functions in this module accept an "axis" parameter. The
# mapping is 0: X; 1: Y; 2: Z.

## Auto-incrementing unique value to mark each Site instance.
unique_site_index = 0


class Site:
    """A Site is a simple container class that represents a saved location on
    the stage."""

    def __init__(self, position, group=None, color=(0, 255, 0), size=25):
        """Set up a saved location on the stage.

        Args:
            position (numpy.ndarray): 3D location (Numpy vector) of the Site.
            group (str): String describing the group the Site belongs to.
            color (tuple): RGB tuple used to color the Site in the UI.
            size (float): Size (in arbitrary units) to draw the site.
        """
        self.position = position
        self.group = group
        self.color = color
        self.size = size
        global unique_site_index
        unique_site_index += 1
        ## Unique ID for the Site.
        self.unique_id = unique_site_index

    def serialize(self):
        """Serialize the site -- convert it to a string we can use to
        reconstruct it later."""
        result = ""
        result += ",".join(map(str, self.position))
        result += ",%s," % self.group
        result += ",".join(map(str, self.color))
        result += ",%.2f,%d" % (self.size, self.unique_id)
        return result


def deserialize_site(line):
    """Generate a new Site from a text string generated from serialize(),
    above."""
    x, y, z, group, r, g, b, size, id = line.split(",")
    x, y, z, r, g, b, size = map(float, (x, y, z, r, g, b, size))
    id = int(id)
    if group == "None":
        group = None
        # Otherwise let group remain as a string.
    result = Site(numpy.array((x, y, z)), group, (r, g, b), size)
    result.unique_id = id
    return result


def _sensible_step_size(
    step_size: float,
    bases: typing.Sequence[int],
    cmp: typing.Callable[[float, float], bool],
) -> float:
    power_of_ten = 10 ** math.floor(math.log10(step_size))
    for base in bases:
        threshold = base * power_of_ten
        if cmp(step_size, threshold):
            return threshold
    else:
        raise RuntimeError(
            "failed to estimate best step size after trying"
            " all bases in %s" % bases
        )


def sensible_next_step_size(step_size: float) -> float:
    return _sensible_step_size(step_size, [2, 5, 10], operator.lt)


def sensible_previous_step_size(step_size: float) -> float:
    return _sensible_step_size(step_size, [5, 2, 1, 0.5], operator.gt)


class StageMover:
    """This class provides an interface between the rest of the UI and the
    Devices that handle moving the stage."""

    def __init__(self):
        """Maps axis to the handlers for that axis, sorted by their range of motion."""
        self.axis_to_handlers = depot.get_sorted_stage_movers()
        if set(self.axis_to_handlers.keys()) != {0, 1, 2}:
            raise ValueError("stage mover requires 3 axis: X, Y, and Z")

        # FIXME: we should have sensible defaults (see issue #638).
        self._saved_top = user_config.get_value("savedTop", default=3010.0)
        self._saved_bottom = user_config.get_value("savedBottom", default=3000.0)

        ## XXX: We have a single index for all axis, even though each
        ## axis may have a different number of stages.  While we don't
        ## refactor this assumption, we just make copies of the movers
        ## with the most precise movement (issues #413 and #415)
        self.n_stages = max([len(s) for s in self.axis_to_handlers.values()])
        for axis, stages in self.axis_to_handlers.items():
            stages.extend([stages[-1]] * (self.n_stages - len(stages)))

        ## Indicates which stage handler is currently under control.
        self.cur_handler_index = 0
        ## Maps Site unique IDs to Site instances.
        self.id_to_site = {}

        # Compute the hard motion limits for each axis as the
        # summation of all limits for handlers on that axis.
        hard_limits = [None] * 3
        for axis in range(3):
            lower = 0.0
            upper = 0.0
            # We need set() to avoid duplicated handlers, and we might
            # have duplicated handlers because of the hack to meet
            # cockpit requirements that all axis have the same number
            # of handlers (see comments on issue #413).
            for handler in set(self.axis_to_handlers[axis]):
                handler_limits = handler.get_hard_limits()
                lower += handler_limits[0]
                upper += handler_limits[1]
            hard_limits[axis] = (lower, upper)
        # Use a tuple to prevent changes to it, and assemble it like
        # this to enable static code analysis.
        self._hard_limits = (hard_limits[0], hard_limits[1], hard_limits[2])

        # Compute the initial step sizes.  We have different step
        # sizes for each handler index which maybe doesn't make sense
        # anymore but comes from the time when it were the handlers
        # themselves that kept track of step size.
        self._step_sizes = (
            []
        )  # type: typing.List[typing.Tuple[float, float, float]]
        for stage_index in range(self.n_stages):
            default_step_sizes = []
            for axis in (0, 1, 2):
                limits = self.axis_to_handlers[axis][stage_index].get_hard_limits()
                step_size = sensible_previous_step_size(
                    (limits[1] - limits[0]) / 100.0
                )
                default_step_sizes.append(step_size)
            # Default is 1/100 of the axis length but in rectangular
            # stages that will lead to xy with different step sizes so
            # use the min of the two.
            min_xy = min(default_step_sizes[0], default_step_sizes[1])
            self._step_sizes.append((min_xy, min_xy, default_step_sizes[2]))

        ## Maps handler names to events indicating if those handlers
        # have stopped moving.
        self.name_to_stopped_event = {}
        events.subscribe(events.STAGE_MOVER, self.on_motion)
        events.subscribe(events.STAGE_STOPPED, self.on_stop)

    @property
    def saved_top(self) -> float:
        return self._saved_top

    @saved_top.setter
    def saved_top(self, pos: float) -> None:
        user_config.set_value("savedTop", pos)
        self._saved_top = pos
        events.publish(events.STAGE_TOP_BOTTOM)

    @property
    def saved_bottom(self) -> float:
        return self._saved_bottom

    @saved_bottom.setter
    def saved_bottom(self, pos: float) -> None:
        user_config.set_value("savedBottom", pos)
        self._saved_bottom = pos
        events.publish(events.STAGE_TOP_BOTTOM)

    def get_step_sizes(self) -> typing.Tuple[float, float, float]:
        """Return a (dX, dY, dZ) tuple of the current step sizes."""
        return self._step_sizes[self.cur_handler_index]

    def step(self, direction: typing.Tuple[int, int, int]) -> None:
        """Move one step with the current active handler in the specified
        direction(s).

        Args:
            direction: A tuple/list of length equal to the number of
                axes of motion, where each element is the number of
                steps (positive or negative) to take along that axis.
        """
        if len(direction) != 3:
            raise ValueError("direction must be a 3 element list")
        for axis, sign in enumerate(direction):
            if sign != 0:
                step_size = self._step_sizes[self.cur_handler_index][axis]
                handler = self.axis_to_handlers[axis][self.cur_handler_index]
                handler.moveRelative(step_size * sign)

    def set_step_size(self, axis: int, step_size: float) -> None:
        if step_size <= 0.0:
            raise ValueError("step size must be a positive number")
        elif axis not in [0, 1, 2]:
            raise ValueError("axis must be in [0, 1, 2]")
        step_sizes = list(self.get_step_sizes())
        step_sizes[axis] = step_size
        self._step_sizes[self.cur_handler_index] = tuple(step_sizes)
        events.publish(events.STAGE_STEP_SIZE, axis, step_size)

    def change_step_size(self, direction: int) -> None:
        if direction == +1:
            guess_new = sensible_next_step_size
        elif direction == -1:
            guess_new = sensible_previous_step_size
        else:
            raise ValueError(
                "direction must be -1 (decrease) or +1 (increase)"
            )
        old_step_sizes = self.get_step_sizes()
        new_step_sizes = tuple([guess_new(x) for x in old_step_sizes])
        self._step_sizes[self.cur_handler_index] = new_step_sizes

        for axis, step_size in enumerate(self.get_step_sizes()):
            events.publish(events.STAGE_STEP_SIZE, axis, step_size)

    def on_motion(self, axis):
        """Handle one of our devices moving.

        We just republish an abstracted stage position for that axis.
        """
        events.publish(events.STAGE_POSITION, axis, get_position_for_axis(axis))

    def on_stop(self, name):
        """Handle one of our devices stopping motion; this unblocks _go_to_axes
        if it is waiting."""
        if name in self.name_to_stopped_event:
            self.name_to_stopped_event[name].set()

    def _go_to_axes(self, position, should_block=False):
        r"""Internal function to go to the specified location (specified as a
        list of (axis, position) tuples).

        Wait for the axes to stop moving, if should_block is true.
        \todo Assumes that the target position is within the range of motion of the current handler.
        """
        waiters = []
        for axis, target in position:
            # Get the offset for the movers that aren't being adjusted.
            offset = 0
            for handler in self.axis_to_handlers[axis]:
                if handler != self.axis_to_handlers[axis][self.cur_handler_index]:
                    offset += handler.getPosition()

            handler = self.axis_to_handlers[axis][self.cur_handler_index]
            # Check if we need to bother moving.
            if (
                abs(handler.getPosition() - (target - offset))
                > STAGE_MIN_MOVEMENT
            ):
                event = threading.Event()
                waiters.append(event)
                self.name_to_stopped_event[handler.name] = event
                handler.move_absolute(target - offset)
        if should_block:
            for event in waiters:
                try:
                    event.wait(30)
                except Exception as e:
                    print("Failed waiting for stage to stop after 30s")


## Global singleton.
mover = None


def initialize():
    """Create the StageMover."""
    global mover
    mover = StageMover()


def make_initial_publications():
    """Publicize any information that various widgets care about."""
    # for axis in range(3):
    for axis in mover.axis_to_handlers.keys():
        events.publish(events.STAGE_POSITION, axis, get_position_for_axis(axis))
        limits = get_soft_limits_for_axis(axis)
        for is_max in [0, 1]:
            events.publish(
                events.SOFT_SAFETY_LIMIT, axis, limits[is_max], bool(is_max)
            )


## Various module-global functions for interacting with the objects in the
# Mover.


def step(direction):
    mover.step(direction)


def change_mover():
    """Change to the next handler."""
    old_index = mover.cur_handler_index
    new_index = (mover.cur_handler_index + 1) % mover.n_stages
    if new_index != old_index:
        old_step_sizes = mover.get_step_sizes()
        mover.cur_handler_index = new_index
        events.publish(events.STAGE_STEP_INDEX, mover.cur_handler_index)
        for axis, new_step_size in enumerate(mover.get_step_sizes()):
            if old_step_sizes[axis] != new_step_size:
                events.publish(events.STAGE_STEP_SIZE, axis, new_step_size)


def change_step_size(direction):
    """Change the step size for the current handlers."""
    mover.change_step_size(direction)


def recenter_fine_motion():
    """Recenter the fine-motion devices by adjusting the large-scale motion
    device."""
    for axis, handlers in mover.axis_to_handlers.items():
        if len(set(handlers)) < 2:
            continue  # Only makes sense if one has at least two stages

        total_delta = 0
        for handler in handlers[1:]:
            # Assume that the fine-motion devices want to be in the center
            # of their ranges.
            cur_position = handler.getPosition()
            safeties = handler.get_hard_limits()
            target = (safeties[1] - safeties[0]) / 2 + safeties[0]
            handler.move_absolute(target)
            total_delta += target - cur_position
        handlers[0].moveRelative(-total_delta)


def go_to(position, should_block=False):
    """Move to the specified position using the current handler."""
    if len(position) != len(mover.axis_to_handlers.keys()):
        raise RuntimeError(
            "Asked to go to position with wrong number of axes (%d != %d)"
            % (len(position), len(mover.axis_to_handlers.keys()))
        )
    mover._go_to_axes(enumerate(position), should_block)


def go_to_xy(position, should_block=False):
    """As go_to, but only in X and Y."""
    if len(position) != 2:
        raise RuntimeError(
            "Asked to go to XY position with wrong number of axes (%d != %d)"
            % (len(position), 2)
        )
    mover._go_to_axes(enumerate(position), should_block)


def go_to_z(position, should_block=False):
    """As go_to, but only in Z."""
    mover._go_to_axes([(2, position)], should_block)


def move_relative(offset, should_block=False):
    """Move by the specified 3D offset."""
    num_axes = len(mover.axis_to_handlers.keys())
    if len(offset) != num_axes:
        raise RuntimeError(
            "Asked to move relatively with wrong number of axes (%d != %d)"
            % (len(offset), num_axes)
        )
    cur_position = get_position()
    vals = [offset[i] + cur_position[i] for i in range(num_axes)]
    go_to(vals, should_block)


def wait_for_stop(timeout=5):
    """Wait for any stage motion to cease."""
    for name, event in mover.name_to_stopped_event.items():
        if not event.wait(timeout):
            raise RuntimeError("Timed out waiting for %s to stop" % name)


def go_to_site(unique_id, should_block=False):
    """Move to the specified site."""
    site = mover.id_to_site[unique_id]
    obj_offset = wx.GetApp().Objectives.get_offset()
    offset_position = list(site.position)
    for i in range(len(offset_position)):
        offset_position[i] = offset_position[i] + obj_offset[i]
    go_to(offset_position, should_block)


def get_site(unique_id):
    """Get a Site with a given ID."""
    return mover.id_to_site[unique_id]


def save_site(new_site=None):
    """Save a new Site.

    Use default settings if none is provided.
    """
    if new_site is None:
        new_site = Site(get_position())
    mover.id_to_site[new_site.unique_id] = new_site
    # Start counting from the new site, if necessary.
    global unique_site_index
    unique_site_index = max(unique_site_index, new_site.unique_id)
    events.publish(events.NEW_SITE, new_site)


def delete_site(site_id):
    """Remove a site with the specified ID."""
    site = mover.id_to_site[site_id]
    del mover.id_to_site[site_id]
    events.publish(events.DELETE_SITE, site)
    # HACK: if this site_id is for the most-recently created
    # site, decrement the global site ID.
    global unique_site_index
    if site_id == unique_site_index:
        unique_site_index -= 1


def get_all_sites() -> typing.List[int]:
    """Retrieve the sites as a list."""
    return list(mover.id_to_site.values())


def does_site_exist(site_id):
    """Return True if there's a site with the specified ID."""
    return site_id in mover.id_to_site


def can_reach_site(site_id):
    """Return True iff the position of the specified site is inside all of our
    soft motion limits."""
    position = mover.id_to_site[site_id].position
    safeties = get_soft_limits()
    for axis, pos in enumerate(position):
        if safeties[axis][0] > pos or pos > safeties[axis][1]:
            return False
    return True


def write_sites_to_file(filename):
    """Record sites to a file."""
    with open(filename, "w") as handle:
        for id, site in mover.id_to_site.items():
            handle.write(site.serialize() + "\n")


def load_sites(filename):
    """Load sites from a file."""
    with open(filename, "r") as handle:
        for line in handle:
            site = deserialize_site(line)
            save_site(site)


def get_position():
    """Return the exact stage position, as the aggregate of all handlers'
    positions."""
    result = 3 * [0]
    for axis, handlers in mover.axis_to_handlers.items():
        for handler in set(handlers):
            result[axis] += handler.getPosition()
    return result


def get_position_for_axis(axis):
    """Return the exact stage position for the given axis."""
    result = 0
    for handler in set(mover.axis_to_handlers[axis]):
        result += handler.getPosition()
    return result


def get_all_positions():
    """Return a list of (X, Y, Z) tuples indicating the positions for all
    handlers we have.

    If there's an axis with more handlers than the others, then those axes will
    have None instead of a position towards the end of the list.
    """
    result = []
    for i in range(mover.n_stages):
        current = [None] * len(mover.axis_to_handlers)
        for axis, handlers in mover.axis_to_handlers.items():
            current[axis] = handlers[i].getPosition()
        result.append(tuple(current))
    return result


def get_cur_step_sizes():
    return mover.get_step_sizes()


def get_cur_handler_index():
    """Simple getter."""
    return mover.cur_handler_index


def get_hard_limits_for_axis(axis: int) -> AxisLimits:
    """Get the hard motion limits for a specific axis, as the summation of all
    limits for movers on that axis."""
    return mover._hard_limits[axis]


def get_hard_limits() -> StageLimits:
    """Repeat the above for each axis."""
    return mover._hard_limits


def get_individual_hard_limits(axis):
    """Returns a list of all hard motion limits for the given axis."""
    return [handler.get_hard_limits() for handler in mover.axis_to_handlers[axis]]


def get_soft_limits_for_axis(axis):
    """Get the soft motion limits for a specific axis, as the summation of all
    limits for movers on that axis."""
    low_limit = 0
    high_limit = 0
    for handler in set(mover.axis_to_handlers[axis]):
        low, high = handler.getSoftLimits()
        low_limit += low
        high_limit += high
    return (low_limit, high_limit)


def get_soft_limits():
    """Repeat the above for each axis."""
    result = []
    for axis in sorted(mover.axis_to_handlers.keys()):
        result.append(get_soft_limits_for_axis(axis))
    return result


def get_individual_soft_limits(axis):
    """Returns a list of all soft motion limits for the given axis."""
    return [handler.getSoftLimits() for handler in mover.axis_to_handlers[axis]]


def set_soft_limit(axis, value, is_max):
    """Try to change the software safety limits.

    Return True if we succeed, False otherwise.
    """
    # \todo For now we assume that only the first handler, with the greatest
    # range of motion, cares about soft limits.
    try:
        mover.axis_to_handlers[axis][0].setSoftLimit(value, is_max)
        events.publish(events.SOFT_SAFETY_LIMIT, axis, value, is_max)
        return True
    except Exception as e:
        # \todo Assuming that any exception here means the safety was not
        # set.
        return False


def set_soft_min(axis, value):
    set_soft_limit(axis, value, False)


def set_soft_max(axis, value):
    set_soft_limit(axis, value, True)


def move_z_check_mover_limits(target):
    # Use the nanomover (and, optionally, also the stage piezo) to
    # move to the target elevation.

    # Need to check current mover limits, see if we exceed them and if
    # so drop down to lower mover handler.
    original_mover = mover.cur_handler_index
    limits = get_individual_soft_limits(2)
    offset = target - get_position()[2]

    # FIXME: IMD 2018-11-07 I think this test needs to be currentpos
    # of the current mover not the overall pos.
    while mover.cur_handler_index >= 0:
        handler = mover.cur_handler_index
        mover_pos = get_all_positions()[handler][2]
        if (mover_pos + offset) > limits[mover.cur_handler_index][1] or (
                mover_pos + offset
        ) < limits[mover.cur_handler_index][0]:
            # need to drop down a handler to see if next handler can do the move
            mover.cur_handler_index -= 1
            if mover.cur_handler_index < 0:
                print("Move too large for any Z mover.")

        else:
            go_to_z(target)
            break

    # return to original active mover.
    mover.cur_handler_index = original_mover


def optimised_site_order(base_order):
    """Use the nearest-neighbor algorithm to select the order in which to visit
    the selected sites (i.e. try to solve the Traveling Salesman problem).

    This could, theoretically, behave worse than just taking the list in its
    default order, so we do check the default list's travel time and use it if
    it's superior, on the assumption that users will typically select sites in
    some basically sane order.

    Args:
        base_order (list): List of site IDs.
    """
    if len(base_order) == 0:
        return []
    marked_points = set()
    points_in_order = []
    total_tour_cost = 0
    cur_point = base_order[0]
    remaining_points = set(base_order)

    def distance(a, b):
        """Calculate the travel time between two sites.

        Since we move simultaneously in each axis, this is simply the maximum
        distance along any given axis.
        """
        p1 = mover.id_to_site[a].position
        p2 = mover.id_to_site[b].position
        return max([abs(a - b) for a, b in zip(p1, p2)])

    while remaining_points:
        marked_points.add(cur_point)
        points_in_order.append(cur_point)
        remaining_points.remove(cur_point)
        if not remaining_points:
            break

        # Find the closest remaining point
        next_point = min(remaining_points, key=lambda a: distance(a, cur_point))
        total_tour_cost += distance(cur_point, next_point)
        cur_point = next_point
    # Add cost of closing the loop back to the first point.
    total_tour_cost += distance(base_order[0], cur_point)

    # Calculate the cost of just doing everything in order.
    simple_tour_cost = 0
    for i in range(len(base_order)):
        simple_tour_cost += distance(
            base_order[i], base_order[(i + 1) % len(base_order)]
        )
    simple_tour_cost += distance(base_order[0], base_order[-1])

    if simple_tour_cost < total_tour_cost:
        # Nearest-neighbor is worse than just going in the
        # user-specified order
        return base_order.copy()
    return points_in_order
