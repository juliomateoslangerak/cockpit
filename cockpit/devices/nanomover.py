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

import re
import time

import cockpit.gui.gui_utils
import cockpit.handlers.generic_positioner
import cockpit.handlers.stage_positioner
import cockpit.interfaces
import cockpit.util.connection
import cockpit.util.threads
import cockpit.util.user_config
from cockpit import events
from cockpit.devices.device import Device


## TODO: test with hardware.

LIMITS_PAT = r"(?P<limits>\(\s*\(\s*[-]?\d*\s*,\s*[-]?\d*\s*\)\s*,\s*\(\s*[-]?\d*\s*\,\s*[-]?\d*\s*\)\s*,\s*\(\s*[-]?\d*\s*,\s*[-]?\d*\s*\)\s*\))"


class Nanomover(Device):
    def __init__(self, name, config={}):
        super().__init__(name, config)
        ## Current stage position information.
        self.cur_position = [14500, 14500, 14500]
        ## Connection to the Nanomover controller program.
        self.connection = None
        # Maps the cockpit's axis ordering (0: X, 1: Y, 2: Z) to the
        # XY stage's ordering (1: Y, 2: X,0: Z)
        self.axis_mapper = {0: 0, 1: 1, 2: 2}
        ## Cached copy of the stage's position. Initialized to an impossible
        # value; this will be modified in initialize.
        self.position_cache = (10, 10, 10)
        ## Maps cockpit axis ordering to a +-1 multiplier to apply to motion,
        # since some of our axes are flipped.
        self.axis_sign_mapper = {0: 1, 1: 1, 2: 1}
        ## Time of last action using the piezo; used for tracking if we should
        # disable closed loop.
        self.timeout = float(config.get("timeout", 0.1))
        try:
            limit_string = config.get("softlimits", "")
            parsed = re.search(LIMITS_PAT, limit_string)
            if not parsed:
                # Could not parse config entry.
                raise Exception("Bad config: Nanomover Limits.")
                # No transform tuple
            else:
                lstr = parsed.groupdict()["limits"]
                self.softlimits = eval(lstr)
                self.safeties = eval(lstr)
        except:
            print("No softlimits section setting default limits")
            self.softlimits = [[0, 25000], [0, 25000], [7300, 25000]]
            self.safeties = [[0, 25000], [0, 25000], [7300, 25000]]

        # a useful middle position for after a home
        self.middle_xy = (
            (self.safeties[0][1] - self.safeties[0][0]) / 2.0,
            (self.safeties[0][1] - self.safeties[0][0]) / 2.0,
        )
        events.subscribe(events.USER_ABORT, self.on_abort)
        events.subscribe(
            events.COCKPIT_INIT_COMPLETE, self.prompt_exercise_stage
        )

    def initialize(self):
        self.connection = cockpit.util.connection.Connection(
            "nano", self.ip_address, self.port
        )
        self.connection.connect(self.receiveData)
        self.cur_position[:] = self.connection.connection.posXYZ_OMX()
        if self.cur_position == [0, 0, 0]:
            print("Homing Nanomover")
            self.connection.connection.startOMX()
            self.home()
            cockpit.interfaces.stage_mover.go_to_xy(
                self.middle_xy, should_block=True
            )

    def prompt_exercise_stage(self):
        """We want to periodically exercise the XY stage to spread the grease
        around on its bearings; check how long it's been since the stage was
        last exercised, and prompt the user if it's been more than a week."""
        last_exercise_timestamp = cockpit.util.user_config.get_value(
            "NanomoverLastExerciseTimestamp", default=0
        )
        cur_time = time.time()
        delay = cur_time - last_exercise_timestamp
        days_passed = delay / float(24 * 60 * 60)
        if days_passed > 7 and cockpit.gui.gui_utils.get_user_permission(
            ("It has been %.1f days since " % days_passed)
            + "the stage was last exercised. Please exercise "
            + "the stage regularly.\n\nExercise stage?",
            "Please exercise the stage",
        ):
            # Move to the middle of the stage, then to one corner, then to
            # the opposite corner, repeat a few times, then back to the middle,
            # then to where we started from. Positions are actually backed off
            # slightly from the true safeties. Moving to the middle is
            # necessary to avoid the banned rectangles, in case the stage is
            # in them when we start.
            initial_pos = tuple(self.position_cache)
            #            cockpit.interfaces.stage_mover.go_to_xy((0, 0), should_block = True)
            for i in range(5):
                print("Rep %d of 5..." % i)
                for position in self.softlimits[0:2]:
                    cockpit.interfaces.stage_mover.go_to_xy(
                        position, should_block=True
                    )
            cockpit.interfaces.stage_mover.go_to_xy(
                self.middle_xy, should_block=True
            )
            cockpit.interfaces.stage_mover.go_to_xy(initial_pos, should_block=True)
            print("Exercising complete. Thank you!")

            cockpit.util.user_config.set_value(
                "NanomoverLastExerciseTimestamp", time.time()
            )

    def perform_subscriptions(self):
        events.subscribe(events.USER_ABORT, self.on_abort)

    def make_initial_publications(self):
        self.publish_position()
        self.send_xy_position_updates()

    ## The XY Macro Stage view is painting itself; draw the banned
    # rectangles as pink excluded zones.
    #    def onMacroStagePaint(self, stage):
    #        glColor3f(1, .6, .6)
    #        glBegin(GL_QUADS)
    #        for (x1, y1), (x2, y2) in BANNED_RECTANGLES:
    #            stage.scaledVertex(x1, y1)
    #            stage.scaledVertex(x1, y2)
    #            stage.scaledVertex(x2, y2)
    #            stage.scaledVertex(x2, y1)
    #        glEnd()

    def get_handlers(self):
        result = []
        for axis in range(3):
            low_limit, high_limit = self.safeties[axis]
            soft_low_limit, soft_high_limit = self.softlimits[axis]
            result.append(
                cockpit.handlers.stage_positioner.PositionerHandler(
                    "%d nanomover" % axis,
                    "%d stage motion" % axis,
                    False,
                    {
                        "move_absolute": self.move_absolute,
                        "move_relative": self.move_relative,
                        "get_position": self.get_position,
                    },
                    axis,
                    (soft_low_limit, soft_high_limit),
                    (low_limit, high_limit),
                )
            )
        return result

    def publish_position(self):
        """Publish the current stage position."""
        for axis in range(3):
            events.publish(events.STAGE_MOVER, axis)

    @cockpit.util.threads.call_in_new_thread
    def send_xy_position_updates(self):
        """Send updates on the XY stage's position, until it stops moving."""
        while True:
            prev_x, prev_y = self.position_cache[:2]
            x, y = self.get_position(should_use_cache=False)[:2]
            delta = abs(x - prev_x) + abs(y - prev_y)
            if delta < 2:
                # No movement since last time; done moving.
                for axis in [0, 1]:
                    events.publish(events.STAGE_STOPPED, "%d nanomover" % axis)
                return
            for axis in [0, 1]:
                events.publish(events.STAGE_MOVER, axis)
            time.sleep(0.1)

    def get_position(self, axis=None, should_use_cache=True):
        if not should_use_cache:
            position = self.connection.connection.posXYZ_OMX()
            x = float(position[self.axis_mapper[0]]) * self.axis_sign_mapper[0]
            y = float(position[self.axis_mapper[1]]) * self.axis_sign_mapper[1]
            z = float(position[self.axis_mapper[2]]) * self.axis_sign_mapper[2]
            self.position_cache = (x, y, z)
        if axis is None:
            return self.position_cache
        return self.position_cache[axis]

    def receiveData(self, *args):
        """Receive information from the Nanomover control program."""
        if args[0] == "nanoMotionStatus":
            self.cur_position[:] = args[1]
            self.publish_position()
            if args[-1] == "allStopped":
                for i in range(3):
                    events.publish(events.STAGE_STOPPED, "%d nanomover" % i)

    def move_absolute(self, axis, pos):
        """Move a specific axis to a given position."""
        self.send_xy_position_updates()
        self.connection.connection.moveOMX_axis(axis, pos)

    def move_relative(self, axis, delta):
        """Move a specific axis by a given amount."""
        self.send_xy_position_updates()
        self.connection.connection.moveOMX_dAxis(axis, delta)

    def on_abort(self, *args):
        """User clicked the abort button; halt motion."""
        self.connection.connection.stopOMX()

    def home(self):
        """function to home stage if needed."""
        # keep a copy of the softlimits.
        #        realSoftlimits= copy.copy(self.softlimits)
        #        self.softlimits = [[-25000, 25000],
        #                           [-25000, 25000],
        #                           [7300, 50000]]
        # home the stage which moves to lrage negative positon until it
        # hits the hard limit switch
        self.connection.connection.findHome_OMX()
        self.send_xy_position_updates()
        self.position_cache = self.get_position(should_use_cache=False)
        # reset softlimits to their original value


#        self.softlimits=realSoftlimits
