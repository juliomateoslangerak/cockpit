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
import socket
import threading
import time

import cockpit.handlers.stage_positioner
import cockpit.util.threads
from cockpit import events
from cockpit.devices import device


_logger = logging.getLogger(__name__)


## TODO: test with hardware.


# few commnads
# pr - position relative
# pa - position aboslute
# mm - position mode (1 -= closed loop)
# sn - units mode 0= steps, 1= counts from encoder
# dh - set home
# md - motion done? 1=finished 0= in motion
# fe - feedback error limit default = 1000
# cl - closeed loop update freq in sec min = 0.1
# ab - abort motion
# mt - move to limit (suggest index on -ve limit on all channels)
# tp? - query position.
# or - find home? not sure what this means with current stages

# Need to sort out defined positions. How do we know if the system has been homed?
# How dowes this compare to "1 or"? I have no clue!
# movement range is ~   335926 counts - need to measure distance.
# moving from one end of the range to the other takes a LONG time.
#
# home routine
# set units to counts (eg encoder)
# 1 sn 1
# enguage closed loop mode
# 1 mm 1
# closed loop timing 0.1s
# 1 cl 0.1
# move to neagtive limit
# 1 mt -
# Set curent pos to 0
# 1 dh
# move back to mid position (need to check position)
# 1 pa 10000


class PicoMotorDevice(device.Device):
    """Newport picomotor controllers.

    The configuration section for these devices requires the following
    values:

    ``cal``
        Calibration.
    ``ipAddress``
        The IP address of the controller.
    ``port``
        The port the controller listens on.

    """

    _config_types = {
        "cal": float,
        "port": int,
    }

    def __init__(self, name, config):
        super().__init__(name, config)
        self.STAGE_CAL = config.get("cal")  # e.g. 13.750
        self.PICO_CONTROLLER = config.get("ipaddress")  # e.g. 172.16.0.30'
        self.PICO_PORT = config.get("port")  # e.g. 23

        ## Maps the cockpit's axis ordering (0: X, 1: Y, 2: Z) to the
        # XY stage's ordering which is
        # x controller 1, motor 1 - '1>1'
        # y controller 1, motor 2 - '1>2'
        # z controller 2, motor 1 - '2>1'
        # Needs moving to config file!
        self.axis_mapper = {0: "1>1", 1: "1>2", 2: "2>1"}

        ## Connection to the Z piezo controller (Pyro4.Proxy of a
        # telnetlib.Telnet instance)
        self.z_connection = None
        ## Lock around sending commands to the Z piezo controller.
        self.z_lock = threading.Lock()
        ## Connection to the XY stage controller (serial.Serial instance)
        self.xy_connection = None
        ## Lock around sending commands to the XY stage controller.
        self.xy_lock = threading.Lock()
        ## Cached copy of the stage's position. Initialized to an impossible
        # value; this will be modified in initialize.
        self.xy_position_cache = [10**100, 10**100, 7500]

        events.subscribe(events.USER_ABORT, self.on_abort)

    def initialize(self):
        self.xy_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.xy_connection.connect((self.PICO_CONTROLLER, self.PICO_PORT))

        # IMD 20130421 reset at startup as the cointroller can go a bit mad
        # Need long timeout as ethernet takes a while to come back up
        self.xy_connection.settimeout(2)
        result = self.send_xy_command("RS", 0)
        time.sleep(2)
        self.xy_connection.close()
        # restart the socket now we have rest the controller.
        self.xy_connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.xy_connection.connect((self.PICO_CONTROLLER, self.PICO_PORT))
        self.xy_connection.settimeout(1)

        # 20130418 - the controller dumps a line of rubbish before we start...
        # give it time to wake up
        result = self.get_xy_response(1)
        result = self.send_xy_command("VE?", 1)
        #        print ("init controller firmware=", result)
        self.xy_connection.settimeout(2)

        # Get the proper initial position.
        # Need to init in some way here.
        # The home functions do this but are VERY slow
        self.get_xy_position(should_use_cache=False)

    def home_motors(self):
        """Routine to home all 3 motors. record position move to negative
        threshold (z axis should go to +ve limit so as not to ram objective)
        check if moving, sleep and loop record new pos set home move to
        start-end position to return to starting position loop through each
        axis."""
        orig_position = self.get_xy_position(should_use_cache=False)
        for axis in range(0, 2):
            print(
                "homeing axis 1 %s, origPosiiton=%d"
                % (self.axis_mapper[axis], orig_position[axis])
            )
            controller, motor = self.axis_mapper[axis].split(">")
            while self.check_for_motion(controller) == 1:
                time.sleep(1)
            # Home this axis (to -ve home)
            self.send_xy_command("%s mt -" % (self.axis_mapper[axis]), 0)
            # wait for home to be done
            while self.check_for_motion(controller) == 1:
                time.sleep(1)
            # record new position
            newposition = self.get_xy_position(should_use_cache=False)
            # set to pos zero
            self.send_xy_command("%s dh " % (self.axis_mapper[axis]), 0)
            # move to pos +100 absolute to make sure we aren't in negative positions
            self.send_xy_command("%s pa 100" % (self.axis_mapper[axis]), 0)
            # calculate how to move back to where we were
            endposition[axis] = -newposition[axis] + orig_position[axis]

        print("home done now returning to last position", endposition)
        for axis in range(0, 2):
            while self.check_for_motion(controller) == 1:
                time.sleep(1)
            self.send_xy_command(
                "%s pa %s" % (self.axis_mapper[axis]), endposition[axis], 0
            )

    def check_for_motion(self, controller):
        """Check for motion on a given controller."""
        motor_state_1 = self.send_xy_command("%s>1 md" % (controller), 1)
        motor_state_2 = self.send_xy_command("%s>2 md" % (controller), 1)
        # need to split of controller number if we have more than one. Use
        # the format of axis 0 axis_mapper string to check this.
        if len(self.axis_mapper[0].split(">")) == 2:
            motor_state_1 = motor_state_1.split(">")[1]
            motor_state_2 = motor_state_2.split(">")[1]

        return motor_state_1 or motor_state_2

    # Routine to home all 3 motors.
    # record position
    # move to negative threshold (z axis should go to +ve limit so as not to ram objective)
    # check if moving, sleep and loop
    # record new pos
    # set home
    # move to start-end position to resturn to starting position
    # loop through each axis

    def home_motors(self):
        orig_position = self.get_xy_position(should_use_cache=False)
        for axis in range(0, 2):
            print(
                "homeing axis 1 %s, origPosiiton=%d"
                % (self.axis_mapper[axis], orig_position[axis])
            )
            controller, motor = self.axis_mapper[axis].split(">")
            while self.check_for_motion(controller) == 1:
                time.sleep(1)
            # Home this axis (to -ve home)
            self.send_xy_command("%s mt -" % (self.axis_mapper[axis]), 0)
            # wait for home to be done
            while self.check_for_motion(controller) == 1:
                time.sleep(1)
            # record new position
            newposition = self.get_xy_position(should_use_cache=False)
            # set to pos zero
            self.send_xy_command("%s dh " % (self.axis_mapper[axis]), 0)
            # move to pos +100 absolute to make sure we aren't in negative positions
            self.send_xy_command("%s pa 100" % (self.axis_mapper[axis]), 0)
            # calculate how to move back to where we were
            endpositon[axis] = -newposition[axis] + oldposition[axis]

        print("home done now returning to last position", endposition)
        for axis in range(0, 2):
            while self.check_for_motion(controller) == 1:
                time.sleep(1)
            self.send_xy_command(
                "%s pa %s" % (self.axis_mapper[axis]), endposition[axis], 0
            )

    ## Send a command to the XY stage controller, read the response, check
    # for errors, and either raise an exception or return the response.
    # Very similar to sendZCommand of course, but xy_connection is a Serial
    # instance and z_connection is a Telnet instance.

    def check_for_motion(self, controller):
        motor_state_1 = self.send_xy_command("%s>1 md" % (controller), 1)
        motor_state_1 = motor_state_1.split(">")[1]
        motor_state_2 = self.send_xy_command("%s>2 md" % (controller), 1)
        motor_state_2 = motor_state_2.split(">")[1]
        return motor_state_1 or motor_state_2

    def send_xy_command(
        self, command, num_expected_lines=1, should_check_errors=True
    ):
        """Send a command to the XY stage controller, read the response, check
        for errors, and either raise an exception or return the response."""
        with self.xy_lock:
            self.xy_connection.sendall(command + "\n")
            # IMD 09052013 The controller needs a little time after a
            # command to digest it
            time.sleep(0.05)
            if num_expected_lines > 0:
                try:
                    response = self.xy_connection.recv(1024)
                except:
                    _logger.debug(
                        "in command %s, %d, No response",
                        command,
                        num_expected_lines,
                    )
                return response

    def get_xy_response(self, num_expected_lines):
        """Read a response off of the XY connection.

        We do this one character at a time in the hopes of avoiding having to
        wait for a timeout. We stop when we hit the right number of newlines.
        """
        response = ""
        num_lines = 0
        while True:
            output = self.xy_connection.recv(1024)
            _logger.debug("Picomotor responce %s", output)
            response += output
            num_lines += 1
            if num_lines == num_expected_lines:
                break
            else:
                # No output; must be done.
                break
        return response

    def on_exit(self):
        """When the user logs out close the network connection."""
        self.xy_connection.close()

    def on_abort(self, *args):
        """Halt motion when the user aborts."""
        self.send_xy_command("AB", 0)

    def get_handlers(self):
        # IMD 20130418 hacked to include Z in this for the Picomovers stage
        result = []
        for axis, min_pos, max_pos in [
            (0, -10000, 10000),
            (1, -10000, 10000),
            (2, -1000, 1000),
        ]:
            result.append(
                cockpit.handlers.stage_positioner.PositionerHandler(
                    "%d PI mover" % axis,
                    "%d stage motion" % axis,
                    False,
                    {
                        "move_absolute": self.move_xy_absolute,
                        "moveRelative": self.move_xy_relative,
                        "getPosition": lambda axis=axis: self.get_xy_position(
                            axis=axis
                        ),
                        "getMovementTime": self.get_xy_movement_time,
                    },
                    axis,
                    (min_pos, max_pos),
                    (min_pos, max_pos),
                )
            )
        return result

    def get_xy_movement_time(self, axis, start, end):
        distance = abs(end - start)
        # IMD15072014 closed loop performance is much slower, or counts != steps.
        # speed is roughly 10,000 counts in 30 secs
        return ((distance * self.STAGE_CAL) / 300) + 0.25

    def move_xy_absolute(self, axis, pos):
        # IMD 20130418 need to calibrate stage properly, approx is 67 steps/um
        # IMD 20130530 suggest by Chris just flip X movemets to fix
        # wrong X and Y directions
        if (axis == 0) or (axis == 1):
            pos = -pos
        self.send_xy_command(
            "%s PA %d" % (self.axis_mapper[axis], int(pos * self.STAGE_CAL)), 0
        )
        self.send_xy_position_updates()

    def move_xy_relative(self, axis, delta):
        # IMD 20130418 need to calibrate stage properly, approx is 67 steps/um
        # print "moving pr %d, %f",axis, delta
        if delta != 0:
            # IMD 20130530 flip X axis to get correct direction.
            if (axis == 0) or (axis == 1):
                delta = -delta

            self.send_xy_command(
                "%s PR %d"
                % (self.axis_mapper[axis], int(delta * self.STAGE_CAL)),
                0,
            )
            self.send_xy_position_updates()

    @cockpit.util.threads.call_in_new_thread
    def send_xy_position_updates(self):
        """Send updates on the XY stage's position, until it stops moving."""
        prev_x, prev_y, prev_z = self.xy_position_cache
        while True:
            x, y, z = self.get_xy_position(should_use_cache=False)
            delta = abs(x - prev_x) + abs(y - prev_y) + abs(z - prev_z)
            if delta < 0.3:
                # No movement since last time; done moving.
                for axis in [0, 1, 2]:
                    events.publish(events.STAGE_STOPPED, "%d PI mover" % axis)
                return
            for axis in [0, 1, 2]:
                events.publish(events.STAGE_MOVER, axis)
            prev_x, prev_y, prev_z = (x, y, z)
            time.sleep(0.1)

    def get_xy_position(self, axis=None, should_use_cache=True):
        """Get the position of the specified axis, or all axes by default.

        If should_use_cache is not set, then we will query the controller, which
        takes some time.
        """
        # +++        self.xy_position_cache = (0, 0,0)
        if not should_use_cache:
            if axis is not None:
                position = self.send_xy_command(
                    "%s TP?" % (self.axis_mapper[axis]), 1, False
                )
                if len(self.axis_mapper[axis].split(">")) == 2:
                    position = position.split(">")[1]
                self.xy_position_cache[axis] = float(position) / self.STAGE_CAL
                return self.xy_position_cache[axis]
            else:
                # axis is None so grab all positions.
                for ax in range(len(self.axis_mapper)):
                    position = self.send_xy_command(
                        "%s TP?" % (self.axis_mapper[ax]), 1, False
                    )
                    # have more than one controller so need to split
                    # controller number from position.
                    if len(self.axis_mapper[ax].split(">")) == 2:
                        position = position.split(">")[1]
                    # Positions are in steps, and we need microns.
                    self.xy_position_cache[ax] = float(position) / self.STAGE_CAL

        # returning cached values, either single axis or all.
        if axis is not None:
            return self.xy_position_cache[axis]
        else:
            return self.xy_position_cache

    def make_initial_publications(self):
        self.send_xy_position_updates()
