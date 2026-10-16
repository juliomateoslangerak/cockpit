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

"""Aerotech stages."""

import re
import socket
from time import sleep

import cockpit.handlers.stage_positioner
from cockpit import events
from cockpit.devices import device


NAME_STRING = "aerotech mover"
LIMITS_PAT = r"(?P<limits>\(?\s*[-]?\d*\s*,\s*[-]?\d*\s*\)?)"

## Characters used by the SoloistCP controller.
# CommandTerminatingCharacter
CTC = bytes((10,))
# CommandSuccessCharacter
CSC = bytes((37,))
# CommandInvalidCharacter
CIC = bytes((33,))
# CommandFaultCharacter
CFC = bytes((35,))
# CommandTimeOutCharacter
CTOC = bytes((36,))
# Map character to meaning
RESPONSE_CHARS = {
    CSC: b"command success",
    CIC: b"command invalid",
    CFC: b"command caused fault",
    CTOC: b"command timeout",
}


class AerotechZStage(device.Device):
    """Cockpit aerotechAxis

    This class creates a simple stage-positioning device.  Sample
    config entry:

    .. code:: ini

        [aerotech]
        type: cockpit.devices.aerotech_axis.AerotechZStage
        ipAddress: 192.168.0.5
        port: 8000
        softlimits: -45000, 5000

    """

    def __init__(self, name, config={}):
        super().__init__(name, config)
        try:
            limit_string = config.get("softlimits", "")
            parsed = re.search(LIMITS_PAT, limit_string)
            if not parsed:
                # Could not parse config entry.
                raise Exception("Bad config: Aerotech Limits.")
                # No transform tuple
            else:
                lstr = parsed.groupdict()["limits"]
                self.softlimits = eval(lstr)
        except:
            print("No softlimits section setting default limits")
            self.softlimits = (-30000, 7000)

        # Subscribe to abort events.
        events.subscribe(events.USER_ABORT, self.on_abort)
        # The cockpit axis does this stage moves along.
        self.axis = int(config.get("axis", 2))
        # Socket used to communicate with controller.
        self.socket = None
        # Last known position (microns)
        self.position = None
        # Axis acceleration (mm / s^2)
        self.acceleration = 200
        # Axis maximum speed (mm / s^2)
        self.speed = 20

    def command(self, cmd):
        """Send a command to the Aerotech SoloistCP and fetch the response."""
        # The terminated command string.
        cmdstr = cmd + CTC
        # A flag indicating that a retry is needed.
        retry = False

        ## Try to send the command.
        try:
            self.socket.send(cmdstr)
        except socket.error as e:
            retry = True
        except:
            raise

        ## Was there a connection error?
        if retry:
            self.open_connection()
            try:
                self.socket.send(cmdstr)
            except:
                raise

        ## Fetch and parse the response
        response = self.socket.recv(256)
        # Response status character:
        response_char = response[0:1]
        if response_char == bytes((255,)):
            # stray character
            response_char = response[1:2]
            response_data = response[2:]
        else:
            response_data = response[1:]

        # Did the controlle report a problem?
        if response_char != CSC and response in RESPONSE_CHARS:
            raise Exception(
                "Aerotech controller error - %s."
                % RESPONSE_CHARS[response_char].decode()
            )
        elif response_char != CSC:
            raise Exception(
                (
                    "Aerotech controller error - unknown response %s"
                    % response_char.decode()
                )
            )
        else:
            return response_data

    def get_handlers(self):
        result = []
        axis = self.axis
        # IMD 2015-03-02 changed hard limits to reflect DeepSIM should go into config file
        min_val = self.softlimits[0]
        max_val = self.softlimits[1]
        handler = cockpit.handlers.stage_positioner.PositionerHandler(
            "%d %s" % (axis, NAME_STRING),
            "%d stage motion" % axis,
            False,
            {
                "moveAbsolute": self.move_absolute,
                "moveRelative": self.move_relative,
                "getPosition": self.get_position,
                "getMovementTime": self.get_movement_time,
            },
            axis,
            (min_val, max_val),
            (min_val, max_val),
        )
        result.append(handler)
        return result

    def initialize(self):
        """Initialize the axis."""
        # Open a connection to the controller.
        self.open_connection()
        self.position = self.command(b"CMDPOS").decode()
        self.speed = int(self.command(b"GETPARM(71)"))
        self.acceleration = int(self.command(b"GETPARM(72"))

    def make_initial_publications(self):
        """Publish our current position."""
        axis = self.axis
        events.publish(events.STAGE_MOVER, axis)

    def on_abort(self):
        """User clicked the abort button; stop moving."""
        self.command(b"ABORT")
        axis = self.axis
        events.publish(events.STAGE_STOPPED, "%d %s" % (axis, NAME_STRING))

    def move_absolute(self, axis, pos):
        """Move the stage to a given position."""
        self.command(b"ENABLE")
        self.command(b"MOVEABS D %f F %f" % (pos / 1000.0, self.speed))
        events.publish(events.STAGE_MOVER, axis)
        # Wait until the move has finished - status bit 2 is InPosition.
        while not int(self.command(b"AXISSTATUS")) & (1 << 2):
            sleep(0.1)
        self.position = self.command(b"CMDPOS").decode()
        events.publish(events.STAGE_MOVER, axis)
        events.publish(events.STAGE_STOPPED, "%d mover" % axis)
        self.command(b"DISABLE")

    def move_relative(self, axis, delta):
        """Move the stage piezo by a given delta."""
        self.command(b"ENABLE")
        self.command(b"MOVEINC D %f F %f" % (delta / 1000.0, self.speed))
        events.publish(events.STAGE_MOVER, axis)
        # Wait until the move has finished - status bit 2 is InPosition.
        while not int(self.command(b"AXISSTATUS")) & (1 << 2):
            sleep(0.1)
        self.position = self.command(b"CMDPOS").decode()
        events.publish(events.STAGE_MOVER, axis)
        events.publish(events.STAGE_STOPPED, "%d %s" % (axis, NAME_STRING))
        self.command(b"DISABLE")

    def get_position(self, axis):
        """Get the current piezo position."""
        return float(self.position) * 1000.0

    def get_movement_time(self, axis, start, end):
        """Get the amount of time it would take the mover to move from the
        initial position to the final position, as well as the amount of time
        needed to stabilize after that point, both in milliseconds. This is
        needed when setting up timings for experiments.
        """
        # top speed mm/s
        v = self.speed
        # acceleration mm/s**2
        a = self.acceleration
        # displacement - passed as um
        d_s = abs(start - end) / 1000.0
        # acceleration / braking distance
        s_acc = 0.5 * v**2 / a

        # Determine time required for move, in ms.
        if d_s < 2 * s_acc:
            dt = 1000 * 2 * (d_s / a) ** 0.5
        else:
            dt = 1000 * ((d_s / v) + (v / a))

        # Add a few ms to allow for any network latency (measured at < 10ms).
        dt += 10

        # The stage slows gradually to a stop so settling time is small.
        # Allow one or two servo cycles - servo rate is 1kHz.
        return (dt, 2)

    def open_connection(self):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            self.socket.connect((self.ip_address, self.port))
        except:
            raise

    def home(self):
        """function to home the stage, should be wrapped with a dialog to
        ensure the stage plate is removed."""
        self.command(b"ENABLE")
        self.command(b"HOME")
        self.command(b"DISABLE")
