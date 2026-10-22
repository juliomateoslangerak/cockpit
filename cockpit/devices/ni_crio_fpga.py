#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2021 Centre National de la Recherche Scientifique (CNRS)
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


## This module handles interacting with the National Instruments cRIO-9068 that sends the digital and
# analog signals that control our light sources, cameras, and piezos. In

# particular, it effectively is solely responsible for running our experiments.
# As such it's a fairly complex module.
#

# A few helpful features that need to be accessed from the commandline:
# 1) A window that lets you directly control the digital and analog outputs
#    of the FPGA.
# >>> import devices.fpga as FPGA
# >>> FPGA.makeOutputWindow()
#
# 2) Create a plot describing the actions that the NI-FPGA set up in the most
#    recent experiment profile.
# >>> import devices.fpga as FPGA
# >>> FPGA._deviceInstance.plotProfile()
#
# 3) Manually advance the SLM forwards some number of steps; useful for when
#    it has gotten offset and is no longer "resting" on the first pattern.
# >>> import devices.fpga as FPGA
# >>> FPGA._deviceInstance.advanceSLM(numSteps)
# (where numSteps is an integer, the number of times to advance it).

"""exemple configuration
[cRIO]
type: cockpit.devices.ni_cRIOFPGA.NIcRIO
ipAddress = 10.6.19.12
sendport = 5000
receiveport = 6666
tickrate = 100
dLines = 24
aLines = 4
"""

import json
import socket
import threading
import time
from itertools import chain
from time import sleep

import numpy as np

import cockpit.handlers.executor
import cockpit.handlers.imager
import cockpit.handlers.light_source
import cockpit.handlers.stage_positioner
import cockpit.util.connection
import cockpit.util.threads
from cockpit import depot, events
from cockpit.devices import executor_devices


COCKPIT_AXES = {"x": 0, "y": 1, "z": 2, "SI angle": -1}
FPGA_IDLE_STATE = 3
FPGA_ABORTED_STATE = 4
FPGA_HEARTBEAT_RATE = (
    0.1  # At which rate is the FPGA sending update status signals
)
FPGA_HEARTBEAT_MAX_MSG_LEN = 2048
MASTER_IP = "10.6.19.11"


class NIcRIO(executor_devices.ExecutorDevice):
    _config_types = {
        "ipaddress": str,
        "sendport": int,
        "receiveport": int,
        "tickrate": int,
        "analog_lines": int,
        "digital_lines": int,
    }

    def __init__(self, name, config):
        super().__init__(name, config)
        self.tickrate = config.get(
            "tickrate"
        )  # Number of ticks per ms. As of the resolution of the action table.
        self.send_port = config.get("sendport")
        self.receive_port = config.get("receiveport")
        self.port = [self.send_port, self.receive_port]
        self._dlines = self.config.get("digital_lines")
        self._alines = self.config.get("analog_lines")
        self._current_analogs = self._alines * [0]
        # Absolute positions prior to the start of the experiment.
        self._last_analogs = self._alines * [0]
        # Store last movement profile for debugging
        self._last_profile = None

    @cockpit.util.threads.locked
    def initialize(self):
        """Connect to ni's RT-ipAddress computer. Overrides ExecutorDevice's initialize."""
        self.connection = Connection(
            parent=self,
            ip_address=self.ip_address,
            port=self.port,
            local_ip=MASTER_IP,
        )
        self.connection.connect()
        self.connection.Abort()

    def on_exit(self) -> None:
        if self.connection is not None:
            self.connection.disconnect()
        self.connection = None

    def on_prepare_for_experiment(
        self, *args
    ):  # TODO: Verify here for weird z movements
        super().on_prepare_for_experiment(*args)
        self._last_analogs = [
            self.connection.ReadPosition(a) for a in range(self._alines)
        ]
        self._last_analogs = list(self._current_analogs)
        self._last_digital = self.connection.ReadDigital()

    def experiment_done(self):
        events.publish(events.EXECUTOR_DONE % self.name)

    def get_analog(self, line):
        """Returns the current output value of the analog line in native units
        line is an integer corresponding to the requested analog on the FPGA
        as entered in the analog config files.
        """
        line = "Analogue " + str(line)
        return self.connection.status.get_status(line)

    def set_analog(self, line, target):
        """Set analog position in native units.

        Args:
            line: Analog line to change.
            target: target value.
        """
        return self.connection.MoveAbsolute(line, target)

    def takeImage(self):
        pass

    def _adapt_actions(self, actions):
        """Adapt tha actions table to the cRIO. We have to:
        - convert float in ms to integer clock ticks
        - separate analogue and digital events into different lists
        - generate a structure that describes the profile
        """
        # Profiles
        analogs = [
            [] for x in range(self._alines)
        ]  # A list of lists (one per channel) of tuples (ticks, (analog values))
        digitals = list()  # A list of tuples (ticks, digital state)
        # # Need to track time of last analog events
        # t_last_analog = None

        for t, (digital_args, analog_args) in actions:
            # Convert t to ticks as int while rounding up. The rounding is
            # necessary, otherwise e.g. 10.1 and 10.1999999... both result in 101.
            ticks = int(float(t) * self.tickrate + 0.5)

            # Digital actions - one at every time point.
            if len(digitals) == 0:
                digitals.append((ticks, digital_args))
            elif (
                ticks == digitals[-1][0]
            ):  # TODO: verify if we need this for the FPGA
                # Used to check for conflicts here, but that's not so trivial.
                # We need to allow several bits to change at the same time point, but
                # they may show up as multiple events in the actionTable. For now, just
                # take the most recent state.
                if digital_args != digitals[-1][1]:
                    digitals[-1] = (ticks, digital_args)
                else:
                    pass
            else:
                digitals.append((ticks, digital_args))

            # Analogue actions - only enter into profile on change.
            # NI-cRIO uses absolute values.
            for analog, analog_arg in zip(analogs, analog_args):
                if len(analog) == 0:  # analogs list is empty
                    analog.append((ticks, analog_arg))
                elif analog[-1][1] != analog_arg:
                    analog.append((ticks, analog_arg))
                else:
                    pass

        # Update records of last positions.
        self._last_digital = digitals[-1][1]
        self._last_analogs = map(
            lambda x, y: x - (y[-1:][1:] or 0), self._last_analogs, analogs
        )

        # Convert digitals to array of uints.
        digitals_arr = np.array(digitals, dtype=np.uint32).reshape(-1, 2)
        # Convert analogs to array of uints.
        analogs_arr = [
            np.array(a, dtype=np.uint32).reshape(-1, 2) for a in analogs
        ]

        # Create a description dict. Will be byte-packed by server-side code.
        maxticks = max(
            chain(
                [d[0] for d in digitals],
                [a[0] for a in chain.from_iterable(analogs)],
            )
        )

        description = {
            "count": maxticks,
            "clock": 1000.0 / float(self.tickrate),
            "InitDio": self._last_digital,
            "nDigital": len(digitals),
            "nAnalog": [len(a) for a in analogs],
        }

        self._last_profile = (description, digitals_arr, analogs_arr)

        return [description, digitals_arr, [*analogs_arr]]

    @cockpit.util.threads.locked
    def run_sequence(self, sequence):
        """Runs a sequence of times-digital pairs"""
        # Convert the times into ticks
        sequence = [(int(t * self.tickrate), d) for t, d in sequence]
        self.connection.run_sequence_on_fpga(sequence)

    @cockpit.util.threads.locked
    def take_burst(self, frame_count=10):
        """
        Use the internal triggering of the camera to take a burst of images

        Experimental
        """
        camera_mask = 0
        light_time_pairs = list()
        max_time = 0
        for handler, line in self.handler_to_digital_line.items():
            if handler.name in self.active_lights:
                max_time = max(max_time, handler.get_exposure_time())
                exposure_time = handler.get_exposure_time()
                light_time_pairs.append((line, exposure_time))
                max_time = max(max_time, exposure_time)
        for name, line in self.name_to_digital_line.items():
            if name in self.active_cameras:
                camera_mask += line
                handler = depot.get_handler_with_name(name)
                handler.set_exposure_time(max_time)

        sleep(5)


class Connection:
    """This class handles the connection with NI's RT-ipAddress computer."""

    def __init__(self, parent, ip_address, port, local_ip):
        self.parent = parent
        self.ip_address = ip_address
        self.port = port
        # Local IP address to use for communication, in the event that this
        # computer has multiple networks to choose from.
        self.local_ip = local_ip
        # ## Function to call when we get something from the camera.
        # self.callback = None
        self.connection = None
        # Edit this dictionary of common commands after updating the NI RT-ipAddress setup
        # We use a number of 3characters integers to define the commands
        # Starting with 1 and 2 are sending digitals and analogues respectively
        # Starting with 3 are asynchronous commands (mainly abort and reset signals
        # that should operate at any moment.
        # Starting with 4 are synchronous commands that can only operate when the
        # FPGA is idle.
        self.command_dict = {
            "sendDigitals": 100,
            "sendAnalogues": 200,
            "abort": 301,
            "reInit": 302,
            "reInitHost": 303,
            "reInitFPGA": 304,
            "updateNrReps": 405,
            "sendStartStopIndexes": 406,
            "initProfile": 407,
            "triggerExperiment": 408,
            "flushFIFOs": 409,
            "writeDigitals": 410,
            "writeAnalogue": 411,
            "runSequence": 413,
        }
        self.error_codes = {
            "0": None,
            "1": "Could not create socket",
            "2": "Could not create socket connection",
            "3": "Send error",
        }
        self.status = None

    def receiveClient(self, URI):
        pass

    def connect(self, timeout=40):
        self.connection = self.create_send_socket(
            self.ip_address, self.port[0], timeout
        )
        # server = depot.getHandlersOfType(depot.SERVER)[0]
        # Create a status instance to query the FPGA status and run it in a separate thread
        self.status = FPGAStatus(self, self.local_ip, self.port[1])
        self.status.start()

    def get_is_connected(self):
        """Return whether or not our connection is active."""
        return self.connection is not None

    def disconnect(self):
        if self.connection is not None:
            try:
                self.connection.close()
            except Exception as e:
                print(
                    "Couldn't disconnect from %s: %s" % (self.parent.name, e)
                )
            self.connection = None

    def create_send_socket(self, host, port, timeout):
        """Creates a TCP socket meant to send commands to the RT-ipAddress
        Returns the connected socket
        """
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(timeout)
            s.connect((host, port))
        except socket.error as e:
            raise e

        return s

    def write_reply(self):
        """For debugging"""
        pass

    def run_command(self, command, args=(), msg_length=20):
        """This method sends to the RT-ipAddress a Json command message in the following way
        - three numbers representing the command
        - if there are arguments to send:
            - the length of the messages to follow = msg_length
            - the amount of messages to follow
        - receives acknowledgement of reception receiving an error code

        command is a 3 digits string obtained from command_dict

        args is a list of strings containing the arguments associated.

        Return a Dictionary with the error description:
        Error Status, Error code and Error Description
        """
        # Transform args into a list of strings of msg_length chars
        send_args = list()
        for arg in args:
            if type(arg) is float:
                raise Exception("Arguments to send cannot be floats")
            if type(arg) == str and len(arg) <= msg_length:
                send_args.append(arg.rjust(msg_length, "0"))
            elif type(arg) == int and len(str(arg)) <= msg_length:
                send_args.append(str(arg).rjust(msg_length, "0"))
            else:
                send_args.append(str(arg).rjust(msg_length, "0"))

        # Create a dictionary to be flattened and sent as json string
        message_cluster = {
            "Command": command,
            "Message Length": msg_length,
            "Number of Messages": len(send_args),
        }

        try:
            # Send the actual command
            self.connection.send(json.dumps(message_cluster).encode())
            self.connection.send(b"\r\n")
        except socket.error as msg:
            print("Send messageCluster failed.\n", msg)

        try:
            # Send the actual messages buffer
            buf = str("").join(send_args).encode()
            self.connection.sendall(buf)
        except socket.error as msg:
            print("Send buffer failed.\n", msg)

        try:
            # receive confirmation error
            # errorLength = int(self.connection.recv(4).decode())
            # if errorLength:
            try:
                datagram = self.connection.recv(1024)
                error = json.loads(datagram)
                if error["status"]:
                    print(f"There has been an FPGA error: {error}")
            except:
                print("We received a TCP error when confirming command.")
                # errorLength.append(self.connection.recv(4096))
                # datagram = self.connection.recv(4096)

        except socket.error as msg:  # Send failed
            print("Receiving error.\n", msg)
        return

    def write_parameter(self, parameter, value):
        """Writes parameter value to RT-ipAddress"""
        pass

    def wait_for_idle(self):
        """Waits for the Idle status of the FPGA"""
        while self.status.get_status("FPGA Main State") != FPGA_IDLE_STATE:
            time.sleep(0.1)

    def Abort(self):
        """Sends abort experiment command to FPGA"""
        self.run_command(self.command_dict["abort"])

    def re_init(self, unit=None):
        """Restarts the RT-ipAddress and FPGA unless 'ipAddress' or 'fpga' is specified as unit

        Returns nothing
        """
        if not unit:
            self.run_command(self.command_dict["reInit"])

        if unit == "ipAddress":
            self.run_command(self.command_dict["reInitHost"])

        if unit == "fpga":
            self.run_command(self.command_dict["reInitFPGA"])

    def update_n_reps(self, new_count, msg_length=20):
        """Updates the number of repetitions to execute on the FPGA.

        new_count must be msg_length characters or less
        msg_length is an int indicating the length of new_count as a decimal string
        """
        new_count = [new_count]

        self.run_command(self.command_dict["updateNrReps"], new_count, msg_length)

    def send_tables(
        self,
        digitals_table,
        analogue_tables,
        msg_length=20,
        digitals_bit_depth=32,
        analogues_bit_depth=16,
    ):
        """Sends through TCP the digitals and analogue tables to the RT-ipAddress.

        Analogues lists must be ordered form 0 onward and without gaps. That is,
        (0), (0,1), (0,1,2) or (0,1,2,3). If a table is missing a dummy table must be introduced
        msg_length is an int indicating the length of every digital table element as a decimal string
        """
        # Convert the digitals numpy table into a list of messages for the TCP
        digitals_list = list()

        for (
            t,
            value,
        ) in digitals_table:  # TODO: Change this into a more efficient code
            digitals_value = int(
                np.binary_repr(t, 32) + np.binary_repr(value, 32), 2
            )
            digitals_list.append(digitals_value)

        # Send digitals after flushing the FPGA FIFOs
        self.run_command(self.command_dict["flushFIFOs"])
        self.run_command(
            self.command_dict["sendDigitals"], digitals_list, msg_length
        )

        # Send Analogues
        analogue_channel = 0
        for analogue_table in analogue_tables:
            # Convert the analogues numpy table into a list of messages for the TCP
            analogue_list = list()

            for t, value in analogue_table:  # TODO: optimize this
                analogue_value = int(
                    np.binary_repr(t, 32) + np.binary_repr(value, 32), 2
                )
                analogue_list.append(analogue_value)

            command = int(self.command_dict["sendAnalogues"]) + analogue_channel
            self.run_command(command, analogue_list, msg_length)
            analogue_channel = analogue_channel + 1

    def write_indexes(
        self,
        index_set,
        digitals_start_index,
        digitals_stop_index,
        analogues_start_indexes,
        analogues_stop_indexes,
        msg_length=20,
    ):
        """Writes to the FPGA the start and stop indexes of the actionTables that
        have to be run on an experiment. Actually, multiple 'indexSets' can be used
        (up to 16) to be used in combined experiments.

        index_set -- the index_set where the indexes are to be sent to. integer from 0 to 15
        digitals_start_index -- the start point of the digitals table. Included in
        the execution of the experiment. integer up to u32bit
        digitals_stop_index -- the stop point of the digitals table. NOT included in
        the execution of the experiment. integer up to u32bit
        analogues_start_indexes -- iterable containing the start points of the analogues tables.
        Included in the execution or the experiment. list or tuple of integers up to u32bit
        analogues_stop_indexes -- iterable containing the stop points of the analogues tables.
        NOT included in the execution or the experiment. list or tuple of integers up to u32bit
        msg_length is an int indicating the length of every element as a decimal string
        """
        # TODO: Validate the value of indexSet is between 0 and 15
        # TODO: Validate that analogues lists are the same length

        # Merge everything in a single list to send. Note that we interlace the
        # analogue indexes (start, stop, start, stop,...) so in the future we can
        # put an arbitrary number.
        send_list = [index_set, digitals_start_index, digitals_stop_index]

        analogues_interleaved = [
            x
            for t in zip(analogues_start_indexes, analogues_stop_indexes)
            for x in t
        ]

        for index in analogues_interleaved:
            send_list.append(index)

        # send indexes.
        self.run_command(
            self.command_dict["sendStartStopIndexes"], send_list, msg_length
        )

    def PrepareActions(self, actions, num_reps):
        """Sends a actions table to the cRIO and programs the execution of a number of repetitions.
        It does not trigger the execution"""
        # We upload the tables to the cRIO
        self.send_tables(digitals_table=actions[1], analogue_tables=actions[2])

        # Now we can send the Indexes.
        # The indexes will tell the FPGA where the table starts and ends.
        # This allows for more flexibility in the future, as we can store more than
        # one experiment per table and just execute different parts of it.
        # Memory addresses on the FPGA are 0 based. We, however, use 1 based indexing so we
        # can initialize certain values on the 0 address of the FPGA, such as a safe state we can
        # securely rely on.
        digitals_start_index = 1
        digitals_stop_index = len(actions[1])
        analogues_start_indexes = [1 for x in actions[2]]
        analogues_stop_indexes = [len(x) for x in actions[2]]
        self.write_indexes(
            index_set=0,
            digitals_start_index=digitals_start_index,
            digitals_stop_index=digitals_stop_index,
            analogues_start_indexes=analogues_start_indexes,
            analogues_stop_indexes=analogues_stop_indexes,
            msg_length=20,
        )

        # We initialize the profile. That is tell the cRIO how many repetitions to produce and the interval.
        # TODO: Because the generic Executor is adding a last element in the table we put a 0 here. Change this
        self.init_profile(num_reps=num_reps, rep_duration=0)

        return True

    def RunActions(self):
        self.trigger_experiment()

    def read_error(self):
        """Gets error code from RT-ipAddress and FPGA

        Returns a tuple with the error code and the corresponding error message
        """
        return self.status.get_status(["Error code", "Error Description"])

    def is_idle(self):
        """Returns True if experiment is running and False if idle"""
        if self.status.get_status("Action State") == FPGA_IDLE_STATE:
            return True
        else:
            return False

    def is_aborted(self):
        """Returns True if FPGA is aborted (in practice interlocked) and False if idle"""
        if self.status.get_status("Aborted"):
            return True
        else:
            return False

    def flush_fifos(self):
        """Flushes the FIFOs of the FPGA."""
        self.run_command(self.command_dict["flushFIFOs"])

    def ReadPosition(self, line):
        """Returns the current output value of the analog line in native units
        line is an integer corresponding to the requested analog on the FPGA
        as entered in the analog config files.
        """
        line = "Analogue " + str(line)
        return self.status.get_status(line)

    def MoveAbsolute(self, analogue_channel, analogue_value_adu, msg_length=20):
        """Changes an analogue_channel output to the specified analogue_value_adu value

        analogue_value_adu is taken as a raw 16 or 32bit value
        analogue_channel is an integer corresponding to the analogue in the FPGA as specified in the config files
        msg_length is an int indicating the max length of the analogue as a decimal string
        """
        analogue = [analogue_channel, int(analogue_value_adu)]
        self.run_command(self.command_dict["writeAnalogue"], analogue, msg_length)
        while self.ReadPosition(analogue[0]) != analogue[1]:
            time.sleep(0.01)
        return

    def write_analogue_delta(self, analogue_delta_value, analogue_channel):
        """Changes an analogue_channel output to the specified analogue_value delta-value

        analogue_delta_value is taken as a raw 16bit value
        analogue_channel is an integer corresponding to the analogue in the FPGA as specified in the config files
        """
        pass

    def WriteDigital(self, digital_value, msg_length=20):
        """Write a specific value to the ensemble of the digitals through a 32bit
        integer digital_value.
        msg_length is an int indicating the length of the digital_value as a decimal string
        """
        digital_value = [digital_value]
        self.run_command(
            self.command_dict["writeDigitals"], digital_value, msg_length
        )

    def ReadDigital(self, digital_channel=None):
        """Get the value of the current Digitals outputs as a 32bit integer.
        If digital_channel is specified, a 0 or 1 is returned.
        """
        value = np.binary_repr(self.status.get_status("Digitals"))

        if digital_channel is not None:
            return int(value[-digital_channel])
        else:
            return int(value, 2)

    def init_profile(self, num_reps, rep_duration=0, msg_length=20):
        """Prepare the FPGA to run the loaded profile.
        Send a certain number of parameters:
        num_reps and a rep_duration

        num_reps -- the number of repetitions to run
        rep_duration -- the time interval between repetitions
        msg_length -- int indicating the length of num_reps and rep_duration as decimal strings
        """
        self.run_command(
            self.command_dict["initProfile"], [num_reps, rep_duration], msg_length
        )

    def get_frame_data(self):
        """Get the current frame"""
        pass

    def trigger_experiment(self):
        """Trigger the execution of an experiment."""
        self.run_command(self.command_dict["triggerExperiment"])

    def run_sequence_on_fpga(
        self, time_digital_sequence, digitals_bit_depth=32, msg_length=20
    ):
        """Runs a small sequence of digital outputs at determined times"""
        send_list = list()
        for t, d in time_digital_sequence:
            # binarize and concatenate time and digital value
            value = np.binary_repr(t, 32) + np.binary_repr(d, digitals_bit_depth)
            value = int(value, 2)
            send_list.append(value)

        self.run_command(self.command_dict["runSequence"], send_list, msg_length)


class FPGAStatus(threading.Thread):
    def __init__(self, parent, host, port):
        threading.Thread.__init__(self)
        self.parent = parent
        self.host = host
        self.port = port
        # Create a dictionary to store the FPGA status and a lock to access it
        self.current_fpga_status = {}
        self.fpga_status_lock = threading.Lock()

        # Create a socket
        self.socket = None
        self.create_receive_socket()

        # Create a handle to stop the thread
        self.should_run = True

    def create_receive_socket(self):
        """Creates a UDP socket meant to receive status information
        form the RT-ipAddress

        returns the bound socket
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as msg:
            print("Failed to create socket. Error code: ", msg)

        try:
            self.socket.bind((self.host, self.port))
        except socket.error as msg:
            print("Failed to bind address.\n", msg)

    def get_status(self, key=None):
        """Method to call from outside to get the status"""
        if key and self.current_fpga_status is not None:
            try:
                return self.current_fpga_status[key]
            except KeyError as e:
                print(e)
        else:
            with self.fpga_status_lock:
                return self.current_fpga_status

    def get_fpga_status(self):
        """This method polls to a UDP socket and get the status information
        of the RT-ipAddress and FPGA.

        It will update the FPGAStatus dictionary.
        """
        try:
            datagram = self.socket.recvfrom(FPGA_HEARTBEAT_MAX_MSG_LEN)[0]
            status = json.loads(datagram)
        except json.JSONDecodeError as e:
            print("Could not serialize status datagram: ", datagram)
            print(e)

            return None

        # for some reason (see taiga issue #125) the returned datagram decodes as an int and the connection is lost
        if type(status) != dict:
            print(
                f"The returned status for the FPGA is not the expected type: {status}"
            )
            return None

        return status

    def publish_fpga_status_changes(self, new_status):
        """FInd interesting status or status changes in the FPGA and publish them

        return the new_status but with the status reset so not to publish multiple times
        """
        if new_status["Event"] in ["done", "FPGA done"]:
            self.parent.parent.experiment_done()
            new_status["Event"] = ""

        return new_status

    def run(self):
        self.current_fpga_status = self.get_fpga_status()
        update_rate = FPGA_HEARTBEAT_RATE / 2
        retries = 0

        while self.should_run:
            new_fpga_status = self.get_fpga_status()
            if retries > 300:
                # retrying to establish connection
                try:
                    self.create_receive_socket()
                except Exception as e:
                    print(
                        f"The status UDP connection to the Executor is lost after {retries} retries"
                    )
                    raise e

            if new_fpga_status is None:
                retries += 1
                continue
            # with self.fpga_status_lock:
            if (
                new_fpga_status["Event"] != self.current_fpga_status["Event"]
                and new_fpga_status["Event"] == "done"
                and new_fpga_status is not None
            ):
                # Publish any interesting events
                self.current_fpga_status = self.publish_fpga_status_changes(
                    new_status=new_fpga_status
                )
            else:
                self.current_fpga_status = new_fpga_status

            # wait for a period of half the broadcasting rate of the FPGA
            time.sleep(update_rate)
