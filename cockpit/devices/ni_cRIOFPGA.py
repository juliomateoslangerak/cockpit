#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2018 Mick Phillips <mick.phillips@gmail.com>
## Copyright (C) 2018 Julio Mateos Langerak <julio.mateos-langerak@igh.cnrs.fr>
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

import json
from time import sleep
import socket
import time
import numpy as np
from itertools import chain
from functools import reduce

from cockpit import depot, events
import cockpit.gui.toggleButton
import cockpit.handlers.executor
import cockpit.handlers.genericHandler
import cockpit.handlers.genericPositioner
import cockpit.handlers.imager
import cockpit.handlers.lightSource
import cockpit.handlers.stagePositioner
import threading
import cockpit.util.threads
import cockpit.util.connection
from . import executorDevices

from six import iteritems

FPGA_IDLE_STATE = 3
FPGA_ABORTED_STATE = 4
FPGA_UPDATE_RATE = .1  # At which rate is the FPGA sending update status signals
MASTER_IP = '10.6.19.11'


class NIcRIO(executorDevices.ExecutorDevice):
    _config_types = {
        'ipaddress': str,
        'sendport': int,
        'receiveport': int,
    }

    def __init__(self, name, config):
        super(self.__class__, self).__init__(name, config)
        # TODO: tickrate should go into a config?
        self.tickrate = 100  # Number of ticks per ms. As of the resolution of the action table.
        # TODO: check
        # if not self.isActive:
        #     return
        self.sendPort = config.get('sendport')
        self.receivePort = config.get('receiveport')
        self.port = [self.sendPort, self.receivePort]
        self._currentAnalogs = 4*[0]
        # Absolute positions prior to the start of the experiment.
        self._lastAnalogs = 4*[0]
        # Store last movement profile for debugging
        self._lastProfile = None
        self.connection = None

    @cockpit.util.threads.locked
    def initialize(self):
        """Connect to ni's RT-ipAddress computer. Overrides ExecutorDevice's initialize.
        """
        self.connection = Connection(serviceName=self.name, ipAddress=self.ipAddress, port=self.port, localIp=MASTER_IP)
        self.connection.connect()
        self.connection.Abort()

    @cockpit.util.threads.locked
    def finalizeInitialization(self):
        server = depot.getHandlersOfType(depot.SERVER)[0]
        # TODO: Check
        self.receiveUri = server.register(self.receiveData)
        # for line in range(self.nrAnalogLines):
        #     self.setAnalog(line, 65536//2)

    def onPrepareForExperiment(self, *args):
        # super(self.__class__, self).onPrepareForExperiment(*args)
        self._lastAnalogs = [line for line in self._currentAnalogs]
        self._lastDigital = self.connection.ReadDigital()

    # TODO: We can add here all other callbacks
    def receiveData(self, action, *args):
        """Receive data from the RT-ipAddress computer."""
        if action.lower() == 'done':
            events.publish(events.EXECUTOR_DONE % self.name)

    def getAnalog(self, line):
        """Returns the current output value of the analog line in native units
        line is an integer corresponding to the requested analog on the FPGA
        as entered in the analog config files.
        """
        line = 'Analogue ' + str(line)
        return self.connection.status.getStatus(line)

    def setAnalog(self, line, target):    # TODO: integrate this function into the configuration files
        """Set analog position in native units
        :param line: Analog line to change
        :param target: target value
        :return:
        """
        # TODO: verify what is native units
        # adus = int(target * 3276.8)
        ## TODO: sensitivity
        return self.connection.MoveAbsolute(line, target)

    def getHandlers(self):
        """We control which light sources are active, as well as a set of stage motion piezos.
        """
        result = []
        h = cockpit.handlers.executor.AnalogDigitalExecutorHandler(
            self.name, "executor",
            {'examineActions': lambda *args: None,
             'executeTable': self.executeTable,
             'readDigital': self.connection.ReadDigital,
             'writeDigital': self.connection.WriteDigital,
             'getAnalog': self.getAnalog,
             'setAnalog': self.setAnalog,
             },
            dlines=self.nrDigitalLines, alines=self.nrAnalogLines)

        result.append(h)

        # TODO: Check take image
        # The takeImage behaviour is now on the handler. It might be better to
        # have hybrid handlers with multiple inheritance, but that would need
        # an overhaul of how depot determines handler types.
        result.append(cockpit.handlers.imager.ImagerHandler(
            "%s imager" % (self.name), "imager",
            {'takeImage': h.takeImage}))

        self.handlers = set(result)
        return result

    def adaptActions(self, actions):
        """Adapt tha actions table to the cRIO. We have to:
        - convert float in ms to integer clock ticks
        - separate analogue and digital events into different lists
        - generate a structure that describes the profile
        """
        # Profiles
        analogs = [[] for x in range(self.nrAnalogLines)]  # A list of lists (one per channel) of tuples (ticks, (analog values))
        digitals = []  # A list of tuples (ticks, digital state)
        # Need to track time of last analog events
        t_last_analog = None

        for (t, (digital_args, analog_args)) in actions:
            # Convert t to ticks as int while rounding up. The rounding is
            # necessary, otherwise e.g. 10.1 and 10.1999999... both result in 101.
            ticks = int(float(t) * self.tickrate + 0.5)

            # Digital actions - one at every time point.
            if len(digitals) == 0:
                digitals.append((ticks, digital_args))
            elif ticks == digitals[-1][0]: # TODO: verify if we need this for the FPGA
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
            offsets = map(lambda base, new: new - base, self._lastAnalogs, analog_args)
            for offset, aarg, a in zip(offsets, analog_args, analogs):
                if (len(a) == 0) or (len(a) > 0 and offset != a[-1][1]):
                    a.append((ticks, aarg))
                    t_last_analog = t

        # Work around some DSP bugs:
        # * The action table needs at least two events to execute correctly.
        # * Last action must be digital --- if the last analog action is at the same
        #   time or after the last digital action, it will not be performed.
        # Both can be avoided by adding a digital action that does nothing.
        # TODO: test if can remove this
        if len(digitals) == 1 or t_last_analog >= digitals[-1][0]:
            # Just duplicate the last digital action, one tick later.
            digitals.append((digitals[-1][0]+1, digitals[-1][1]))

        # Update records of last positions.
        self._lastDigital = digitals[-1][1]
        self._lastAnalogs = map(lambda x, y: x - (y[-1:][1:] or 0), self._lastAnalogs, analogs)

        # Convert digitals to array of uints.
        digitalsArr = np.array(digitals, dtype=np.uint32).reshape(-1, 2)
        # Convert analogs to array of uints.
        analogsArr = [np.array(a, dtype=np.uint32).reshape(-1, 2) for a in analogs]

        # Create a description dict. Will be byte-packed by server-side code.
        maxticks = reduce(max, chain(list(zip(*digitals))[0],
                                     *[(list(zip(*a)) or [[None]])[0] for a in analogs]))

        description = {}
        description['count'] = maxticks
        description['clock'] = 1000. / float(self.tickrate)
        description['InitDio'] = self._lastDigital
        description['nDigital'] = len(digitals)
        description['nAnalog'] = [len(a) for a in analogs]

        self._lastProfile = (description, digitalsArr, analogsArr)

        return [description, digitalsArr, [*analogsArr]]

    def legacyExecuteTable(self, name, table, startIndex, stopIndex, numReps, repDuration):
        """Actually execute the events in an experiment ActionTable, starting at
        startIndex and proceeding up to but not through stopIndex.
        Convert the desired portion of the table into a "profile" for
        the FPGA.
        """
        # Take time and arguments (i.e. omit handler) from table to generate actions.
        # For NI-cRIO fpga, we also need to:
        #  - convert float in ms to integer clock ticks and ensure digital
        #    lines are not changed twice on the same tick;
        #  - separate analogue and digital events into different lists;
        #  - generate a structure that describes the profile.

        # Start time
        t0 = float(table[startIndex][0])
        # Profiles
        analogs = [[] for x in range(self.nrAnalogLines)]  # A list of lists (one per channel) of tuples (ticks, (analog values))
        digitals = []  # A list of tuples (ticks, digital state)
        # Need to track time of last analog events to workaround a
        # DSP bug later. Also used to detect when events exceed timing
        # resolution
        tLastA = None

        actions = [(float(row[0])-t0,) + tuple(row[2:]) for row in table[startIndex:stopIndex]]
        # If there are repeats, add an extra action to wait until repDuration expired.
        if repDuration is not None:
            repDuration = float(repDuration)
            if actions[-1][0] < repDuration:
                # Repeat the last event at t0 + repDuration
                actions.append((t0+repDuration,) + tuple(actions[-1][1:]))

        # # The DSP executes an analogue movement profile, which is defined using
        # # offsets relative to a baseline at the time the profile was initialized.
        # # These offsets are encoded as unsigned integers, so at profile
        # # intialization, each analogue channel must be at or below the lowest
        # # value it needs to reach in the profile.
        # lowestAnalogs = [min(channel) for channel in zip(*zip(*zip(*actions)[1])[1])]
        # for line, lowest in enumerate(lowestAnalogs):
        #     if lowest < self._lastAnalogs[line]:
        #         self._lastAnalogs[line] = lowest
        #         self.setAnalog(line, lowest)

        for (t, (darg, aargs)) in actions:
            # Convert t to ticks as int while rounding up. The rounding is
            # necessary, otherwise e.g. 10.1 and 10.1999999... both result in 101.
            ticks = int(float(t) * self.tickrate + 0.5)

            # Digital actions - one at every time point.
            if len(digitals) == 0:
                digitals.append((ticks, darg))
            elif ticks == digitals[-1][0]:
                # Used to check for conflicts here, but that's not so trivial.
                # We need to allow several bits to change at the same time point, but
                # they may show up as multiple events in the actionTable. For now, just
                # take the most recent state.
                if darg != digitals[-1][1]:
                    digitals[-1] = (ticks, darg)
                else:
                    pass
            else:
                digitals.append((ticks, darg))

            # Analogue actions - only enter into profile on change.
            # NI-cRIO uses absolute values.
            offsets = map(lambda base, new: new - base, self._lastAnalogs, aargs)
            for offset, aarg, a in zip(offsets, aargs, analogs):
                if (len(a) == 0) or (len(a) > 0 and offset != a[-1][1]):
                    a.append((ticks, aarg))
                    tLastA = t

        # Work around some DSP bugs:
        # * The action table needs at least two events to execute correctly.
        # * Last action must be digital --- if the last analog action is at the same
        #   time or after the last digital action, it will not be performed.
        # Both can be avoided by adding a digital action that does nothing.
        # TODO: test if can remove this
        if len(digitals) == 1 or tLastA >= digitals[-1][0]:
            # Just duplicate the last digital action, one tick later.
            digitals.append((digitals[-1][0]+1, digitals[-1][1]))

        # Update records of last positions.
        self._lastDigital = digitals[-1][1]
        self._lastAnalogs = map(lambda x, y: x - (y[-1:][1:] or 0), self._lastAnalogs, analogs)

        events.publish(events.UPDATE_STATUS_LIGHT, 'device waiting',
                       'Waiting for\nFPGA to finish', (255, 255, 0))
        # Convert digitals to array of uints.
        digitalsArr = np.array(digitals, dtype=np.uint32).reshape(-1, 2)
        # Convert analogs to array of uints.
        analogsArr = [np.array(a, dtype=np.uint32).reshape(-1, 2) for a in analogs]

        # Create a description dict. Will be byte-packed by server-side code.
        maxticks = reduce(max, chain(zip(*digitals)[0],
                                     *[(zip(*a) or [[None]])[0] for a in analogs]))
        description = {}
        description['count'] = maxticks
        description['clock'] = 1000. / float(self.tickrate)
        description['InitDio'] = self._lastDigital
        description['nDigital'] = len(digitals)
        description['nAnalog'] = [len(a) for a in analogs]

        self._lastProfile = (description, digitalsArr, analogsArr)

        self.connection.profileSet(description, digitalsArr, *analogsArr)
        self.connection.DownloadProfile()
        self.connection.InitProfile(numReps)
        events.executeAndWaitFor(events.EXECUTOR_DONE % self.name, self.connection.trigCollect)
        events.publish(events.EXPERIMENT_EXECUTION)

# ====================================================================================
#         profileStr, digitals, analogs = self.generateProfile(table[startIndex:stopIndex], repDuration)
#
#         # Update our positioning values in case we have to make a new profile
#         # in this same experiment. The analog values are a bit tricky, since
#         # they're deltas from the values we used to create the profile.
#         self.lastDigitalVal = digitals[-1, 1]
#         for aline in range(4):
#             self.lastAnalogPositions[aline] = analogs[aline][-1][1] + self.lastAnalogPositions[aline]
#
#         # Apologies for the messiness here; basically we're checking if any
#         # aspect of the experiment profile has changed compared to the last
#         # experiment we ran, if any. If there are differences, then we must
#         # upload the new profile; otherwise we can skip that step.
#         if (self.prevProfileSettings is None or
#                 profileStr != self.prevProfileSettings[0] or
#                 numpy.any(digitals != self.prevProfileSettings[1]) or
#                 sum([numpy.any(analogs[i] != self.prevProfileSettings[2][i]) for i in range(4)])):
#             # We can't just re-use the already-loaded profile.
#             self.connection.sendTables(digitalsTable = digitals, analogueTables = analogs)
#             self.prevProfileSettings = (profileStr, digitals, analogs)
#
#             # Now we can send the Indexes.
#             # The indexes will tell the FPGA where the table starts and ends.
#
#             # This allows for more flexibility in the future, as we can store more than
#
#             # one experiment per table and just execute different parts of it.
#             analoguesStartIndexes = [1 for x in analogs]
#             analoguesStopIndexes = [len(x) for x in analogs]
#             self.connection.writeIndexes(indexSet=0,
#
#                                          digitalsStartIndex=1,
#
#                                          digitalsStopIndex=len(digitals),
#
#                                          analoguesStartIndexes=analoguesStartIndexes,
#                                          analoguesStopIndexes=analoguesStopIndexes,
#                                          msgLength=20)
#
#         events.publish('update status light', 'device waiting',
#                 'Waiting for\nFPGA to finish', (255, 255, 0))
#         # InitProfile will declare the current analog positions as a "basis"
#         # and do all actions as offsets from those bases, so we need to
#         # ensure that the variable retarder is zeroed out first.
#         # TODO: verify if this is true for the FPGA
#         retarderLine = self.handlerToAnalogLine[self.retarderHandler]
#         self.setAnalog(retarderLine, 0)
#
#         self.connection.initProfile(numReps, repDuration)
#         events.executeAndWaitFor("DSP done", self.connection.triggerExperiment)
#
#         events.publish('experiment execution')
#         return
#
    # def performSubscriptions(self):
    #     """
    #     We care when cameras are enabled, since we control some of them
    #
    #     via external trigger. There are also some light sources that we don't
    #     control directly that we need to know about.
    #     """
    #     events.subscribe('camera enable', self.toggleCamera)
    #     events.subscribe('light source enable', self.toggleLightHandler)
    #     events.subscribe('user abort', self.onAbort)
    #     events.subscribe('prepare for experiment', self.onPrepareForExperiment)
    #     events.subscribe('cleanup after experiment', self.cleanupAfterExperiment)
    #
    # def makeInitialPublications(self):
    #     """
    #     As a side-effect of setting our initial positions, we will also
    #     publish them. We want the Z piezo to be in the middle of its range
    #     of motion.
    #     """
    #     self.moveRetarderAbsolute(None, 0)
    #
    # def onAbort(self):
    #     """
    #     User clicked the abort button.
    #     """
    #     self.connection.abort()
    #     events.publish("DSP done") # TODO: change this to a FPGA-done
    #
    # def toggleLight(self, lightName, isEnabled):
    #     """
    #     Enable/disable a specific light source
    #     """
    #     if isEnabled:
    #         self.activeLights.add(lightName)
    #     elif lightName in self.activeLights:
    #         self.activeLights.remove(lightName)
    #
    # def triggerNow(self, line, dt=0.01):
    #     self.connection.writeDigitals(self.connection.readDigitals() ^ line)
    #     time.sleep(dt)
    #     self.connection.writeDigitals(self.connection.readDigitals() ^ line)
    #
    # def toggleLightHandler(self, handler, isEnabled):
    #     """
    #     As toggleLight, but accepts a handler instead.
    #     """
    #     self.toggleLight(handler.name, isEnabled)
    #
    # def setExposureTime(self, name, value):
    #     """
    #     Update the exposure time for a specific light source.
    #     """
    #     self.lightToExposureTime[name] = value
    #
    # def getExposureTime(self, name):
    #     """
    #     Retrieve the exposure time for a specific light source.
    #     """
    #     return self.lightToExposureTime[name]
    #
    # def toggleCamera(self, camera, isEnabled):
    #     """
    #     Enable/disable a specific camera.
    #     """
    #     if not isEnabled and camera.name in self.activeCameras:
    #         self.activeCameras.remove(camera.name)
    #     else:
    #         self.activeCameras.add(camera.name)
    #
    # def publishPiezoPosition(self, axis):
    #     """
    #     Report the new position of a piezo.
    #     """
    #     events.publish('stage mover', '%d piezo' % axis,
    #
    #             axis, self.curPosition[axis])
    #
    # def movePiezoAbsolute(self, axis, pos):
    #     """
    #     Move a stage piezo to a given position.
    #     """
    #     self.curPosition.update({axis: pos})
    #     # Convert from microns to ADUs.
    #     aline = self.axisMapper[axis]
    #     aduPos = self.convertMicronsToADUs(aline, pos)
    #     # TODO: sensitivity
    #     self.connection.writeAnalogueADU(aline, aduPos)
    #     self.publishPiezoPosition(axis)
    #     # Assume piezo movements are instantaneous;
    #
    #     # TODO: we could establish a verification through the FPGA eg: connection.MoveAbsolute does not return until done
    #     events.publish('stage stopped', '%d piezo' % axis)
    #
    # def movePiezoRelative(self, axis, delta):
    #     """
    #     Move the stage piezo by a given delta
    #     """
    #     self.movePiezoAbsolute(axis, self.curPosition[axis] + delta)
    #
    # def getPiezoPos(self, axis):
    #     """
    #     Get the current Piezo position.
    #     """
    #     return self.curPosition[axis]
    #
    # def getPiezoMovementTime(self, axis, start, end):
    #     """
    #     Get the amount of time it would take the piezo to move from the
    #
    #     initial position to the final position, as well
    #     as the amount of time needed to stabilize after that point,
    #
    #     both in milliseconds. These numbers are both somewhat arbitrary;
    #     we just say it takes 1ms per micron to stabilize and .1ms to move.
    #     """
    #     distance = abs(start - end)
    #     return (decimal.Decimal('.1'), decimal.Decimal(distance * 1))
    #
    # def setSLMPattern(self, name, position):
    #     """
    #     Set the SLM's position to a specific value.
    #
    #     For now, do nothing; the only way we can change the SLM position is by
    #
    #     sending triggers so we have no absolute positioning.
    #     """
    #     pass
    #
    # def moveSLMPatternBy(self, name, delta):
    #     """
    #     Adjust the SLM's position by the specified offset. Again, do nothing.
    #     """
    #     pass
    #
    # def getCurSLMPattern(self, name):
    #     """
    #     Get the current SLM position, either angle or phase depending on the
    #
    #     caller. We have no idea, really.
    #     """
    #     return 0
    #
    # def getSLMStabilizationTime(self, name, prevPos, curPos):
    #     """
    #     Get the time to move to a new SLM position, and the stabilization time,
    #
    #     in milliseconds. Note we assume that this requires only one triggering
    #     of the SLM.
    #     """
    #     return (1, 30)
    #
    # def moveRetarderAbsolute(self, name, pos):
    #     """
    #     Move the variable retarder to the specified voltage.
    #     """
    #     self.curRetarderVoltage = pos
    #     handler = depot.getHandlerWithName('SI angle')
    #     aline = self.handlerToAnalogLine[handler]
    #     # Convert from volts to ADUs.
    #     # TODO: add volts to ADUs in config files
    #     self.connection.writeAnalogueADU(aline, int(pos * 3276.8))
    #
    # def moveRetarderRelative(self, name, delta):
    #     """
    #     Move the variable retarder by the specified voltage offset.
    #     """
    #     self.moveRetarderAbsolute(self.curRetarderVoltage + delta)
    #
    # def getRetarderPos(self, name):
    #     """
    #     Get the current variable retarder voltage
    #     """
    #     return self.curRetarderVoltage
    #
    # def getRetarderMovementTime(self, name, start, end):
    #     """
    #     Get the time needed for the variable retarder to move to a new value.
    #     """
    #     return (1, 1000)

    @cockpit.util.threads.locked
    def takeImage(self):
        """Take an image with the current light sources and active cameras.
        """
        cameraMask = 0
        lightTimePairs = []
        maxTime = 0
        cameraReadTime = 0
        for handler, line in iteritems(self.handlerToDigitalLine):
            if handler.name in self.activeLights:
                maxTime = max(maxTime, handler.getExposureTime())
                exposureTime = handler.getExposureTime()
                lightTimePairs.append((line, exposureTime))
                maxTime = max(maxTime, exposureTime)
        for name, line in iteritems(self.nameToDigitalLine):
            if name in self.activeCameras:
                cameraMask += line
                handler = depot.getHandlerWithName(name)
                handler.setExposureTime(maxTime)
                cameraReadTime = max(cameraReadTime, handler.getTimeBetweenExposures())
        self.connection.takeImage(cameraMask, lightTimePairs, cameraReadTime)

    @cockpit.util.threads.locked
    def takeBurst(self, frameCount = 10):
        """
        Use the internal triggering of the camera to take a burst of images

        Experimental
        """
        cameraMask = 0
        lightTimePairs = []
        maxTime = 0
        for handler, line in iteritems(self.handlerToDigitalLine):
            if handler.name in self.activeLights:
                maxTime = max(maxTime, handler.getExposureTime())
                exposureTime = handler.getExposureTime()
                lightTimePairs.append((line, exposureTime))
                maxTime = max(maxTime, exposureTime)
        for name, line in iteritems(self.nameToDigitalLine):
            if name in self.activeCameras:
                cameraMask += line
                handler = depot.getHandlerWithName(name)
                handler.setExposureTime(maxTime)

        sleep(5)
    #
    # def cleanupAfterExperiment(self, *args):
    #     """
    #     Cleanup after an experiment completes: restore our cached position.
    #     """
    #     for axis, position in iteritems(self.preExperimentPosition):
    #         self.movePiezoAbsolute(axis, position)
    #
    # def getNumRunnableLines(self, name, table, index):
    #     """
    #     Get the number of actions from the provided table that we are
    #     capable of executing.
    #     """
    #     # TODO: replace this method by a more sophisticated setup as the FPGA may
    #     # control repetitions and the duration
    #     count = 0
    #     for time, handler, parameter in table[index:]:
    #         # Check for analog and digital devices we control.
    #         if (handler not in self.handlers and
    #
    #                 handler.name not in self.nameToDigitalLine):
    #             # Found a device we don't control.
    #             break
    #         count += 1
    #     return count
    #
    # def cleanupPiezo(self, axis, isCleanupFinal):
    #     """
    #     Clean up after the experiment is done
    #     """
    #     if isCleanupFinal:
    #         # The DSP may complain about still being in collection mode
    #         # even though it's told us it's done; wait a bit.
    #         time.sleep(.25)
    #         position = self.connection.getAnalog(self.axisMapper[axis])
    #         self.curPosition.update({axis: position})
    #         self.publishPiezoPosition(axis)
    #         # Manually force all digital lines to 0, because for some reason the
    #         # DSP isn't doing this on its own, even though our experiments end
    #         # with an all-zeros entry.
    #         self.connection.writeDigitals(0)
    #         # Likewise, force the retarder back to 0.
    #         retarderLine = self.handlerToAnalogLine[self.retarderHandler]
    #         self.setAnalog(retarderLine, 0)
    #
    # def generateProfile(self, events, repDuration):
    #     """
    #     Given a list of (time, handle, action) tuples, generate several Numpy
    #     arrays: one of digital actions, and one each for each analog output.
    #     We also generate the "profile string" that is used to describe these
    #     arrays.
    #     """
    #     # Maps digital lines to the most recent setting for that line.
    #     digitalToLastVal = {}
    #     # Maps analog lines to lists of (time, value) pairs.
    #
    #     analogToPosition = {}
    #
    #     # Expand out the timepoints so we can use integers to refer to
    #
    #     # sub-millisecond events, since the DSP table doesn't use
    #     # floating point.
    #     # Convert from decimal.Decimal instances to floating point.
    #     times = [float(e[0] * self.actionsPerMillisecond) for e in events]
    #     # Now convert to int while rounding. The rounding is necessary,
    #     # otherwise e.g. 10.1 and 10.1999999... both result in 101.
    #     times = [int(t + .5) for t in times]
    #     times = sorted(list(set(times)))
    #     baseTime = times[0]
    #
    #     # Take into account the desired rep duration
    #
    #     # If the desired repDuration is shorter than the time the
    #
    #     # experimental time, print a warning
    #     if repDuration is not None:
    #         repDuration *= self.actionsPerMillisecond
    #         waitTime = repDuration - (times[-1] - baseTime)
    #         if waitTime < 0:
    #             print('WARNING!! The desired experiment timing does NOT')
    #             print('fit into the requested repetition duration.')
    #             print('OMX-T will acquire as fast as possible')
    #     # HACK: ensure that there's at least 2 timesteps in the experiment,
    #     # or else it won't run properly.
    #     havePaddedDigitals = False
    #     if len(times) == 1:
    #         times.append(times[0] + 1)
    #         havePaddedDigitals = True
    #
    #     digitals = numpy.zeros((len(times), 2), dtype = numpy.uint32)
    #     digitals[:, 0] = times
    #     # Rebase the times so that they start from 0.
    #     digitals[:, 0] -= baseTime
    #
    #     # Construct lists of (time, value) pairs for the DSP's digital and
    #     # analog outputs.
    #     curDigitalValue = self.lastDigitalVal
    #     alineToAnalogs = {}
    #     for time, handler, action in events:
    #         # Do the same "Decimal -> float -> rounded int" conversion
    #         time = int(float(time * self.actionsPerMillisecond) + .5)
    #         index = times.index(time)
    #         # Ensure a valid (nonzero) digital value exists regardless of the
    #         # type of action, e.g. so analog actions don't zero the digital
    #         # output.
    #         digitals[index, 1] = curDigitalValue
    #         if handler in self.handlerToDigitalLine:
    #             # Update curDigitalValue according to the value of the output
    #             # line for this handler. Digital actions are either on or off,
    #             # and they stay that way until told otherwise.
    #             line = self.handlerToDigitalLine[handler]
    #             if line not in digitalToLastVal or digitalToLastVal[line] != action:
    #                 # Line has changed
    #                 addend = line
    #                 if not action:
    #                     addend = -line
    #                 if curDigitalValue + addend < 0:
    #                     # This should never happen.
    #                     raise RuntimeError("Negative current digital value from adding %s to %s" % (bin(addend), bin(curDigitalValue)))
    #                 curDigitalValue += addend
    #                 digitals[index, 1] = curDigitalValue
    #             digitalToLastVal[line] = action
    #         elif handler in self.handlerToAnalogLine:
    #             # Analog lines step to the next position.
    #
    #             # HACK: the variable retarder shows up here too, and for it
    #             # we set specific voltage values depending on position.
    #             aline = self.handlerToAnalogLine[handler]
    #             value = 0
    #             if handler is self.retarderHandler:
    #                 value = int(self.retarderVoltages[action] * 3276.8)
    #             else:
    #                 value = self.convertMicronsToADUs(aline, action)
    #                 # Add the start position as the experiment is generating deltas
    #                 value += self.convertMicronsToADUs(aline, self.lastAnalogPositions[aline])
    #             # If we're in the
    #             # middle of an experiment, then these values need to be
    #             # re-baselined based on where we started from, since when the
    #             # DSP treats all analog positions as offsets of where it was
    #             # when it started executing the profile.
    #             ## not needed for the FPGA
    #             if aline not in alineToAnalogs:
    #                 alineToAnalogs[aline] = []
    #             alineToAnalogs[aline].append((time - baseTime, value))
    #         else:
    #             raise RuntimeError("Unhandled handler when generating FPGA profile: %s" % handler.name)
    #
    #     if havePaddedDigitals:
    #         # We created a dummy digitals entry since there was only one
    #         # timepoint, but that dummy entry has an output value of 0 instead
    #         # of whatever the current output is, so replace it.
    #         digitals[-1, 1] = curDigitalValue
    #
    #     # Convert the analog actions into Numpy arrays now that we know their
    #     # lengths. Default to [0, 0], fill in a proper array for any axis where
    #     # we actually do something.
    #     analogs = [numpy.zeros((1, 2), dtype = numpy.uint32) for i in range(4)]
    #     for aline, actions in iteritems(alineToAnalogs):
    #         analogs[aline] = numpy.zeros((len(actions), 2), dtype = numpy.uint32)
    #         for i, (time, value) in enumerate(actions):
    #             analogs[aline][i] = (time, value)
    #
    #     # Generate the string that describes the profile we've created.
    #     description = numpy.rec.array(None,
    #             formats = "u4, f4, u4, u4, 4u4",
    #             names = ('count', 'clock', 'InitDio', 'nDigital', 'nAnalog'),
    #             aligned = True, shape = 1)
    #
    #     runtime = max(digitals[:, 0])
    #     for aline in range(4):
    #         runtime = max(runtime, max(analogs[aline][:, 0]))
    #     clock = 1000 / float(self.actionsPerMillisecond)
    #     description[0]['count'] = runtime
    #     description[0]['clock'] = clock
    #     description[0]['InitDio'] = self.lastDigitalVal
    #     description[0]['nDigital'] = len(digitals)
    #     description['nAnalog'] = [len(a) for a in analogs]
    #
    #     return description.tostring(), digitals, analogs
    #
    # def convertMicronsToADUs(self, aline, position):
    #     """Given a target position for the specified axis, generate an
    #
    #     appropriate value for the NI-FPGA's analog system.
    #     """
    #     return long(position / self.alineToUnitsPerADU[aline])

    def setDigital(self, value):
        """Debugging function: set the digital output for the NI-FPGA."""
        self.connection.WriteDigital(value)

    # def plotProfile(self):
    #     """
    #     Debugging function: plot the NI-FPGA profile we last used.
    #     """
    #     if not self.prevProfileSettings:
    #         return
    #     digitals = self.prevProfileSettings[1]
    #     analogs = self.prevProfileSettings[2]
    #
    #     # Determine the X (time) axis
    #     start = min([a[0][0] for a in analogs])
    #     start = min(start, digitals[0,0])
    #     end = max([a[-1][0] for a in analogs])
    #     end = max(end, digitals[-1, 0])
    #
    #     # Determine the Y (voltage) axis. Voltage is arbitrary for digital
    #     # values -- they're either on or off, but we want analogs to use the
    #     # full viewing area.
    #     minVoltage = None
    #     maxVoltage = None
    #     for axis, analog in enumerate(analogs):
    #         for time, val in analog:
    #             #TODO: integrate this conversion into the config files
    #             converted = val / 3276.80 # Convert ADUs -> volts
    #             if minVoltage is None or minVoltage > converted:
    #                 minVoltage = converted
    #             if maxVoltage is None or maxVoltage < converted:
    #                 maxVoltage = converted
    #     # Ensure some vaguely sane values
    #     if minVoltage is None:
    #         minVoltage = 0
    #     if maxVoltage is None or maxVoltage == minVoltage:
    #         maxVoltage = minVoltage + 1
    #
    #     figure = matplotlib.figure.Figure((6, 4),
    #             dpi = 100, facecolor = (1, 1, 1))
    #     axes = figure.add_subplot(1, 1, 1)
    #     axes.set_axis_bgcolor('white')
    #     axes.set_title('NI-FPGA profile plot')
    #     axes.set_ylabel('Volts')
    #     axes.set_xlabel('Time (tenths of ms)')
    #     axes.xaxis.set_major_locator(matplotlib.ticker.MaxNLocator(25))
    #
    #     lines = []
    #     labels = []
    #     colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k']
    #     colorIndex = 0
    #     for aline, analog in enumerate(analogs):
    #         if numpy.any(analog) != 0:
    #             xVals = [a[0] for a in analog]
    #             # TODO: integrate this conversion into the config files
    #             yVals = [a[1] / 3276.80 for a in analog]
    #             lines.append(axes.plot(xVals, yVals, colors[colorIndex]))
    #             colorIndex += 1
    #             name = 'Line %d' % aline
    #             labels.append(name)
    #
    #     # Currently-active handlers at this point
    #     activeNames = set()
    #     # Maps handler names to lists of (time, isActive) pairs
    #     nameToVals = {}
    #     for time, pattern in digitals:
    #         for handler, line in iteritems(self.handlerToDigitalLine):
    #             matches = line & pattern
    #             if matches and handler.name not in activeNames:
    #                 # We trigger this handler here
    #                 activeNames.add(handler.name)
    #                 if handler.name not in nameToVals:
    #                     # Everyone starts at 0.
    #                     nameToVals[handler.name] = [(start, 0)]
    #                 nameToVals[handler.name].append((time - .00001, 0))
    #                 nameToVals[handler.name].append((time, 1))
    #             elif not matches and handler.name in activeNames:
    #                 # We deactivate this handler here.
    #                 activeNames.remove(handler.name)
    #                 nameToVals[handler.name].append((time, 1))
    #                 nameToVals[handler.name].append((time + .00001, 0))
    #
    #     for i, name in enumerate(sorted(nameToVals.keys())):
    #         scale = float(i + 1) / len(nameToVals.keys()) / 2
    #         xVals = []
    #         yVals = []
    #         for pair in nameToVals[name]:
    #             xVals.append(pair[0])
    #             scaledVal = minVoltage + pair[1] * scale * (maxVoltage - minVoltage)
    #             yVals.append(scaledVal)
    #         color = colors[colorIndex % len(colors)]
    #         colorIndex += 1
    #         lines.append(axes.plot(xVals, yVals, color))
    #         labels.append(name)
    #
    #     figure.legend(lines, labels, loc = 'upper left')
    #     frame = wx.Frame(None, title = 'NI-FPGA Profile Plot')
    #     canvas = matplotlib.backends.backend_wxagg.FigureCanvasWxAgg(
    #             frame, -1, figure)
    #     canvas.draw()
    #     frame.Show()
    #
    # def advanceSLM(self, count = 1):
    #     """
    #     Debugging function: advance the SLM.
    #     """
    #     handler = depot.getHandlerWithName('SI SLM')
    #     line = self.handlerToDigitalLine[handler]
    #     for i in range(count):
    #         self.setDigital(line)
    #         self.setDigital(0)
    #         time.sleep(.1)
    #
    # def runProfile(self, digitals, analogs, numReps = 1, baseDigital = 0):
    #     """
    #     load and execute a profile.
    #     """
    #     description = numpy.rec.array(None,
    #             formats = "u4, f4, u4, u4, 4u4",
    #             names = ('count', 'clock', 'InitDio', 'nDigital', 'nAnalog'),
    #             aligned = True, shape = 1)
    #     # Only doing the max of the digitals or the Z analog piezo.
    #     runtime = max(max(digitals[:,0]), max(analogs[1][:,0]))
    #     clock = 1000 / float(self.actionsPerMillisecond)
    #     description[0]['count'] = runtime
    #     description[0]['clock'] = clock
    #     description[0]['InitDio'] = baseDigital
    #     description[0]['nDigital'] = len(digitals)
    #     description['nAnalog'] = [len(a) for a in analogs]
    #     profileStr = description.tostring()
    #
    #     self.connection.profileSet(profileStr, digitals, *analogs)
    #     self.connection.DownloadProfile()
    #     # InitProfile will declare the current analog positions as a "basis"
    #     # and do all actions as offsets from those bases, so we need to
    #     # ensure that the variable retarder is zeroed out first.
    #     retarderLine = self.axisMapper[self.handlerToAnalogAxis[self.retarderHandler]]
    #     self.setAnalog(retarderLine, 0)
    #
    #     self.connection.initProfile(numReps)
    #     events.executeAndWaitFor("NI-FPGA done", self.connection.triggerExperiment)

class Connection:
    """This class handles the connection with NI's RT-ipAddress computer."""
    def __init__(self, serviceName, ipAddress, port, localIp):
        self.serviceName = serviceName
        self.ipAddress = ipAddress
        self.port = port
        # Local IP address to use for communication, in the event that this
        # computer has multiple networks to choose from.
        self.localIp = localIp
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
        self.commandDict = {'sendDigitals': 100,
                            'sendAnalogues': 200,
                            'abort': 301,
                            'reInit': 302,
                            'reInitHost': 303,
                            'reInitFPGA': 304,
                            'updateNrReps': 405,
                            'sendStartStopIndexes': 406,
                            'initProfile': 407,
                            'triggerExperiment': 408,
                            'flushFIFOs': 409,
                            'writeDigitals': 410,
                            'writeAnalogue': 411,
                            'takeImage': 413,
                            }
        self.errorCodes = {'0': None,  # TODO: verify this
                           '1': 'Could not create socket',
                           '2': 'Could not create socket connection',
                           '3': 'Send error'}
        self.status = None

    def connect(self, timeout=40):
        self.connection = self.createSendSocket(self.ipAddress, self.port[0])
        # Set  atimeout for the socket
        self.connection.settimeout(timeout)
        # server = depot.getHandlersOfType(depot.SERVER)[0]
        ## Create a status instance to query the FPGA status and run it in a separate thread
        self.status = FPGAStatus(self, self.localIp, self.port[1])
        self.status.start()
        #        self.fn = fn
        #        self.startCollectThread()
        #        self.reInit()
        #        self.clientConnection = None
        #        self.MoveAbsolute(0, 10)
        #        self.WriteShutter(255)

    ## Return whether or not our connection is active.
    def getIsConnected(self):
        return self.connection is not None

    def disconnect(self):
        if self.connection is not None:
            server = depot.getHandlersOfType(depot.SERVER)[0]
            server.unregister(self.callback)
            try:
                self.connection.close()
            except Exception as e:
                print ("Couldn't disconnect from %s: %s" % (self.serviceName, e))
            self.connection = None

    def createSendSocket(self, host, port):
        """Creates a TCP socket meant to send commands to the RT-ipAddress
        Returns the connected socket
        """
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        except socket.error as msg:
            print ('Failed to create socket.\n', msg)
            return 1, '1'

        try:
            s.connect((host , port))
        except socket.error as msg:
            print ('Failed to establish connection.\n', msg)
            return 1, '2'

        return s

    def writeReply(self):
        """For debugging"""
        pass

    def runCommand(self, command, args=[], msgLength=20):
        """This method sends to the RT-ipAddress a Json command message in the following way
        - three numbers representing the command
        - if there are arguments to send:
            - the length of the messages to follow = msglength
            - the amount of messages to follow
        - receives acknowledgement of reception receiving an error code

        command is a 3 digits string obtained from commandDict

        args is a list of strings containing the arguments associated.

        Return a Dictionary with the error description:
        Error Status, Error code and Error Description
        """
        # Transform args into a list of strings of msgLength chars
        error = False
        sendArgs = []
        for arg in args:
            if type(arg) == str and len(arg) <= msgLength:
                sendArgs.append(arg.rjust(msgLength, '0'))
            elif type(arg) == int and len(str(arg)) <= msgLength:
                sendArgs.append(str(arg).rjust(msgLength, '0'))
            else:
                try:
                    sendArgs.append(str(arg).rjust(msgLength, '0'))
                except:
                    print('Cannot send arguments to the executing device')

        # Create a dictionary to be flattened and sent as json string
        messageCluster = {'Command': command,
                          'Message Length': msgLength,
                          'Number of Messages': len(sendArgs)
                          }
#         # if the command is asynchronous, wait until FPGA is idle
#         if command // 100 != 3:
#             self.waitForIdle()
#

        try:
            ## Send the actual command
            self.connection.send(json.dumps(messageCluster).encode())
            self.connection.send(b'\r\n')
        except socket.error as msg:
            print('Send messageCluster failed.\n', msg)

        try:
            ## Send the actual messages buffer
            buf = str('').join(sendArgs).encode()
            self.connection.sendall(buf)
        except socket.error as msg:
            print('Send buffer failed.\n', msg)

        try:
            ## receive confirmation error
            errorLength = self.connection.recv(4)
            try:
                error = self.connection.recv(int(errorLength))
            except ValueError:
                errorLength.append(self.connection.recv(4096))
                error = errorLength
            if error == b'0':
                error = False
        except socket.error as msg:
            #Send failed
            print('Receiving error.\n', msg)
        finally:
            if error: print(f'This is the error: {error}')

        return


    def writeParameter(self, parameter, value):
        """Writes parameter value to RT-ipAddress
        """
        pass

    def waitForIdle(self):
        """Waits for the Idle status of the FPGA"""
        while self.status.getStatus('FPGA Main State') != FPGA_IDLE_STATE:
            time.sleep(0.1)

    def Abort(self):
        """Sends abort experiment command to FPGA
        """
        self.runCommand(self.commandDict['abort'])

    def reInit(self, unit = None):
        """Restarts the RT-ipAddress and FPGA unless 'ipAddress' or 'fpga' is specified as unit

        Returns nothing
        """
        if not unit:
            self.runCommand(self.commandDict['reInit'])

        if unit == 'ipAddress':
            self.runCommand(self.commandDict['reInitHost'])

        if unit == 'fpga':
            self.runCommand(self.commandDict['reInitFPGA'])

    def updateNReps(self, newCount, msgLength=20):
        """Updates the number of repetitions to execute on the FPGA.

        newCount must be msgLength characters or less
        msgLength is an int indicating the length of newCount as a decimal string
        """
        newCount = [newCount]

        self.runCommand(self.commandDict['updateNrReps'], newCount, msgLength)

    def sendTables(self, digitalsTable, analogueTables, msgLength = 20, digitalsBitDepth = 32, analoguesBitDepth = 16):
        """Sends through TCP the digitals and analogue tables to the RT-ipAddress.

        Analogues lists must be ordered form 0 onward and without gaps. That is,
        (0), (0,1), (0,1,2) or (0,1,2,3). If a table is missing a dummy table must be introduced
        msgLength is an int indicating the length of every digital table element as a decimal string
        """

        # Convert the digitals numpy table into a list of messages for the TCP
        digitalsList = []

        for t, value in digitalsTable:
            digitalsValue = int(np.binary_repr(t, 32) + np.binary_repr(value, 32), 2)
            digitalsList.append(digitalsValue)

        # Send digitals after flushing the FPGA FIFOs
        self.runCommand(self.commandDict['flushFIFOs'])
        self.runCommand(self.commandDict['sendDigitals'], digitalsList, msgLength)

        # Send Analogues
        analogueChannel = 0
        for analogueTable in analogueTables:

            # Convert the analogues numpy table into a list of messages for the TCP
            analogueList = []

            for t, value in analogueTable:
                analogueValue = int(np.binary_repr(t, 32) + np.binary_repr(value, 32), 2)
                analogueList.append(analogueValue)

            command = int(self.commandDict['sendAnalogues']) + analogueChannel
            self.runCommand(command, analogueList, msgLength)
            analogueChannel = analogueChannel + 1

    def writeIndexes(self, indexSet, digitalsStartIndex, digitalsStopIndex, analoguesStartIndexes, analoguesStopIndexes,
                     msgLength=20):
        """Writes to the FPGA the start and stop indexes of the actionTables that
        have to be run on an experiment. Actually, multiple 'indexSets' can be used
        (up to 16) to be used in combined experiments.

        indexSet -- the indexSet where the indexes are to be sent to. integer from 0 to 15
        digitalsStartIndex -- the start point of the digitals table. Included in
        the execution of the experiment. integer up to u32bit
        digitalsStopIndex -- the stop point of the digitals table. NOT included in
        the execution of the experiment. integer up to u32bit
        analoguesStartIndexes -- iterable containing the start points of the analogues tables.
        Included in the execution or the experiment. list or tuple of integers up to u32bit
        analoguesStopIndexes -- iterable containing the stop points of the analogues tables.
        NOT included in the execution or the experiment. list or tuple of integers up to u32bit
        msgLength is an int indicating the length of every element as a decimal string
        """
        # TODO: Verify the value of indexSet is between 0 and 15
        # TODO: Verify that analogues lists are the same length

        # Merge everything in a single list to send. Note that we interlace the

        # analogue indexes (start, stop, start, stop,...) so in the future we can

        # put an arbitrary number.
        sendList = [indexSet, digitalsStartIndex, digitalsStopIndex]

        analoguesInterleaved = [x for t in zip(analoguesStartIndexes, analoguesStopIndexes) for x in t]

        for index in analoguesInterleaved:
            sendList.append(index)

        # send indexes.

        self.runCommand(self.commandDict['sendStartStopIndexes'], sendList, msgLength)

    def PrepareActions(self, actions, numReps):
        """Sends a actions table to the cRIO and programs the execution of a number of repetitions.
        It does not trigger the execution"""
        # We upload the tables to the cRIO
        self.sendTables(digitalsTable=actions[1], analogueTables=actions[2])

        # Now we can send the Indexes.
        # The indexes will tell the FPGA where the table starts and ends.
        # This allows for more flexibility in the future, as we can store more than
        # one experiment per table and just execute different parts of it.
        analoguesStartIndexes = [1 for x in actions[2]]
        analoguesStopIndexes = [len(x) for x in actions[2]]
        self.writeIndexes(indexSet=0,
                          digitalsStartIndex=1,
                          digitalsStopIndex=len(actions[1]),
                          analoguesStartIndexes=analoguesStartIndexes,
                          analoguesStopIndexes=analoguesStopIndexes,
                          msgLength=20)

        # We initialize the profile. That is tell the cRIO how many repetitions to produce and the
        # interval.
        # TODO: Because the generic Executor is adding a last element in the table we put a 0 here. We have to change this
        self.initProfile(numReps=numReps, repDuration=0)

        return True

    def RunActions(self):
        self.triggerExperiment()

    def readError(self):
        """Gets error code from RT-ipAddress and FPGA

        Returns a tuple with the error code and the corresponding error message
        """
        return self.status.getStatus(['Error code', 'Error Description'])

    def isIdle(self):
        """Returns True if experiment is running and False if idle
        """
        if self.status.getStatus('Action State') == FPGA_IDLE_STATE:
            return True
        else:
            return False

    def isAborted(self):
        """Returns True if FPGA is aborted (in practice interlocked) and False if idle
        """
        if self.status.getStatus('Aborted'):
            return True
        else:
            return False

    def flushFIFOs(self):
        """Flushes the FIFOs of the FPGA.
        """
        self.runCommand(self.commandDict['flushFIFOs'])

    def MoveAbsolute(self, analogueChannel, analogueValueADU, msgLength=20):
        """Changes an analogueChannel output to the specified analogueValue value

        analogueValue is taken as a raw 16 or 32bit value
        analogueChannel is an integer corresponding to the analogue in the FPGA as specified in the config files
        msgLength is an int indicating the max length of the analogue as a decimal string
        """
        analogue = [analogueChannel, analogueValueADU]
        self.runCommand(self.commandDict['writeAnalogue'], analogue, msgLength)

    def writeAnalogueDelta(self, analogueDeltaValue, analogueChannel):
        """Changes an analogueChannel output to the specified analogueValue delta-value

        analogueDeltaValue is taken as a raw 16bit value
        analogueChannel is an integer corresponding to the analogue in the FPGA as specified in the config files
        """
        pass

    def WriteDigital(self, digitalValue, msgLength=20):
        """Write a specific value to the ensemble of the digitals through a 32bit
        integer digitalValue.
        msgLength is an int indicating the length of the digitalValue as a decimal string
        """
        digitalValue = [digitalValue]
        self.runCommand(self.commandDict['writeDigitals'], digitalValue, msgLength)

    def ReadDigital(self, digitalChannel=None):
        """Get the value of the current Digitals outputs as a 32bit integer.
        If digitalChannel is specified, a 0 or 1 is returned.
        """
        value = np.binary_repr(self.status.getStatus('Digitals'))

        if digitalChannel is not None:
            return int(value[-digitalChannel])
        else:
            return int(value, 2)

    def initProfile(self, numReps, repDuration=0, msgLength=20):
        """Prepare the FPGA to run the loaded profile.
        Send a certain number of parameters:
        numberReps and a repDuration

        numberReps -- the number of repetitions to run
        repDuration -- the time interval between repetitions
        msgLength -- int indicating the length of numberReps and repDuration as decimal strings
        """
        self.runCommand(self.commandDict['initProfile'], [numReps, repDuration], msgLength)

    def getframedata(self):
        """Get the current frame"""
        pass

    def triggerExperiment(self):
        """Trigger the execution of an experiment."""
        self.runCommand(self.commandDict['triggerExperiment'])

    def takeImage(self, cameras, lightTimePairs, cameraReadTime = 0, actionsPerMillisecond=100, digitalsBitDepth = 32, msgLength=20):
        """Performs a snap with the selected cameras and light-time pairs

        Generates a list of times and digitals that is sent to the FPGA to be run

        Trigger the camera and wait until all the pixels are exposed
        Expose all lights at the start, then drop them out
        as their exposure times come to an end.
        """
        if lightTimePairs:
            # transform the times in FPGA time units
            lightTimePairs = [(light, int(time * actionsPerMillisecond)) for (light, time) in lightTimePairs]
            cameraReadTime = int(np.ceil(cameraReadTime * actionsPerMillisecond))

            # Sort so that the longest exposure time comes last.
            lightTimePairs.sort(key = lambda a: a[1])

            # at time 0 all the cameras are triggered
            timingList = [(cameras, 0)]

            # the first timepoint: all cameras and lights are turned on and time is 0
            timingList.append((cameras + sum([p[0] for p in lightTimePairs]), cameraReadTime))

            # For similar exposure times, we just send the digitals values that turn off
            # all the lights
            for light, time in lightTimePairs:
                if time == timingList[-1][1]:
                    timingList[-1] = (timingList[-1][0] - light, time + cameraReadTime)
                else:
                    timingList.append((timingList[-1][0] - light, time + cameraReadTime))

            # In the last time point also the cameras should be turned off
            timingList[-1] = (timingList[-1][0] - cameras, timingList[-1][1])

            # Add a 0 at the end will stop the execution of the list

            timingList.append((0, 0))

            lightTimePairs = timingList

            sendList = []

            for light, time in lightTimePairs:
                # binarize and concatenate time and digital value
                value = np.binary_repr(time, 32) + np.binary_repr(light, digitalsBitDepth)
                value = int(value, 2)
                sendList.append(value)

            self.runCommand(self.commandDict['takeImage'], sendList, msgLength)

class FPGAStatus(threading.Thread):
    def __init__(self, parent, host, port):
        threading.Thread.__init__(self)
        self.parent = parent
        ## Create a dictionary to store the FPGA status and a lock to access it
        self.currentFPGAStatus = {}
        self.FPGAStatusLock = threading.Lock()

        ## create a socket
        self.socket = self.createReceiveSocket(host, port)

        ## Create a handle to stop the thread
        self.shouldRun = True

    def createReceiveSocket(self, host, port):
        """Creates a UDP socket meant to receive status information
        form the RT-ipAddress

        returns the bound socket
        """
        try:
            # Create an AF_INET, Datagram socket (UDP)
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except socket.error as msg:
            print('Failed to create socket. Error code: ' + str(msg[0]) + ' , Error message : ' + msg[1])

        try:
            # Bind Socket to local ipAddress and port
            s.bind((host , port))
        except socket.error as msg:
            print('Failed to bind address.\n', msg)

        return s

    def getStatus(self, key=None):
        """Method to call from outside to get the status
        """
        if key and self.currentFPGAStatus is not None:
            try:
                with self.FPGAStatusLock:
                    return self.currentFPGAStatus[key]
            except:
                print('Key does not exist')
        else:
            with self.FPGAStatusLock:
                return self.currentFPGAStatus

    def getFPGAStatus(self):
        """This method polls to a UDP socket and get the status information
        of the RT-ipAddress and FPGA.

        It will update the FPGAStatus dictionary.
        """
        try:
            # Receive Datagram
            datagramLength = int(self.socket.recvfrom(4)[0])
            datagram = self.socket.recvfrom(datagramLength)[0]
        except:
            return None
        # parse json datagram
        return json.loads(datagram)

    def publishFPGAStatusChanges(self, newStatus):
        """FInd interesting status or status changes in the FPGA and publish them

        return the newStatus but with the status reset so not to publish multiple times
        """
        if newStatus['Event'] == 'FPGA done':
            events.publish(events.EXECUTOR_DONE, self.parent.serviceName)
            print(newStatus['Event'])
            newStatus['Event'] = ''

        return newStatus

    def run(self):

        self.currentFPGAStatus = self.getFPGAStatus()

        while self.shouldRun:
            newFPGAStatus = self.getFPGAStatus()
            with self.FPGAStatusLock:
                if newFPGAStatus is not None and newFPGAStatus != self.currentFPGAStatus:

                    # Publish any interesting change and update
                    self.currentFPGAStatus = self.publishFPGAStatusChanges(newStatus=newFPGAStatus)

            ## wait for a period of half the broadcasting rate of the FPGA
            time.sleep(FPGA_UPDATE_RATE / 2)
