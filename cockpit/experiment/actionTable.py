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


class ActionTable:
    """Represent the actions performed during an experiment.

    Each action has a timestamp and the parameters for the action to be
    performed.
    """

    toggleTime = decimal.Decimal(".1")

    def __init__(self):
        # List of (time, handler, parameter) tuples indicating what actions
        # must be taken at what times.
        self.actions = []
        # Time of our first action.
        # \todo How do we handle the removal of actions rendering this invalid?
        # For now, we don't.
        self.firstActionTime = None
        # Time of our last action.
        self.lastActionTime = None

    def addAction(self, time, handler, parameter):
        """Insert an element into self.actions."""
        self.actions.append((time, handler, parameter))
        if self.firstActionTime is None or self.firstActionTime > time:
            self.firstActionTime = time
        if self.lastActionTime is None or self.lastActionTime < time:
            self.lastActionTime = time
        return time

    def addToggle(self, time, handler):
        """Like addDigital, but rapidly toggle the output on and then off.

        Return the time after the toggle is completed.
        """
        time, dt = handler.addToggle(time, self)
        return time

    def getLastActionFor(self, handler):
        """Retrieve the last time and action we performed with the specified
        handler.

        NB assumes that self.actions has been sorted.
        """
        for time, altHandler, parameter in reversed(self.actions):
            if altHandler is handler:
                return time, parameter
        return None, None

    def sort(self):
        r"""Sort all the actions in the table by time.

        \todo We should remove redundant entries in here (e.g. due to
        0 stabilization time for stage movement).
        """
        # First element in each action is the timestamp.
        self.actions.sort(key=lambda a: a[0])

    def clearBadEntries(self):
        """Clear invalid entries from the list.

        Sometimes when the table is modified, an entry needs to be deleted
        without affecting indexing into the list; thus, the user sets it to
        None and then calls this function afterwards.
        """
        pairs = [item for item in enumerate(self.actions)]
        for i, action in reversed(pairs):
            if action is None:
                del self.actions[i]

    def enforcePositiveTimepoints(self):
        """Go through the table and ensure all timepoints are positive.

        NB assumes the table has been sorted.
        """
        delta = -self.actions[0][0]
        if delta < 0:
            # First event is at a positive time, so we're good to go.
            return
        for i in range(len(self.actions)):
            self.actions[i] = (
                self.actions[i][0] + delta,
                self.actions[i][1],
                self.actions[i][2],
            )
        self.firstActionTime += delta
        self.lastActionTime += delta

    def shiftActionsBack(self, markTime, delta):
        """Move all actions after the specified time back by the given offset.

        This makes room for some new action.
        """
        for i, (actionTime, handler, action) in enumerate(self.actions):
            if actionTime >= markTime:
                self.actions[i] = (actionTime + delta, handler, action)
        if self.firstActionTime > markTime:
            self.firstActionTime += delta
        if self.lastActionTime > markTime:
            self.lastActionTime += delta

    def getFirstAndLastActionTimes(self, canUseCache=True):
        """Return the time of the first and last action we have.

        Use our cached values if allowed.
        """
        if canUseCache:
            return self.firstActionTime, self.lastActionTime
        firstTime = lastTime = None
        for actionTime, handler, action in self.actions:
            if firstTime is None:
                firstTime = lastTime = actionTime
            firstTime = min(firstTime, actionTime)
            lastTime = max(lastTime, actionTime)
        return firstTime, lastTime

    def __getitem__(self, index):
        """Access an element in the table."""
        return self.actions[index]

    def __setitem__(self, index, val):
        """Modify an item in the table."""
        self.actions[index] = val

    def __len__(self):
        """Get the length of the table."""
        return len(self.actions)

    def prettyString(self, handlers=[]):
        """Generate pretty text for our table.

        Optionally only for the specified handler(s).
        """
        result = ""
        for event in self.actions:
            if event is None:
                result += "<Deleted event>\n"
            else:
                time, handler, action = event
                if not handlers or handler in handlers:
                    result += "%8.2f % 20s % 20s" % (
                        time,
                        handler.name,
                        action,
                    )
                    result += "\n"
        return result

    def __repr__(self):
        """Cast to a string -- generate a textual description of our actions."""
        return self.prettyString()
