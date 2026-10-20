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

import threading

import cockpit.util.threads
from cockpit import events


## A DeviceHandler acts as the interface between the GUI and the device module.
# In other words, it tells the GUI what the device does, and translates GUI
# events into commands for the device. A variety of stock DeviceHandler
# subclasses are available for representing common types of hardware. In order
# to make your hardware accessible to the UI, you need to make a Device, and
# implement its getHandlers() method so that it returns a list of
# DeviceHandlers.
#
# The device handler now supports multiple UI controls by allowing them
# to watch for device parameter changes: each control should use addWatch
# to register a callback that takes the new value as its only argument:
#    addWatch(parameterName, callback)


class STATES:
    """Device states."""

    error = -1
    disabled = 0
    enabled = 1
    enabling = 2
    constant = 3
    busy = 4


class DeviceHandler:
    @staticmethod
    def cached(f):
        """Decorate a method so that its results are cached per device.

        The cache is cleared by reset_cache.
        """

        def wrapper(self, *args, **kwargs):
            key = (f, args, frozenset(sorted(kwargs.items())))
            # Previously, I checked for key existence and, if it wasn't
            # found, added the key and value to the cache, then returned
            # self.__cache[key]. If another thread calls reset_cache
            # between the cache assignment and the return, this can
            # cause a KeyError, so instead I now put the result in a
            # local variable, cache it, then return the local.
            try:
                return self.__cache[key]
            except KeyError:
                result = f(self, *args, **kwargs)
                self.__cache[key] = result
                return result

        return wrapper

    @staticmethod
    def reset_cache(f=None):
        def wrapper(self, *args, **kwargs):
            self.__cache = {}
            if f is None:
                return f
            else:
                return f(self, *args, **kwargs)

        return wrapper

    def __init__(
        self, name, group_name, is_eligible_for_experiments, callbacks, device_type
    ):
        """Set up the handler for a device.

        Args:
            name (str): The name of the device being controlled. This should
                be unique, as it is used to indicate the specific
                DeviceHandler in many callback functions.
            group_name (str): The name of the group of objects this object
                falls under. Multiple DeviceHandlers can correspond to a
                single group when their purpose is similar.
            is_eligible_for_experiments (bool): True if the device can be used in
                experiments (i.e. data collections).
            callbacks (dict): Mapping of strings to functions. Each Handler has
                a different selection of functions that must be filled in by
                the Device that created it. Refer to the Handler's constructor.
            device_type (str): Type of device this is; each subclass of
                DeviceHandler should have a distinct device_type. Normal users
                don't need to worry about this as it is provided automatically
                by the DeviceHandler subclass.
        """
        # Set up dict for attribute-change listeners.
        super().__init__()
        self._publisher = events.Publisher()
        self.state = None
        self.__cache = {}
        self.name = name
        self.group_name = group_name
        self.callbacks = callbacks
        self.is_eligible_for_experiments = is_eligible_for_experiments
        self.device_type = device_type
        # A set of controls that listen for device events.
        self.enable_lock = threading.Lock()
        self.clear_cache = self.__cache.clear

    def __setattr__(self, key, value):
        object.__setattr__(self, key, value)
        self._publisher.publish(key, value)

    def __lt__(self, other):
        """Define __lt__ to make handlers sortable."""
        return self.name.lower() < other.name.lower()

    def get_identifier(self):
        """Return a string that identifies the device."""
        return "%s:%s" % (self.device_type, self.name)

    def make_ui(self, parent):
        """Construct any necessary UI widgets for this Device to perform its
        job.

        Args:
            parent (wx.Window): The WX object that will own the UI.

        Returns:
            wx.Sizer or None: A WX sizer holding the result, or None if
            nothing is to be inserted into the parent object.
        """
        if "make_ui" in self.callbacks:
            return self.callbacks["make_ui"](parent)
        else:
            return None

    def make_initial_publications(self):
        """Publish any necessary events to declare our initial configuration to
        anything that cares.

        At this point, all device handlers should be initialized.
        """
        get_is_enabled = getattr(
            self, "get_is_enabled", None
        ) or self.callbacks.get("get_is_enabled", None)
        if get_is_enabled:
            events.publish(events.DEVICE_STATUS, self, get_is_enabled())

    def finalize_initialization(self):
        """Do any final initialization actions, now that all devices are set up,
        all subscriptions have been made, and all initial publications are done."""
        pass

    def get_is_eligible_for_experiments(self):
        """Return True if we can be used during experiments."""
        return self.is_eligible_for_experiments

    def get_savefile_info(self):
        """Generate a string of information that we want to save into the
        experiment file's header.

        There's limited space (800 characters) so only important information
        should be preserved. This callback is optional; by default nothing is
        generated.
        """
        if "get_savefile_info" in self.callbacks:
            return self.callbacks["get_savefile_info"](self.name)
        return ""

    def cleanup_after_experiment(self, is_cleanup_final=True):
        """Do any necessary cleanup when an experiment is finished.

        In that situation, some cleanup steps may be unnecessary and should be
        omitted for performance reasons.

        Args:
            is_cleanup_final (bool): Indicates if we're about to leap into a
                followup experiment.
        """
        pass

    def __repr__(self):
        """Debugging: print some pertinent info."""
        return "<%s named %s in group %s>" % (
            self.device_type,
            self.name,
            self.group_name,
        )

    def add_watch(self, name, callback):
        """Add a watch/callback on a device attribute."""
        self._publisher.subscribe(name, callback)

    def remove_watch(self, name: str, callback) -> None:
        """Remove a watch/callback on a device attribute."""
        self._publisher.unsubscribe(name, callback)

    @cockpit.util.threads.call_in_new_thread
    def toggle_state(self, *args, **kwargs):
        """A function that any control can call to toggle enabled/disabled
        state."""
        if self.state == STATES.enabling:
            # Already processing a previous toggle request.
            return
        get_is_enabled = getattr(
            self, "get_is_enabled", None
        ) or self.callbacks.get("get_is_enabled", None)
        set_enabled = getattr(self, "set_enabled", None) or self.callbacks.get(
            "set_enabled", None
        )
        if not all([get_is_enabled, set_enabled]):
            raise Exception(
                "toggleState dependencies not implemented for %s." % self.name
            )

        # Do nothing if lock locked as en/disable already in progress.
        if not self.enable_lock.acquire(False):
            return
        events.publish(events.DEVICE_STATUS, self, STATES.enabling)
        try:
            set_enabled(not (get_is_enabled()))
        except Exception as e:
            events.publish(events.DEVICE_STATUS, self, STATES.error)
            raise Exception(
                "Problem encountered en/disabling %s:\n%s" % (self.name, e)
            )
        finally:
            self.enable_lock.release()
        events.publish(events.DEVICE_STATUS, self, get_is_enabled())

    def add_toggle(self, time, table):
        """Add a toggle event to the action table.

        Return time of last action, and response time before ready after
        trigger.
        """
        table.add_action(time, self, True)
        table.add_action(time + table.toggle_time, self, False)
        return time + table.toggle_time, 0
