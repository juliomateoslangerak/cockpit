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
import time
import traceback
import typing

import wx

import cockpit.util.threads
from cockpit import events
from cockpit.handlers.imager import ImagerHandler


## This module provides an interface for taking images with the current
# active cameras and light sources. It's used only outside of experiment
# mode.


def pause_video(func):
    """A wrapper to pause and resume video."""

    def wrapper(*args, **kwargs):
        was_in_video_mode = wx.GetApp().Imager.am_in_video_mode
        if was_in_video_mode:
            wx.GetApp().Imager.should_stop_video_mode = True
            tstart = time.time()
            while wx.GetApp().Imager.am_in_video_mode:
                time.sleep(0.05)
                if time.time() > tstart + 1.0:
                    print("Timeout pausing video mode - abort and restart.")
                    events.publish(events.USER_ABORT)
                    break
        result = func(*args, **kwargs)
        if was_in_video_mode:
            wx.GetApp().Imager.video_mode()
        return result

    return wrapper


class Imager:
    """Simple container class."""

    def __init__(self, handlers: typing.Sequence[ImagerHandler]) -> None:
        """List of Handlers capable of taking images."""
        self._image_handlers = handlers
        ## Set of active cameras, so we can check their framerates.
        self.active_cameras = set()
        ## Set of active light sources, so we can check their exposure times.
        self.active_lights = set()
        ## Time of last call to take_image(), so we can avoid calling it
        # faster than the time it takes to actually collect another image.
        self.last_image_time = time.time()
        ## Boolean to control activity of the video mode thread.
        self.should_stop_video_mode = False
        ## Boolean that indicates if we're currently in video mode.
        self.am_in_video_mode = False

        self._lock = threading.Lock()
        events.subscribe(events.LIGHT_SOURCE_ENABLE, self._on_light_enable)
        events.subscribe(events.CAMERA_ENABLE, self._on_camera_enable)
        events.subscribe(events.LIGHT_EXPOSURE_UPDATE, self._on_light_update)

        events.subscribe(events.USER_ABORT, self.stop_video)

    def _get_exposure_time(self) -> int:
        if self.active_lights:
            return max([l.get_exposure_time() for l in self.active_lights])
        else:
            return 0

    @pause_video
    def _update_exposure_time(self):
        e_time = self._get_exposure_time()
        for c in self.active_cameras:
            c.set_exposure_time(e_time)

    def _toggle(self, container, thing, should_add):
        if should_add:
            container.add(thing)
        else:
            container.discard(thing)

    def _on_camera_enable(self, handler, is_enabled):
        with self._lock:
            self._toggle(self.active_cameras, handler, is_enabled)
            self._update_exposure_time()

    def _on_light_enable(self, handler, is_enabled):
        with self._lock:
            self._toggle(self.active_lights, handler, is_enabled)
            self._update_exposure_time()

    def _on_light_update(self, handler):
        with self._lock:
            self._update_exposure_time()

    def take_image(self, should_block=False, should_stop_video=True):
        """Take an image.

        Args:
            should_block (bool): True if we want to wait for the cameras and
                lights to be ready so we can take the image; False if we don't
                want to wait and should just give up if they aren't ready.
            should_stop_video (bool): True if we should stop video mode. Only
                really used by self.video_mode().
        """
        from cockpit.experiment import experiment

        if experiment.isRunning():
            print("Skipping takeImage because an experiment is running.")
            return
        elif len(self.active_cameras) == 0:
            message = "There are no cameras enabled."
            wx.MessageBox(message, caption="No cameras active")
            return
        if should_stop_video:
            self.stop_video()
        wait_time = self.get_next_image_time() - time.time()
        if wait_time > 0:
            if should_block:
                time.sleep(wait_time)
            else:
                return
        for handler in self._image_handlers:
            handler.takeImage()
        self.last_image_time = time.time()

    @cockpit.util.threads.call_in_new_thread
    def video_mode(self):
        """Video mode: continuously take images at our maximum update rate.

        We stop whenever the user invokes take_image() manually or the abort
        button is pressed. We also limit our image rate if there are any non-
        room-light light sources to 1 image per second, to avoid excessive
        sample damage.
        """
        if not self.active_cameras:
            # No cameras, no video mode.
            events.publish(cockpit.events.VIDEO_MODE_TOGGLE, False)
            return
        if self.am_in_video_mode:
            # Just cancel the current video mode.
            events.publish(cockpit.events.VIDEO_MODE_TOGGLE, False)
            self.stop_video()
            return

        events.publish(cockpit.events.VIDEO_MODE_TOGGLE, True)
        self.should_stop_video_mode = False
        self.am_in_video_mode = True
        while not self.should_stop_video_mode:
            if not self.active_lights:
                break
            # HACK: only wait for one camera.
            camera = list(self.active_cameras)[0]
            # Some cameras drop frames, i.e., take_image() returns but
            # an image is never received.  If that happens, video_mode
            # waits forever since there's no NEW_IMAGE event hence the
            # timeout.  On top of the time to actual acquire the
            # image, we add 5 seconds for any processing and transfer
            # which should be more than enough (see issue #584).
            timeout = 5.0 + (
                (camera.get_exposure_time() + camera.get_time_between_exposures())
                / 1000
            )
            try:
                events.executeAndWaitForOrTimeout(
                    events.NEW_IMAGE % (camera.name),
                    self.take_image,
                    timeout,
                    should_block=True,
                    should_stop_video=False,
                )
            except Exception as e:
                print("Video mode failed:", e)
                events.publish(cockpit.events.VIDEO_MODE_TOGGLE, False)
                traceback.print_exc()
                break
        self.am_in_video_mode = False
        events.publish(cockpit.events.VIDEO_MODE_TOGGLE, False)

    def stop_video(self):
        """Stop our video thread, if relevant."""
        self.should_stop_video_mode = True

    def get_next_image_time(self):
        """Get the next time it's safe to call take_image(), based on the
        cameras' time between images and the light sources' exposure times."""
        cam_limiter = 0
        for camera in self.active_cameras:
            cam_limiter = max(cam_limiter, camera.get_time_between_exposures())
        light_limiter = self._get_exposure_time()
        # The limiters are in milliseconds; downconvert.
        return self.last_image_time + (cam_limiter + light_limiter) / 1000.0
