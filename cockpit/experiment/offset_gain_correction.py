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

"""Generates offset/gain correction files (which add an offset to
pixel values and then multiply them by a gain factor).
"""

import decimal
import threading
import time

import numpy
import wx

import cockpit.handlers.camera
import cockpit.util.datadoc
import cockpit.util.threads
import cockpit.util.user_config
from cockpit import events
from cockpit.experiment import action_table, experiment
from cockpit.gui import gui_utils


## Provided so the UI knows what to call this experiment.
EXPERIMENT_NAME = "Offset/gain correction file"


class OffsetGainCorrectionExperiment(experiment.Experiment):
    """This class collects the data needed to generate an offset and gain
    correction file.

    It takes exposures of some flat illumination source (e.g. a Chroma slide)
    at varying exposure times, to characterize the response of the cameras to
    incident light.
    """

    def __init__(
        self,
        cameras,
        lights,
        exposure_settings,
        num_exposures,
        save_path,
        exposure_multiplier=2,
        max_intensity=2 ** 12,
        cosmic_ray_threshold=10,
        num_collections=5,
        should_preserve_intermediary_files=False,
        **kwargs,
    ):
        """Set up the offset and gain correction experiment.

        Args:
            num_exposures (int): How many images to take for each exposure
                time.
            exposure_multiplier (float): Factor by which to increase exposure
                time for each step.
            max_intensity (int): Once the max intensity exceeds this value for
                a given camera, we stop collecting data for it.
            cosmic_ray_threshold (float): If any pixels are more than this many
                standard deviations away from the median of the overall
                image, then the image is discarded.
            num_collections (int): Maximum number of different exposure times
                to try, assuming no other stopping condition is hit first.
            should_preserve_intermediary_files (bool): If True, then save the raw
                data to a separate file, alongside the actual result file.
        """
        # Note we fill in some dummy values here since we don't actually
        # move the stage during this experiment, and we handle data saving
        # manually. Some of these values may actually have been passed in via
        # the kwargs parameter (since our caller may have generic code for
        # setting up arbitrary experiments that assumes e.g. a Z stack), but
        # we ignore them.
        super().__init__(
            num_reps=1,
            rep_duration=0,
            z_positioner=None,
            alt_bottom=0,
            z_height=0,
            slice_height=0,
            cameras=cameras,
            lights=lights,
            exposure_settings=exposure_settings,
        )
        self.num_exposures = num_exposures
        self.save_path = save_path
        self.exposure_multiplier = decimal.Decimal(exposure_multiplier)
        self.max_intensity = max_intensity
        self.cosmic_ray_threshold = cosmic_ray_threshold
        self.num_collections = num_collections
        self.should_preserve_intermediary_files = should_preserve_intermediary_files

        ## Maps camera handlers to lists of averaged images.
        self.cam_to_averages = dict([(cam, []) for cam in cameras])
        ## Current image accumulator. This is wiped with each iteration of
        # the experiment. Maps camera handlers to lists of images.
        self.cam_to_images = None
        ## Maps camera handlers to number of images received thus far.
        self.cam_to_num_images_received = None
        ## Maps cameras to locks around the above two fields.
        self.cam_to_lock = {}
        ## Maps camera handlers to functions to record their images.
        self.cam_to_func = {}
        for camera in self.cameras:
            self.cam_to_func[camera] = (
                lambda image, timestamp, camera=camera: self.record_image(
                    image, camera
                )
            )

        ## Thread for tracking when we're done receiving images.
        self.done_receiving_thread = None
        ## Time we last received an image, used in the above thread.
        self.last_image_time = None

    @cockpit.util.threads.call_in_new_thread
    def run(self):
        """Broadly similar to Experiment.run(), but we generate and execute
        multiple tables."""
        # For debugging purposes
        experiment.last_experiment = self

        self.sanity_check_environment()
        self.prepare_handlers()

        self.camera_to_readout_time = dict(
            [
                (c, c.get_time_between_exposures(is_exact=True))
                for c in self.cameras
            ]
        )
        for camera, readTime in self.camera_to_readout_time.items():
            if type(readTime) is not decimal.Decimal:
                raise RuntimeError(
                    "Camera %s did not provide an exact (decimal.Decimal) readout time"
                    % camera.name
                )

        # Start out with no-exposure-time images to get a measured offset.
        multiplier = 0
        for camera, func in self.cam_to_func.items():
            events.subscribe(events.NEW_IMAGE % camera.name, func)
        active_cameras = set(self.cameras)
        for i in range(self.num_collections):
            if not active_cameras or self.should_abort:
                break
            print("Running with cams", active_cameras)
            self.cam_to_images = {}
            self.cam_to_num_images_received = {}
            self.cam_to_lock = {}
            for camera in active_cameras:
                # Prepare a memory buffer to store images in.
                width, height = camera.get_image_size()
                self.cam_to_images[camera] = numpy.zeros(
                    (self.num_exposures, height, width)
                )
                self.cam_to_num_images_received[camera] = 0
                self.cam_to_lock[camera] = threading.Lock()
                # Indicate any frame transfer cameras for reset at start of
                # table.
                if (
                    camera.get_exposure_mode()
                    == cockpit.handlers.camera.TRIGGER_AFTER
                ):
                    self.camera_to_is_ready[camera] = False

            self.table = self.generate_actions(multiplier, active_cameras)
            self.table.sort()
            self.examine_actions()
            self.table.sort()
            self.table.enforce_positive_timepoints()
            self.last_minute_actions()
            self.done_receiving_thread = threading.Thread(target=self.waiter)
            self.done_receiving_thread.start()
            self.execute()
            # Wait until it's been a short time after the last received image.
            self.done_receiving_thread.join()

            if multiplier == 0:
                multiplier = decimal.Decimal(1)
            else:
                multiplier *= self.exposure_multiplier
            active_cameras = self.process_images(multiplier)
            print("Came out with active cams", active_cameras)

        for camera, func in self.cam_to_func.items():
            events.unsubscribe(events.NEW_IMAGE % camera.name, func)

        if self.should_abort:
            # Don't bother processing images.
            self.cleanup()
            return

        results = []
        for camera in self.cameras:
            results.append(self.make_fit(self.cam_to_averages[camera]))
        results = numpy.array(results, dtype=numpy.float32)
        results.shape = (
            len(self.cameras),
            1,
            2,
            results.shape[-2],
            results.shape[-1],
        )

        # Construct a header for the image data.
        pixel_size = wx.GetApp().objectives.get_pixel_size()
        wavelengths = [c.wavelength for c in self.cameras]
        header = cockpit.util.datadoc.make_header_for(
            results, xy_size=pixel_size, z_size=0, wavelengths=wavelengths
        )

        file_handle = open(self.save_path, "wb")
        cockpit.util.datadoc.write_mrc_header(header, file_handle)
        file_handle.write(results)
        file_handle.close()

        self.cleanup()

    def generate_actions(self, multiplier, active_cameras):
        """Create the ActionTable needed to run the experiment.

        Args:
            multiplier (float): Amount to multiply exposure times by.
            active_cameras (set): Cameras that we should still be taking
                images for.
        """
        table = action_table.ActionTable()
        cur_time = 0
        for cameras, light_time_pairs in self.exposure_settings:
            used_cams = active_cameras.intersection(cameras)
            if used_cams:
                settings = []
                for light, time in light_time_pairs:
                    # We can't actually have a zero exposure time.
                    exposure_time = max(
                        time * multiplier, decimal.Decimal(".1")
                    )
                    settings.append((light, exposure_time))
                for i in range(self.num_exposures):
                    cur_time = self.expose(cur_time, used_cams, settings, table)
        return table

    def record_image(self, image, camera):
        """Record an image for the specified camera."""
        with self.cam_to_lock[camera]:
            index = self.cam_to_num_images_received[camera]
            self.cam_to_images[camera][index] = image
            self.cam_to_num_images_received[camera] += 1
            self.last_image_time = time.time()

    def waiter(self):
        """This function waits for a certain amount of time to pass after an
        image is received."""
        # Clearly-invalid initial value
        self.last_image_time = time.time() + 1000
        while time.time() - self.last_image_time < 0.25:
            time.sleep(0.1)

    def process_images(self, multiplier):
        """Examine the images in self.cam_to_images, discard any that indicate
        cosmic ray strikes, average the remainder, and put them into
        self.cam_to_averages.

        Return a set of cameras that had at least 1 valid image.
        """
        active_cameras = set()
        for camera, image_data in self.cam_to_images.items():
            images = image_data[: self.cam_to_num_images_received[camera]]
            if self.should_preserve_intermediary_files:
                # Save the raw data.
                cockpit.util.datadoc.write_data_as_mrc(
                    images.astype(numpy.uint16),
                    "%s-raw-%s-%d" % (self.save_path, camera.name, multiplier),
                )

            std_dev = numpy.std(images)
            median = numpy.median(images)
            print(
                "For camera", camera, "have median", median, "and std", std_dev
            )
            threshold = self.cosmic_ray_threshold * std_dev + median
            clean_images = []
            for image in images:
                max_val = image.max()
                if max_val < threshold and max_val < self.max_intensity:
                    # Verified no cosmic ray strike.
                    clean_images.append(image)
            print(len(clean_images), "images are valid")
            if clean_images:
                self.cam_to_averages[camera].append(
                    numpy.mean(clean_images, axis=0)
                )
                active_cameras.add(camera)
        return active_cameras

    def make_fit(self, images):
        """Given an array of images, make the offset and gain images.

        NB assumes that the first image in images is the measured dark offset.
        """
        x_vals = map(numpy.mean, images)
        y_vals = numpy.array(images)
        image_shape = y_vals.shape[1:]
        y_vals.shape = len(x_vals), numpy.product(image_shape)
        slopes, intercepts = numpy.polyfit(x_vals, y_vals, 1)
        slopes.shape = image_shape

        return numpy.array([images[0], slopes])


## A consistent name to use to refer to the experiment class itself.
EXPERIMENT_CLASS = OffsetGainCorrectionExperiment
from cockpit.gui.gui_utils import FLOATVALIDATOR, INTVALIDATOR


class ExperimentUI(wx.Panel):
    """Generate the UI for special parameters used by this experiment."""

    def __init__(self, parent, config_key):
        super().__init__(parent=parent)

        self.config_key = config_key
        self.settings = self.load_settings()

        sizer = wx.GridSizer(3, 2, 2, 2)
        ## Maps strings to TextCtrls describing how to configure
        # correction file experiments.
        self.correction_args = {}
        for key, label, helper_string, validator in [
            (
                "correction_num_exposures",
                "Number of exposures",
                "How many exposures to take for each exposure time.",
                INTVALIDATOR,
            ),
            (
                "correction_num_collections",
                "Number of collections",
                "Maximum number of exposure times to collect data for.",
                INTVALIDATOR,
            ),
            (
                "correction_exposure_multiplier",
                "Exposure multiplier",
                "Multiplicative factor that governs how quickly we increase exposure time for measuring the camera's response.",
                FLOATVALIDATOR,
            ),
            (
                "correction_max_intensity",
                "Max intensity",
                'Any images above this value are discarded; if we complete imaging and no images "survive", then we are done with data collection.',
                FLOATVALIDATOR,
            ),
            (
                "correction_cosmic_ray_threshold",
                "Cosmic ray threshold",
                "If any pixels in an image are more than this many standard deviations from the median, then the image is discarded.",
                FLOATVALIDATOR,
            ),
        ]:
            control = gui_utils.add_labeled_input(
                self,
                sizer,
                label=label,
                default_value=self.settings[key],
                helper_string=helper_string,
            )
            control.SetValidator(validator)
            self.correction_args[key] = control
        row_sizer = wx.BoxSizer(wx.HORIZONTAL)
        control = wx.CheckBox(self, label="Preserve intermediary files")
        control.SetValue(
            self.settings["correction_should_preserve_intermediary_files"]
        )
        row_sizer.Add(control)
        gui_utils.add_helper_string(
            self,
            row_sizer,
            "Keep the raw data in addition to the averaged files.",
        )
        self.correction_args["correction_should_preserve_intermediary_files"] = (
            control
        )
        sizer.Add(row_sizer)
        self.SetSizerAndFit(sizer)

    def augment_params(self, params):
        """Given a parameters dict (parameter name to value) to hand to the
        experiment instance, augment them with our special parameters."""
        self.save_settings()
        params["num_exposures"] = gui_utils.try_parse_num(
            self.correction_args["correction_num_exposures"]
        )
        params["num_collections"] = gui_utils.try_parse_num(
            self.correction_args["correction_num_collections"]
        )
        params["exposure_multiplier"] = gui_utils.try_parse_num(
            self.correction_args["correction_exposure_multiplier"], float
        )
        params["max_intensity"] = gui_utils.try_parse_num(
            self.correction_args["correction_max_intensity"]
        )
        params["cosmic_ray_threshold"] = gui_utils.try_parse_num(
            self.correction_args["correction_cosmic_ray_threshold"], float
        )
        params["should_preserve_intermediary_files"] = self.correction_args[
            "correction_should_preserve_intermediary_files"
        ].GetValue()
        return params

    def load_settings(self):
        """Load the saved experiment settings, if any."""
        return cockpit.util.user_config.get_value(
            self.config_key + "offset_gain_experiment_settings",
            default={
                "correction_cosmic_ray_threshold": "10",
                "correction_exposure_multiplier": "2",
                "correction_max_intensity": "5000",
                "correction_num_collections": "5",
                "correction_num_exposures": "250",
                "correction_should_preserve_intermediary_files": False,
            },
        )

    def get_settings_dict(self):
        """Generate a dict of our settings."""
        return {(key, c.GetValue()) for key, c in self.correction_args.items()}

    def save_settings(self, settings=None):
        """Save the current experiment settings to config."""
        if settings is None:
            settings = self.get_settings_dict()
        cockpit.util.user_config.set_value(
            self.config_key + "offset_gain_experiment_settings", settings
        )
