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

"""Generates response map correction files (which use detailed
response maps of the cameras, combined with linear interpolation, to
correct for nonlinear camera response).
"""

import decimal

import matplotlib

import cockpit.gui.image_sequence_viewer
import cockpit.gui.progress_dialog
import cockpit.handlers.camera
import cockpit.util.correct_nonlinear
import cockpit.util.datadoc
import cockpit.util.threads
import cockpit.util.user_config
from cockpit import events
from cockpit.experiment import action_table, experiment, offset_gain_correction
from cockpit.gui import gui_utils


matplotlib.use("WXAgg")
import threading

import matplotlib.backends.backend_wxagg
import matplotlib.figure
import numpy
import wx


## Provided so the UI knows what to call this experiment.
EXPERIMENT_NAME = "Response map correction file"


class ResponseMapExperiment(
    offset_gain_correction.OffsetGainCorrectionExperiment
):
    """This class collects the data needed to perform response-map correction.

    It takes exposures of some flat illumination source (e.g. a Chroma slide)
    at varying exposure times, to characterize the response of the cameras to
    incident light. Structurally this class is very similar to the
    OffsetGainCorrectionExperiment class.
    """

    def __init__(
        self,
        cameras,
        lights,
        exposure_settings,
        num_exposures,
        save_path,
        exposure_times,
        cosmic_ray_threshold,
        should_preserve_intermediary_files,
        **kwargs,
    ):
        """Set up the response map experiment.

        Args:
            num_exposures (int): How many images to take for each exposure
                time.
            exposure_times (list): Exposure times to take images for.
            cosmic_ray_threshold (float): If any pixels are more than this many
                standard deviations away from the median of the overall
                image, then the image is discarded.
            should_preserve_intermediary_files (bool): If True, then save the raw
                data in addition to the averaged files.
        """
        # Fill in some dummy values here for parameters that we don't actually
        # use.
        super().__init__(
            cameras,
            lights,
            exposure_settings,
            num_exposures,
            save_path,
            exposure_multiplier=0,
            max_intensity=None,
            cosmic_ray_threshold=cosmic_ray_threshold,
            should_preserve_intermediary_files=should_preserve_intermediary_files,
        )
        self.num_exposures = num_exposures
        self.save_path = save_path
        self.exposure_times = map(decimal.Decimal, exposure_times)
        self.cosmic_ray_threshold = cosmic_ray_threshold
        ## List of (exposure time, averaged images, sample raw image) tuples.
        self.times_and_images = []
        ## Maximum width/height of an image for any camera we use.
        self.max_image_dims = [0, 0]

    @cockpit.util.threads.call_in_new_thread
    def run(self):
        """Very similar to OffsetGainCorrectionExperiment.run() except that we
        calculate the exposure times to use differently, and have a different
        stopping condition.

        We also generate some output at the end to demonstrate our results.
        """
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
        for camera, read_time in self.camera_to_readout_time.items():
            if type(read_time) is not decimal.Decimal:
                raise RuntimeError(
                    "Camera %s did not provide an exact (decimal.Decimal) readout time"
                    % camera.name
                )

        for camera, func in self.cam_to_func.items():
            events.subscribe(events.NEW_IMAGE % camera.name, func)
        for exposure_time in self.exposure_times:
            if self.should_abort:
                break
            self.cam_to_images = {}
            self.cam_to_num_images_received = {}
            self.cam_to_lock = {}
            for camera in self.cameras:
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

            self.table = self.generate_actions(exposure_time)
            self.table.sort()
            self.examine_actions()
            self.table.sort()
            self.table.enforce_positive_timepoints()
            self.last_minute_actions()
            self.done_receiving_thread = threading.Thread(target=self.waiter)
            self.done_receiving_thread.start()
            self.execute()

            if self.should_abort:
                break

            # Wait until it's been a short time after the last received image.
            self.done_receiving_thread.join()
            progress = cockpit.gui.progress_dialog.ProgressDialog(
                "Processing images",
                "Processing images for exposure time %.4f" % exposure_time,
                parent=None,
            )
            self.process_images(exposure_time)
            progress.Destroy()

        for camera, func in self.cam_to_func.items():
            events.unsubscribe(events.NEW_IMAGE % camera.name, func)

        self.save()
        self.show_results()
        self.cleanup()

    def save(self):
        """Save our averaged images together in a single file, with the exposure time in the extended header."""
        # First, re-order our images into WTZ order (with the Z axis being
        # flat).
        num_times = len(self.times_and_images)
        num_cams = len(self.times_and_images[0][1])
        all_images = numpy.zeros(
            (num_cams, num_times, 1, self.max_image_dims[0], self.max_image_dims[1]),
            dtype=numpy.float32,
        )
        exposure_times = []
        for timepoint, (exposure_time, images, raw_images) in enumerate(
            self.times_and_images
        ):
            for wavelength, image in enumerate(images):
                height, width = image.shape
                all_images[wavelength, timepoint, 0, :height, :width] = image
                exposure_times.append(exposure_time)

        header = cockpit.util.datadoc.make_header_for(
            all_images, wavelengths=[cam.wavelength for cam in self.cameras]
        )

        # Number of bytes allocated to the extended header: 4 per image, since
        # we use a 32-bit floating point for the exposure time.
        header.next = 4 * num_cams * num_times
        header.NumFloats = 1
        handle = open(self.save_path, "wb")
        handle.write(header._array.tobytes())
        exposure_times = numpy.array(exposure_times, dtype=numpy.float32)
        handle.write(exposure_times)
        handle.write(all_images)
        handle.close()

    def show_results(self):
        """Demonstrate our results to the user in a number of ways."""
        # First, show plots of the linearity of some of the pixels, before
        # and after correction.
        x_vals = numpy.array([d[0] for d in self.times_and_images])
        averaged_images = numpy.array([d[1] for d in self.times_and_images])
        raw_images = numpy.array([d[2] for d in self.times_and_images])
        self.plot_pixels(
            x_vals, raw_images, "Pre-corrected pixel linearity survey"
        )
        corrected_images = numpy.empty(raw_images.shape)
        for cam in range(raw_images.shape[1]):
            corrector = cockpit.util.correct_nonlinear.Corrector(
                x_vals, averaged_images[:, cam]
            )
            corrected_images[:, cam] = map(corrector.correct, raw_images[:, cam])
        self.plot_pixels(
            x_vals, corrected_images, "Corrected pixel linearity survey"
        )

        # Also show what the images look like before and after correction.
        # This requires reordering the images from (exposure time, cam, Y, X)
        # to (cam, exposure time, 1, Y, X)
        raw_images.shape = list(raw_images.shape) + [1]
        corrected_images.shape = raw_images.shape
        # T, W, Y, X, Z -> W, T, Z, Y, X
        raw_images = raw_images.transpose(1, 0, 4, 2, 3)
        corrected_images = corrected_images.transpose(1, 0, 4, 2, 3)
        # Ensure we have valid datatypes for display.
        raw_images = numpy.float32(raw_images)
        corrected_images = numpy.float32(corrected_images)
        viewer1 = cockpit.gui.image_sequence_viewer.ImageSequenceViewer(
            raw_images, "Uncorrected sample images", parent=None
        )
        viewer2 = cockpit.gui.image_sequence_viewer.ImageSequenceViewer(
            corrected_images, "Corrected sample images", parent=None
        )

    def plot_pixels(self, x_vals, images, title):
        """Given some images, plot a few pixels."""
        figure = matplotlib.figure.Figure((6, 4), dpi=100, facecolor=(1, 1, 1))
        axes = figure.add_subplot(1, 1, 1)
        axes.set_axis_bgcolor("white")
        axes.set_title(title)
        axes.set_ylabel("Individual pixel value")
        axes.set_xlabel("Exposure time (ms)")

        # Plot pixels from the centers of each quadrant as well as from
        # the center of the image and the average of the image as a whole.
        lines = []
        labels = []
        for cam in range(images.shape[1]):
            cam_images = images[:, cam]
            height, width = cam_images[0].shape
            cy = height / 2
            cx = width / 2
            for color, x_offset, y_offset in [
                ("r", -1, -1),
                ("g", -1, 1),
                ("b", 1, 1),
                ("c", 1, -1),
                ("m", 0, 0),
            ]:
                y = cy + y_offset * height / 4
                x = cx + x_offset * width / 4
                lines.append(axes.plot(x_vals, cam_images[:, y, x], color))
                labels.append("Pixel at %d, %d for cam %d" % (x, y, cam))
            lines.append(axes.plot(x_vals, map(numpy.mean, cam_images), "k"))
            labels.append("Average for cam %d" % cam)
        figure.legend(lines, labels, loc="upper left")
        frame = wx.Frame(None, title="Linearity plot")
        canvas = matplotlib.backends.backend_wxagg.FigureCanvasWxAgg(
            frame, -1, figure
        )
        canvas.draw()
        # Big enough to be visible, but unlikely to run off the edge of
        # the screen.
        frame.SetSize((640, 480))
        frame.Show()

    def generate_actions(self, exposure_time):
        """Create the ActionTable needed to run the experiment.

        We basically ignore the values in self.exposure_settings in favor of the
        provided exposure time.
        """
        table = action_table.ActionTable()
        cur_time = 0
        all_cams = set(self.cameras)
        for cameras, light_time_pairs in self.exposure_settings:
            used_cams = all_cams.intersection(cameras)
            if used_cams:
                settings = []
                for light, time in light_time_pairs:
                    # We can't actually have a zero exposure time, so use
                    # 1ns as a minimum.
                    exposure_time = max(
                        exposure_time, decimal.Decimal(".000001")
                    )
                    settings.append((light, exposure_time))
                for i in range(self.num_exposures):
                    cur_time = self.expose(cur_time, used_cams, settings, table)
        return table

    def process_images(self, exposure_time):
        """Examine the images in self.cam_to_images, discard any that indicate cosmic ray strikes or have unusual median intensities, average the remainder, and put them into self.cam_to_averages.

        Return a (possibly empty) set of cameras that had at least 1 valid
        image.
        """
        active_cameras = set()
        averages = []
        raws = []
        cameras = sorted(self.cam_to_images.keys())
        for camera in cameras:
            images = self.cam_to_images[camera][
                : self.cam_to_num_images_received[camera]
            ]
            # Save the last image, on the assumption that any issue with
            # camera or light variance will have gotten flattened out by that
            # time.
            raws.append(images[-1])
            if self.should_preserve_intermediary_files:
                # Save the raw data.
                cockpit.util.datadoc.write_data_as_mrc(
                    images.astype(numpy.uint16),
                    "%s-raw-%s-%04.5fms"
                    % (self.save_path, camera.name, exposure_time),
                )

            # Calculate a threshold for cosmic ray strikes.
            std_dev = numpy.std(images)
            median = numpy.median(images)
            print(
                "For camera", camera, "have median", median, "and std", std_dev
            )
            threshold = self.cosmic_ray_threshold * std_dev + median
            # To cope with the fact that some images may be improperly
            # exposed, we want to find statistics that are true for most
            # of the images, hence the per-image statistics here.
            median_std = numpy.median([numpy.std(image) for image in images])
            median_median = numpy.median(
                [numpy.median(image) for image in images]
            )
            accumulator = numpy.zeros(images[0].shape, dtype=numpy.float32)
            num_clean_images = 0
            for image in images:
                image_max = image.max()
                image_median = numpy.median(image)
                if (
                    image_max < threshold
                    and abs(image_median - median_median) < median_std
                ):
                    # Verified no cosmic ray strike and sensible intensity.
                    accumulator += image
                    num_clean_images += 1

            accumulator /= num_clean_images

            averages.append(accumulator)
            for i in range(2):
                self.max_image_dims[i] = max(
                    self.max_image_dims[i], accumulator.shape[i]
                )
            print(num_clean_images, "images are valid")
        self.times_and_images.append((exposure_time, averages, raws))


## A consistent name to use to refer to the experiment class itself.
EXPERIMENT_CLASS = ResponseMapExperiment
from cockpit.gui.gui_utils import CSVVALIDATOR, FLOATVALIDATOR, INTVALIDATOR


class ExperimentUI(wx.Panel):
    """Generate the UI for special parameters used by this experiment."""

    def __init__(self, parent, config_key):
        super().__init__(parent=parent)
        self.config_key = config_key
        sizer = wx.GridSizer(2, 2, 2, 2)
        ## Maps strings to TextCtrls describing how to configure
        # response curve experiments.
        self.settings = self.load_settings()
        self.response_args = {}
        for key, label, helper_string, validator in [
            (
                "response_map_num_exposures",
                "Number of exposures",
                "How many exposures to take for each exposure time.",
                INTVALIDATOR,
            ),
            (
                "response_map_exposure_times",
                "Exposure times",
                "Comma-separated list of exposure times at which to collect data.",
                CSVVALIDATOR,
            ),
            (
                "response_map_cosmic_ray_threshold",
                "Cosmic ray threshold",
                "If any pixels in an image are more than this many standard deviations from the median, then the image is discarded.",
                FLOATVALIDATOR,
            ),
        ]:
            control = gui_utils.addLabeledInput(
                self,
                sizer,
                label=label,
                defaultValue=self.settings[key],
                helperString=helper_string,
            )
            if validator is not None:
                control.SetValidator(validator)
            self.response_args[key] = control
        row_sizer = wx.BoxSizer(wx.HORIZONTAL)
        control = wx.CheckBox(self, label="Preserve intermediary files")
        control.SetValue(
            self.settings["response_map_should_preserve_intermediary_files"]
        )
        row_sizer.Add(control)
        gui_utils.addHelperString(
            self,
            row_sizer,
            "Keep the raw data in addition to the averaged files.",
        )
        self.response_args["response_map_should_preserve_intermediary_files"] = (
            control
        )
        sizer.Add(row_sizer)
        self.SetSizerAndFit(sizer)

    def augment_params(self, params):
        """Given a parameters dict (parameter name to value) to hand to the experiment instance, augment them with our special parameters."""
        self.save_settings()
        params["num_exposures"] = gui_utils.tryParseNum(
            self.response_args["response_map_num_exposures"]
        )
        tokens = self.response_args["response_map_exposure_times"].GetValue()
        tokens = tokens.split(",")
        params["exposure_times"] = map(float, tokens)
        params["cosmic_ray_threshold"] = gui_utils.tryParseNum(
            self.response_args["response_map_cosmic_ray_threshold"]
        )
        params["should_preserve_intermediary_files"] = self.response_args[
            "response_map_should_preserve_intermediary_files"
        ].GetValue()
        return params

    def load_settings(self):
        """Load the saved experiment settings, if any."""
        return cockpit.util.user_config.get_value(
            self.config_key + "response_map_experiment_settings",
            default={
                "response_map_cosmic_ray_threshold": "10",
                "response_map_exposure_times": "1,2,3,4,5",
                "response_map_num_exposures": "250",
                "response_map_should_preserve_intermediary_files": False,
            },
        )

    def get_settings_dict(self):
        """Generate a dict of our settings."""
        return {(key, c.GetValue()) for key, c in self.response_args.items()}

    def save_settings(self, settings=None):
        """Save the current experiment settings to config."""
        if settings is None:
            settings = self.get_settings_dict()
        cockpit.util.user_config.set_value(
            self.config_key + "response_map_experiment_settings", settings
        )
