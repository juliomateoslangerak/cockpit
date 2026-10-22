#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2021 University of Oxford, CNRS
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

import queue
import threading
import time

import numcodecs
import numpy as np
import wx
import zarr

import cockpit.util.datadoc
import cockpit.util.threads
from cockpit import events


## Unique ID for identifying saver instances
unique_id = 0


class MrcDataSaver:
    """This class simply records all data received during an experiment and
    saves it to disk in MRC format."""

    def __init__(
        self,
        cameras,
        num_reps,
        rep_duration,
        camera_to_images_per_rep,
        camera_to_ignored_image_indices,
        run_thread,
        save_path,
        pixel_size_z,
        titles,
        camera_to_excitation,
    ):
        """
        Args:
            cameras (list): CameraHandler instances for the cameras that will
                be generating images.
            num_reps (int): How many times the experiment will be repeated.
            rep_duration (float): How long each rep lasts.
            camera_to_images_per_rep (dict): Maps camera handlers to how many
                images to expect for that camera in a single repeat of the
                experiment.
            camera_to_ignored_image_indices (dict): Maps camera handlers to
                indices of images that we don't actually want to keep.
            run_thread (threading.Thread): Thread that is executing the
                experiment. When it exits, we know to stop expecting more
                images.
            save_path (str): Path to save the incoming data to.
            pixel_size_z (float): Size of the Z "pixel" (i.e. distance between
                Z slices).
            titles (list): Strings to insert into the MRC file's header. Per
                the file format, each string can be up to 80 characters long
                and there can be up to 10 of them.
            camera_to_excitation (dict): Maps camera handlers to the excitation
                wavelength used to generate the images it will acquire.
        """
        self.cameras = cameras
        self.num_reps = num_reps
        self.rep_duration = rep_duration
        self.camera_to_images_per_rep = camera_to_images_per_rep
        self.camera_to_ignored_image_indices = camera_to_ignored_image_indices
        self.run_thread = run_thread

        ## We want to write the excitation wavelength for each image
        ## on the metadata (see issue #290).  We only allow one
        ## excitation wavelength per image.  This is a limitation of
        ## the dv format.  We also assume that all images from a
        ## camera will have the same light source.  This is a
        ## limitation of the cockpit interface.
        self.camera_to_excitation = camera_to_excitation

        ## Maximum size, in megabytes, of each file generated.  If the
        # experiment data exceeds this, then a new file will be opened, and
        # each file will have a suffix appended to it (e.g.  ".001", ".002",
        # etc.). This is not a precise cap, since it only considers the amount
        # of space allocated to image data -- not the header or extended
        # header.
        # The default of a googol megabytes ought to be enough to avoid
        # splitting files if no cap is specified. :)
        self.max_filesize = 10 ** 100

        global unique_id
        ## Unique ID for our instance
        self.unique_id = unique_id
        unique_id += 1
        # Find the maximum image size (in pixels) in X and Y. While we're at it,
        # assign a number to each camera, for indexing into our data array
        # later, and figure out how many images per camera we'll actually be
        # *keeping*.
        self.max_width, self.max_height = 0, 0
        ## We need to establish a consistent ordering for cameras so that
        # each image gets stored in the correct part of the file. This
        # maps camera handlers to indices.
        self.camera_to_index = {}
        ## Maps camera handlers to total images kept per rep.
        self.camera_to_images_kept_per_rep = {}
        for i, camera in enumerate(self.cameras):
            width, height = camera.get_image_size()
            self.max_width = max(width, self.max_width)
            self.max_height = max(height, self.max_height)
            self.camera_to_index[camera] = i
            self.camera_to_images_kept_per_rep[camera] = self.camera_to_images_per_rep[
                camera
            ] - len(self.camera_to_ignored_image_indices[camera])
        ## We need this for the upper bound on the array of data we write.
        self.max_images_per_rep = max(self.camera_to_images_kept_per_rep.values())

        ## Number of bytes to allocate for each image in the file.
        # \todo Assuming unsigned 16-bit integer here.
        self.plane_bytes = int(self.max_width * self.max_height * 2)

        ## Number of timepoints per file, based on the above and
        # self.max_filesize.
        self.max_reps_per_file = self.max_filesize // (
            self.max_images_per_rep
            * self.plane_bytes
            * len(self.cameras)
            / 1024.0
            / 1024.0
        )
        # Now check that there are less than 2^15 -1 images per channel as
        # dv files can't index more than this.
        if (self.max_reps_per_file * self.max_images_per_rep) > (2 ** 15) - 1:
            # then ensure that we split files at the end of a time point
            # // operator is floor division
            self.max_reps_per_file = ((2 ** 15) - 1) // self.max_images_per_rep
        # Sanity check.
        self.max_reps_per_file = max(self.max_reps_per_file, 1)
        ## Whether or not we need to split the data into multiple files.
        self.do_need_to_split_files = self.max_reps_per_file < self.num_reps
        # For simplicity's sake, we bring self.max_reps_per_file down to
        # self.num_reps in cases where we only need a single file anyway.
        self.max_reps_per_file = min(self.num_reps, self.max_reps_per_file)

        ## Maps ints to cameras; the ints represent the order in which the
        # images are stored.
        self.index_to_camera = {v: k for k, v in self.camera_to_index.items()}
        ## Timestamp of the first image we receive.
        # We need this so we can rebase the timestamps of images to
        # to be relative to the beginning of the experiment -- Python
        # timestamps can't be stored directly as 32-bit floating points without
        # losing a lot of precision. And we want to store image timestamps in
        # the extended header, to help us identify when frames get dropped.
        self.first_timestamp = None

        ## Time at which we last received an image, so we know when images
        # have stopped arriving.
        self.last_image_time = time.time()
        self.start_time = time.time()

        ## File handles we will write the data to.
        self.file_handles = []
        ## Filenames for same.
        self.file_names = []
        if self.do_need_to_split_files:
            # We have multiple file_handles, each with a suffix.
            # A bit tricky here: we want a suffix that has only as many
            # digits as needed, e.g. not doing ".001" when you're only going to
            # use 2 files.
            num_file_handles = int(
                np.ceil(float(self.num_reps) / self.max_reps_per_file)
            )
            num_digits = int(np.ceil(np.log10(num_file_handles)))
            # Generates e.g. "%05d" if we need 5 digits, or "%01d" if we only
            # need 1.
            format_string = "%0" + str(num_digits) + "d"
            for i in range(num_file_handles):
                filename = "%s.%s" % (save_path, format_string % i)
                self.file_handles.append(open(filename, "wb"))
                self.file_names.append(filename)
        else:
            # We have just a single filehandle with the save path as specified.
            self.file_handles.append(open(save_path, "wb"))
            self.file_names.append(save_path)

        ## Lock on writing to each file.
        self.file_locks = [threading.Lock() for handle in self.file_handles]

        pixel_size_xy = wx.GetApp().Objectives.get_pixel_size()
        lens_id = wx.GetApp().Objectives.get_current().lens_ID
        # wavelength should always be on camera even if "0"
        wavelengths = [c.wavelength for c in self.cameras]

        ## Size of one plane's worth of metadata in the extended header.
        num_integers = 8
        num_floats = 32
        self.extended_bytes = 4 * (num_integers + num_floats)

        ## MRC header objects for each file.
        self.headers = []
        self.int_metadata_buffers = []
        self.float_metadata_buffers = []
        for i in range(len(self.file_handles)):
            # Calculate how many timepoints fit into this particular file
            # (potentially different for the final file).
            num_timepoints = self.max_reps_per_file
            if i == len(self.file_handles) - 1:
                num_timepoints = self.num_reps - (
                        self.max_reps_per_file * (len(self.file_handles) - 1)
                )
            header = cockpit.util.datadoc.make_header_for_shape(
                (
                    len(self.cameras),
                    num_timepoints,
                    self.max_images_per_rep,
                    self.max_height,
                    self.max_width,
                ),
                np.uint16,
                pixel_size_xy,
                pixel_size_z,
                wavelengths,
            )
            # write the lens_id to the header if not zero (meaning undefined)
            if lens_id != 0:
                header.LensNum = lens_id

            # By default, the headers generated by DataDoc are for files in ZWT
            # order. But for efficient saving of large multi-wavelength files,
            # we need to store in WZT order (where the cameras are as close
            # together as possible).
            header.ImgSequence = 1
            # Write out the "titles" (metadata, like exposure settings)
            temp_titles = list(titles)
            if self.do_need_to_split_files and len(titles) < 8:
                # We have room for an extra title indicating where this file
                # falls in the sequence.
                temp_titles.append(
                    "File %d of %d; base timepoint %d"
                    % (i + 1, len(self.file_handles), i * self.max_reps_per_file)
                )
            header.NumTitles = len(temp_titles)
            header.title[: len(temp_titles)] = temp_titles
            # Write the size of the extended header, in bytes.
            header.next = (
                self.extended_bytes
                * self.max_images_per_rep
                * len(self.cameras)
                * num_timepoints
            )
            # Number of 32-bit ints and floats in extended header, per plane.
            header.NumIntegers = num_integers
            header.NumFloats = num_floats

            self.headers.append(header)

            ## This will hold the metadata for one image plane at a
            ## time and will be written into the extended header.  We
            ## could create a new array each time for each plane but
            ## these arrays are small and there will be many image
            ## planes.  We do this to avoid memory fragmentation.
            self.int_metadata_buffers.append(
                np.array([0] * num_integers, dtype=np.int32)
            )
            float_metadata_buffer = np.array([0.0] * num_floats, dtype=np.float32)
            float_metadata_buffer[12] = 1.0  # intensity scaling
            self.float_metadata_buffers.append(float_metadata_buffer)

        # Write the headers, to get us started. We will re-write this at the
        # end when we have more metadata to fill in (specifically, the min/max
        # values for each wavelength).
        for i, handle in enumerate(self.file_handles):
            with self.file_locks[i]:
                cockpit.util.datadoc.write_mrc_header(self.headers[i], handle)

        ## List of how many images we've received, on a per-camera basis.
        self.images_received = [0] * len(self.cameras)
        ## List of how many images we've written, on a per-camera basis.
        self.images_kept = [0] * len(self.cameras)
        ## List of functions that receive image data and feed it into
        # self.images_received.
        self.lambdas = []
        ## List of (min, max) tuples, on a per-camera basis, tracking
        # the dimmest and brightest pixels.
        self.min_max_vals = []

        ## True if we should stop collecting data.
        self.should_abort = False
        ## True if we are done collecting data.
        self.am_done = False
        ## Queue of (camera index, image data, timestamp) tuples for images
        # that need to be saved
        self.image_queue = queue.Queue()

        # Use dye name if available, otherwise use camera name.
        names = [camera.dye or camera.name for camera in self.cameras]
        totals = []
        for camera in self.cameras:
            totals.append(self.camera_to_images_kept_per_rep[camera] * self.num_reps)
        ## Thread that handles updating the UI.
        self.status_thread = StatusUpdateThread(
            names, totals, self.num_reps, self.rep_duration
        )

        # Start the data-saving thread.
        self.save_data()

        self.start_collecting()
        self.save_thread = threading.Thread(
            target=self.execute_and_save, name="Experiment-execute-save"
        )
        self.save_thread.start()

    def start_collecting(self):
        """Subscribe to the new-camera-image events for the cameras we care
        about.

        Save the functions we generate for handling the subscriptions, so we
        can unsubscribe later. Initialize self.min_max_vals. Start our status-
        update thread.
        """
        for camera in self.cameras:

            def func(data, metadata, camera=camera):
                return self.on_image(self.camera_to_index[camera], data, metadata)

            self.lambdas.append(func)
            events.subscribe(events.NEW_IMAGE % camera.name, func)

            self.min_max_vals.append((float("inf"), float("-inf")))
        events.subscribe(events.USER_ABORT, self.on_abort)
        self.status_thread.start()

    def on_abort(self):
        """User aborted; stop saving data."""
        self.should_abort = True
        self.status_thread.should_stop = True

    def execute_and_save(self):
        """Wait for the run_thread to finish, then wait a bit longer in case
        some images are laggardly, before we close our file_handles."""
        # Joining the thread doesn't actually work until it has started,
        # hence the delay here.
        time.sleep(0.5)
        self.run_thread.join()

        # Wait until it's been a bit without getting any more images in, or
        # until we have all the images we expected to get for each camera.
        while (
                time.time() - self.last_image_time < self.rep_duration + 1.0
        ) or not self.image_queue.empty():
            #            print ((time.time() - self.start_time),self.rep_duration*self.num_reps)
            am_done = True
            for camera in self.cameras:
                total = self.images_kept[self.camera_to_index[camera]]
                target = self.camera_to_images_kept_per_rep[camera] * self.num_reps
                if total != target:
                    # There exists a camera for which we do not have all
                    # images yet.
                    am_done = False
                    break
            if am_done or self.should_abort:
                break
            time.sleep(0.01)
        self.am_done = True

        self.cleanup()

        # Determine min/max vals for each wavelength.
        for header in self.headers:
            for i in range(len(self.cameras)):
                # HACK: camera 1 is supposed to get min/max/median. However,
                # computing the median of a large dataset takes a very long
                # time (30s for a 2GB file on a fairly powerful computer),
                # so we just store 0.
                min_val, max_val = self.min_max_vals[i]
                if i == 0:
                    setattr(header, "mmm1", (min_val, max_val, 0))
                else:
                    setattr(header, "mm%d" % (i + 1), (min_val, max_val))
        # Rewrite the headers, now that we know what the min/max values are.
        # Of course, these won't be precisely accurate for every file.
        # \todo Track min/max values on a per-file basis.
        # Then, close the filehandle.
        for i, handle in enumerate(self.file_handles):
            with self.file_locks[i]:
                cockpit.util.datadoc.write_mrc_header(self.headers[i], handle)
                handle.close()

    def cleanup(self):
        """Clean up once saving is completed."""
        self.status_thread.should_stop = True
        for i, camera in enumerate(self.cameras):
            events.unsubscribe(events.NEW_IMAGE % camera.name, self.lambdas[i])
        events.unsubscribe(events.USER_ABORT, self.on_abort)

    def on_image(self, camera_index, image_data, metadata):
        """Receive new data, and add it to the queue."""
        self.image_queue.put((camera_index, image_data, metadata))

    @cockpit.util.threads.call_in_new_thread
    def save_data(self):
        """Continually poll our image_queue and save data to the file."""
        while not self.am_done:
            if self.should_abort:
                # Do nothing.
                return
            camera_index, image_data, metadata = self.image_queue.get()
            timestamp = metadata["timestamp"]
            if self.first_timestamp is None:
                self.first_timestamp = timestamp
            # Store the timestamp as a rebased 32-bit float; we can't use
            # 64-bit due to the file format restriction, and if we don't
            # rebase then the numbers are big enough that we lose decimal
            # precision.
            timestamp = timestamp - self.first_timestamp
            self.write_image(camera_index, image_data, timestamp)

    def write_image(self, camera_index, image_data, timestamp):
        """Write a single image to the file."""
        self.images_received[camera_index] += 1
        camera = self.index_to_camera[camera_index]
        # First determine if we actually want to keep this image.
        if (
            self.images_received[camera_index]
            % self.camera_to_images_per_rep[camera]
        ) in self.camera_to_ignored_image_indices[camera]:
            # This image is one that should be discarded.
            return

        # Calculate the time and Z indices for the new image. This will in turn
        # help us to calculate which file to write to and the offset of the
        # image in the file.
        num_images = self.images_kept[camera_index]
        timepoint = num_images // self.max_images_per_rep
        file_index = timepoint // self.max_reps_per_file
        # Rebase the timepoint to be relative to the beginning of this specific
        # file.
        timepoint -= file_index * self.max_reps_per_file
        z_index = num_images % self.camera_to_images_kept_per_rep[camera]

        num_cameras = len(self.cameras)
        plane_index = (
                int(timepoint * self.max_images_per_rep * num_cameras)
                + (z_index * num_cameras)
                + camera_index
        )

        ## Offsets for the plane metadata in the extended header, and
        ## for the plane data in the image section.  1024 is the
        ## length of the base header.
        metadata_offset = 1024 + (plane_index * self.extended_bytes)
        data_offset = (
            1024
            + int(self.headers[file_index].next)
            + (plane_index * self.plane_bytes)
        )

        height, width = image_data.shape

        # Pad with zeros. I wouldn't normally think this would be
        # necessary, but we get "invalid argument" errors when writing
        # to the filehandle if we don't.
        # \todo Figure out why this is necessary.
        padded_buffer = np.zeros(
            (self.max_height, self.max_width), dtype=np.uint16
        )
        padded_buffer[:height, :width] = image_data

        image_min = image_data.min()
        image_max = image_data.max()

        ex_wavelength = self.camera_to_excitation[camera]
        em_wavelength = camera.wavelength

        with self.file_locks[file_index]:
            handle = self.file_handles[file_index]

            ## The extended header has the following structure per
            ## plane (see issue #290):
            ##
            ##     8 32bit signed integers whose meaning we don't
            ##     know.  Often are all set to zero.
            ##
            ##     Followed by 32 32bit floats.  We only what the
            ##     first 14 are:
            ##
            ##     photosensor reading (typically in mV)
            ##     elapsed time (seconds since experiment began)
            ##     x stage coordinates
            ##     y stage coordinates
            ##     z stage coordinates
            ##     minimum intensity
            ##     maximum intensity
            ##     mean intensity
            ##     exposure time (seconds)
            ##     neutral density (fraction of 1 or percentage)
            ##     excitation wavelength
            ##     emission wavelength
            ##     intensity scaling (usually 1)
            ##     energy conversion factor (usually 1)
            ##
            ## Experience from inspecting actual dv files from API
            ## systems, tells us that we can leave most of them at
            ## zero.
            int_metadata_buffer = self.int_metadata_buffers[file_index]
            float_metadata_buffer = self.float_metadata_buffers[file_index]
            float_metadata_buffer[1] = timestamp
            float_metadata_buffer[5] = image_min
            float_metadata_buffer[6] = image_max
            # TODO float_metadata_buffer[8] could be exposure time in seconds
            float_metadata_buffer[10] = ex_wavelength
            float_metadata_buffer[11] = em_wavelength

            try:
                handle.seek(metadata_offset)
                handle.write(int_metadata_buffer)
                handle.write(float_metadata_buffer)
                handle.seek(data_offset)
                handle.write(padded_buffer)
            except Exception as e:
                print("Error writing image:", e)
                raise e

            self.images_kept[camera_index] += 1
            self.last_image_time = time.time()

            cur_min, cur_max = self.min_max_vals[camera_index]
            self.min_max_vals[camera_index] = (
                min(cur_min, image_min),
                max(cur_max, image_max),
            )

        # Update the status text. But first, check for abort/experiment
        # completion, since we may actually be done now and we don't want
        # a misleading status text.
        if self.should_abort or self.am_done:
            return
        self.status_thread.new_image(camera_index)

    def get_filenames(self):
        """Return a list of the file_names we are writing to."""
        return self.file_names


class ZarrDataSaver:
    """This class simply records all data received during an experiment and
    saves it to disk in ome-zarr format."""

    def __init__(
        self,
        exposure_settings,
        num_reps,
        rep_duration,
        camera_to_images_per_rep,
        camera_to_ignored_image_indices,
        run_thread,
        save_path,
        pixel_size_xy,
        slice_height,
        ome_metadata=None,
        downscale=None,
        max_layer=1,
        downscale_method="nearest",
        compression="blosc",
        overwrite=True,
    ):
        """Set up the OME-Zarr saver for the images of an experiment.

        When it exits, we know to stop expecting more images.

        Args:
            exposure_settings (list): List of ([cameras], [(light, exposure
                time)]) tuples describing how to take images.
            num_reps (int): How many times the experiment will be repeated.
            rep_duration (float): How long each rep lasts.
            camera_to_images_per_rep (dict): Maps camera handlers to how many
                images to expect for that camera in a single repeat of the
                experiment.
            camera_to_ignored_image_indices (dict): Maps camera handlers to
                indices of images that we don't actually want to keep.
            run_thread (threading.Thread): Thread that is executing the
                experiment.
            save_path (str): Path to save the incoming data to.
            pixel_size_xy (float): Size of the XY "pixel".
            slice_height (float): Distance between Z slices.
            ome_metadata (str): OME-XML Metadata to be included in the
                OME-Zarr file.
            downscale (int): Downscaling factor for x and y axes.
            max_layer (int): Number of downscalings. Defaults to 1.
            downscale_method (str): Downscaling method. Default is 'nearest'.
            compression (str): Compressor to be used. Defaults to 'blosc'.
            overwrite (bool): Overwrite existing files.
        """
        self._exposure_settings = exposure_settings
        self._num_reps = num_reps
        self._rep_duration = rep_duration
        self._camera_to_images_per_rep = camera_to_images_per_rep
        self._camera_to_ignored_image_indices = camera_to_ignored_image_indices
        self.run_thread = run_thread
        self.save_path = save_path
        self._pixel_size_xy = pixel_size_xy
        self._slice_height = slice_height

        global unique_id
        # Unique ID for our instance
        self.unique_id = unique_id
        unique_id += 1

        # We need this so we can rebase the timestamps of images to
        # to be relative to the beginning of the experiment -- Python
        # timestamps can't be stored directly as 32-bit floating points without
        # losing a lot of precision. And we want to store image timestamps in
        # the extended header, to help us identify when frames get dropped.
        self._first_timestamp = None

        # Time at which we last received an image, so we know when images
        # have stopped arriving.
        self._last_image_time = time.time()
        self._start_time = time.time()

        # Exposure settings are List of ([cameras], [(light, exposure time)])
        # tuples describing how to take images. We need to transform this into
        # a list of channels, which is a list of (camera, [light, exposure time])
        # tuples. Those will map to channels in the data.
        self._channels = []
        for cameras, exposure in self._exposure_settings:
            self._channels.extend((camera, exposure) for camera in cameras)

        # Some experiments are triggering images that are not ment to be kept.
        # For each camera, find the number of images that are going to be kept
        self._camera_to_images_kept_per_rep = {
            camera: self._camera_to_images_per_rep[camera]
            - len(self._camera_to_ignored_image_indices[camera])
            for camera in self._camera_to_images_per_rep
        }

        # Map cameras to the channel index where they are used
        self._cameras_to_channel_ids()

        # Calculate what are going to be the shapes of the channels
        self._compute_channel_shapes()

        # We need to know how many images we are going to keep per camera
        self._camera_to_images_kept = {
            camera: images_per_rep * self._num_reps
            for camera, images_per_rep in self._camera_to_images_kept_per_rep.items()
        }
        # Dict to keep track of images we've received, on a per-camera basis.
        self._images_received = {
            camera: 0 for camera in self._camera_to_channel_ids
        }
        # Dict to keep track of how many images we've written, on a per-camera basis.
        self._images_kept = {camera: 0 for camera in self._camera_to_channel_ids}

        # List of functions that receive image data.
        self._image_receiving_funcs = []
        for camera in self._camera_to_channel_ids:

            def func(data, metadata, camera=camera):
                return self.on_image(camera, data, metadata)

            self._image_receiving_funcs.append(func)
            events.subscribe(events.NEW_IMAGE % camera.name, func)

        # Zarr specific settings
        # Scaling and compression
        self.compression = compression
        self.overwrite = overwrite

        # zarr specific settings
        # Shape and chunking
        self.array_shape = (
            self._num_reps,  # T: timepoints
            len(self._channels),  # C: number of channels
            self.channel_shapes[0][0],  # Z: number of slices
            self.channel_shapes[0][1],  # Y
            self.channel_shapes[0][2],  # X
        )
        self.chunk_shape = (
            1,
            1,
            1,
            self.channel_shapes[0][1],
            self.channel_shapes[0][2],
        )
        # self.scaler = scale.Scaler(
        #     downscale=downscale,
        #     max_layer=max_layer,
        #     method=downscale_method
        # )
        # self.coordinate_transforms = [
        #     [{'scale': [slice_height, pixel_size_xy, pixel_size_xy], 'type': 'scale'}],
        # ]
        # self.storage_options = {
        #     "chunks": chunk_shape,
        #     "compression": compression,
        #     "overwrite": overwrite,
        # }
        self._create_zarr_array()

        # A ThreadLock to protect some operations.
        self._thread_lock = threading.Lock()

        ## Flag to indicate if we should stop collecting data because of:
        # user abort
        self.should_abort = False
        # experiment done.
        self.am_done = False
        # Queue of (camera index, image data, timestamp) tuples for images
        # that need to be saved
        self._image_queue = queue.Queue()

        # Thread that handles updating the UI.
        # TODO: This might be simplified by passing to the StatusUpdateThread
        #  a dictionary with the camera name and the target number of images to be kept.
        #  Also because the total of expected images is used later in the executeAndSave method.
        self.status_thread = StatusUpdateThread(
            [camera.dye or camera.name for camera in self._camera_to_channel_ids],
            list(self._camera_to_images_kept.values()),
            self._num_reps,
            self._rep_duration,
        )

        # Start the data-saving thread.
        self.save_data()

        # Start our status-update thread.
        events.subscribe(events.USER_ABORT, self.on_abort)
        self.status_thread.start()

        self.save_thread = threading.Thread(
            target=self.execute_and_save, name="Experiment-execute-save"
        )
        self.save_thread.start()

    def _cameras_to_channel_ids(self):
        """
        Map every camera to the index of the channel where they are used
        """
        self._camera_to_channel_ids = {
            camera: [] for camera in self._camera_to_images_kept_per_rep
        }
        for channelId, channel in enumerate(self._channels):
            self._camera_to_channel_ids[channel[0]] += [channelId]

    def _compute_channel_shapes(self):
        """
        Calculate the shape of the channel based on the input arguments.
        If the channel shape is not the same for every channel we raise
        an Exception.
        """
        # We expect that every channel has the same number of images (typically
        # a z-stack). So the expected images per channel must match the expected
        # images per camera divided by the number of channels where the camera is
        # used.
        self.channel_shapes = []
        for channel in self._channels:
            camera = channel[0]
            if (
                self._camera_to_images_kept_per_rep[camera]
                % len(self._camera_to_channel_ids[camera])
                != 0
            ):
                raise ValueError(
                    "The number of images per camera is not divisible by the number of channels"
                )
            shape_z = self._camera_to_images_kept_per_rep[camera] // len(
                self._camera_to_channel_ids[camera]
            )
            shape_xy = camera.get_image_size()
            self.channel_shapes.append((shape_z, shape_xy[0], shape_xy[1]))

        # We want to make sure that the shape in z is consistent across all
        # channels. This might be supported in the future, but for now we assume
        # that all channels have the same number of slices.
        if any(
            channel_shape[0] != self.channel_shapes[0][0]
            for channel_shape in self.channel_shapes
        ):
            raise NotImplementedError(
                "The number of slices is not the same for all channels"
            )

        # We want to make sure that the x and y dimensions are consistent across all
        # channels. This might be supported in the future, but for now we assume
        # that all channels have the same x and y dimensions.
        if any(
            channel_shape[1] != self.channel_shapes[0][1]
            or channel_shape[2] != self.channel_shapes[0][2]
            for channel_shape in self.channel_shapes
        ):
            raise NotImplementedError(
                "The x and y dimensions are not the same for all channels"
            )

    def on_abort(self):
        """User aborted; stop saving data."""
        self.should_abort = True
        self.status_thread.should_stop = True

    def execute_and_save(self):
        """Wait for the run_thread to finish, then wait a bit longer in case
        some images are laggardly, before we close our file_handles."""
        # Joining the thread doesn't actually work until it has started,
        # hence the delay here.
        time.sleep(0.5)
        self.run_thread.join()

        # Wait until it's been a bit without getting any more images in, or
        # until we have all the images we expected to get for each camera.
        while (
                time.time() - self._last_image_time < self._rep_duration + 1.0
        ) or not self._image_queue.empty():
            am_done = True
            if self._images_kept != self._camera_to_images_kept:
                # There exists a camera for which we do not have all
                # images yet.
                am_done = False
            if am_done or self.should_abort:
                break
            time.sleep(0.01)
        self.am_done = True

        self.cleanup()

    def cleanup(self):
        """Clean up once saving is completed."""
        self.status_thread.should_stop = True
        for i, camera in enumerate(self._camera_to_channel_ids):
            events.unsubscribe(
                events.NEW_IMAGE % camera.name, self._image_receiving_funcs[i]
            )
        events.unsubscribe(events.USER_ABORT, self.on_abort)

    def on_image(self, camera, image_data, metadata):
        """Receive new data, and add it to the queue."""
        self._image_queue.put((camera, image_data, metadata))

    @cockpit.util.threads.call_in_main_thread
    def _create_zarr_array(self):
        self._zarrRoot = zarr.create_group(
            store=self.save_path,
            overwrite=self.overwrite,
            attributes=self._construct_ome_attributes(),
        )
        self._zarr_array = self._zarrRoot.create_array(
            name="0",
            dimension_names=["t", "c", "z", "y", "x"],
            shape=self.array_shape,
            chunks=self.chunk_shape,
            # compressor=self.compression,
            overwrite=self.overwrite,
            dtype="uint16",
        )

    def _construct_ome_attributes(self):
        ome_metadata = {
            "ome": {
                "version": "0.5",
                # "series": ["0", "1"],  # TODO: Series are to be put here
                "multiscales": [
                    {
                        "name": "5D",
                        "axes": [
                            dict(name="t", type="time", unit="second"),
                            dict(name="c", type="channel"),
                            dict(name="z", type="space", unit="micrometer"),
                            dict(name="y", type="space", unit="micrometer"),
                            dict(name="x", type="space", unit="micrometer"),
                        ],
                        "datasets": [
                            {
                                "path": "0",
                                "coordinateTransformations": [
                                    {
                                        "type": "scale",
                                        "scale": [
                                            self._rep_duration,
                                            1.0,
                                            self._slice_height,
                                            self._pixel_size_xy,
                                            self._pixel_size_xy,
                                        ],
                                    }
                                ],
                            }
                        ],
                    },
                ],
            },
        }
        return ome_metadata

    @cockpit.util.threads.call_in_new_thread
    def save_data(self):
        """Continually poll our image_queue and save data to the file."""
        while not self.am_done:
            if self.should_abort:
                # Do nothing.
                return
            camera, image_data, metadata = self._image_queue.get()
            timestamp = metadata["timestamp"]
            if self._first_timestamp is None:
                self._first_timestamp = timestamp
            # Store the timestamp as a rebased 32-bit float; we can't use
            # 64-bit due to the file format restriction, and if we don't
            # rebase then the numbers are big enough that we lose decimal
            # precision.
            timestamp = timestamp - self._first_timestamp
            self.write_image(camera, image_data, timestamp)

    def write_image(self, camera, image_data, timestamp):
        """Write a single image to the file."""
        self._images_received[camera] += 1
        # First determine if we actually want to keep this image.
        if (
                self._images_received[camera] % self._camera_to_images_per_rep[camera]
        ) in self._camera_to_ignored_image_indices[camera]:
            # This image should be discarded.
            return

        # Calculate the time and Z indices for the new image. This will in turn
        # help us to calculate which file to write to and the offset of the
        # image in the file.
        image_index = self._images_kept[camera]
        time_index = image_index // self._camera_to_images_kept_per_rep[camera]
        channel_z_reminder = image_index % self._camera_to_images_kept_per_rep[camera]
        channel_index = self._camera_to_channel_ids[camera][
            channel_z_reminder
            // self.channel_shapes[0][
                0
            ]  # Only takes into account one possible z-shape per channel or camera
        ]
        z_index = (
            channel_z_reminder % self.channel_shapes[0][0]
        )  # Only takes into account one possible z-shape per channel or camera

        with self._thread_lock:
            self.append_image_to_zarr(time_index, channel_index, z_index, image_data)
            self._images_kept[camera] += 1
            self._last_image_time = time.time()

        # Update the status text. But first, check for abort/experiment
        # completion, since we may actually be done now and we don't want
        # a misleading status text.
        if self.should_abort or self.am_done:
            return
        self.status_thread.new_image(
            list(self._camera_to_images_kept_per_rep.keys()).index(camera)
        )

    @cockpit.util.threads.call_in_main_thread
    def append_image_to_zarr(self, time_index, channel_index, z_index, image_data):
        """
        Append a new image to the zarr array.
        We call this in the main thread, so we can ensure it is run in the
        asyncio event loop.
        """
        self._zarr_array[time_index, channel_index, z_index] = image_data

    def get_filenames(self):
        """Return a list of the file_names we are writing to."""
        return self.save_path


class StatusUpdateThread(threading.Thread):
    """This thread handles telling the saving status light to update twice per
    second."""

    def __init__(self, camera_names, totals, num_reps, rep_duration):
        super().__init__()
        ## List of names of the cameras.
        self.camera_names = camera_names
        ## List of images received per camera.
        self.images_received = [0 for name in self.camera_names]
        ## Lock on updating the above.
        self.image_count_lock = threading.Lock()
        ## List of total images expected per camera.
        self.totals = totals
        self.num_reps = num_reps
        self.rep_duration = rep_duration
        self.start_time = time.time()
        ## Set to True to end the thread.
        self.should_stop = False
        self.name = "DataSaver-status"

    def run(self):
        prev_counts = list(self.images_received)
        self.update_text()
        count = 0
        while not self.should_stop:
            if prev_counts != self.images_received:
                # Have received new images since the last update;
                # update the display.
                count = 0
                with self.image_count_lock:
                    self.update_text()
                    prev_counts = list(self.images_received)
            else:
                # No images; wait a bit, but update wait text every 0.5s
                time.sleep(0.1)
                count = count + 1
                if count == 5:
                    count = 0
                    self.update_text()
        # Clear the status light.
        events.publish(events.UPDATE_STATUS_LIGHT, "image count", "")

    def update_text(self):
        """Push a new text to the status light."""
        status_text = []
        for i, name in enumerate(self.camera_names):
            cur_count = self.images_received[i]
            max_count = self.totals[i]
            status_text.append("%s: %d/%d" % (name, cur_count, max_count))
        if (sum(self.images_received) % (max_count / self.num_reps)) == 0:
            # we are between reps
            rep_time = (time.time() - self.start_time) % self.rep_duration
            time_left = self.rep_duration - rep_time
            status_text.append("waiting %.0fs for next repeat" % time_left)
        events.publish(
            events.UPDATE_STATUS_LIGHT, "image count", " | ".join(status_text)
        )

    def new_image(self, index):
        """Update our image count."""
        with self.image_count_lock:
            self.images_received[index] += 1
