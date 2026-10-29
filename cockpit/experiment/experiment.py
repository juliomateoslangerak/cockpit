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
import gc
import logging
import os
import threading
import time

import wx
from microscope import ElectronicShutteringMode

import cockpit.handlers.camera
import cockpit.interfaces.stage_mover
from cockpit import depot, events
from cockpit.experiment import data_saver
from cockpit.gui import gui_utils


_logger = logging.getLogger(__name__)


## Purely for debugging purposes, a copy of the last Experiment that was
# executed.
last_experiment = None

## A track of files generated in previous experiments, so they can be
# viewed in the UI. A list of lists, since each experiment can generate
# multiple files.
generated_filenames = []


def is_running():
    """Is an experiment running?"""
    if last_experiment is None:
        return False
    else:
        return last_experiment.is_running()


class Experiment:
    """This class is the root class for generating and running experiments.

    You should make a subclass of this class to implement a specific
    experiment type.
    """

    def __init__(
        self,
        num_reps,
        rep_duration,
        z_positioner,
        alt_bottom,
        z_height,
        slice_height,
        exposure_settings,
        other_handlers=[],
        metadata="",
        save_path="",
    ):
        """This constructor accepts certain parameters that will be shared
        by all experiment types.

        Args:
            num_reps (int): Number of repetitions of the experiment to perform.
            rep_duration (float): Amount of time to spend on each repetition,
                or 0 to spend as little as possible. In seconds.
            z_positioner (StagePositioner): Handler to use to move in Z.
            alt_bottom (float): Altitude of the stage at the bottom of the
                stack.
            z_height (float): Total height of the stack.
            slice_height (float): Distance between slices in the stack.
            exposure_settings (list): List of ([cameras], [(light, exposure
                time)]) tuples describing how to take images.
            other_handlers (list): Miscellaneous handlers that are involved
                in the experiment.
            metadata (str): Extra metadata to insert into the "titles"
                section of the saved file.
            save_path (str): Path to save image data to. If this isn't
                provided then no data will be saved.

        *Altitudes* refer to the net position of the Z stage, and are used by
        the stagemover. *z* values refer to the position of the z_positioner
        specified in the args.
        """
        self.num_reps = num_reps
        self.rep_duration = rep_duration
        self.z_positioner = z_positioner
        self.alt_bottom = alt_bottom
        self.z_height = z_height
        self.slice_height = slice_height
        self.exposure_settings = exposure_settings

        self.cameras = []
        self.lights = []
        for cameras, light_times in exposure_settings:
            self.cameras.extend(cameras)
            self.lights.extend([light for light, time in light_times])

        self.other_handlers = list(other_handlers)
        self.metadata = metadata
        self.save_path = save_path

        self.run_thread = None

        # Check for save paths that don't actually have a final filename
        # (i.e. just point to a directory); those aren't valid.
        if not os.path.basename(self.save_path):
            self.save_path = ""

        ## List of all handlers we care about, so we can conveniently set them
        # up.
        self.all_handlers = self.cameras + self.lights + self.other_handlers
        if self.z_positioner is not None:
            # It may be None in some special experiments that don't do Z
            # stacks.
            self.all_handlers.append(self.z_positioner)
        # Ensure all handlers are experiment-eligible.
        for handler in self.all_handlers:
            if not handler.get_is_eligible_for_experiments():
                raise RuntimeError(
                    "Handler [%s] is not usable in experiments." % handler.name
                )

        ## Maps camera handlers to their minimum time between exposures.
        # Must be populated after exposure time has been set on camera.
        self.camera_to_readout_time = {}
        ## Maps cameras to whether or not they need to be blanked before they

        # next take an image (because they expose continuously and other

        # cameras have taken images while they were waiting).

        self.camera_to_is_ready = {c: True for c in self.cameras}
        ## Maps camera handlers to how many images we'll be taking with that
        # camera.
        self.camera_to_image_count = {c: 0 for c in self.cameras}
        ## Maps camera handlers to indices of which images we will be ignoring
        # from them.
        self.camera_to_ignored_image_indices = {c: set() for c in self.cameras}

        ## Whether or not we should stop the experiment at the next opportunity.
        self.should_abort = False
        events.subscribe(events.USER_ABORT, self.on_abort)

        ## Resultant Z altitude prior to execution, used to restore position.
        self.initial_altitude = None

        ## Baseline position of the z_positioner, used as offset in Z stacks
        self.z_start = None

        ## Maps light sources to sets of exposure times used. This is helpful
        # when setting the "titles" in the MRC header.
        self.light_to_exposure_time = {l: set() for l in self.lights}

    def on_abort(self):
        """Cancel the experiment, if it's running."""
        self.should_abort = True

    def run(self):
        """Run the experiment.

        We spin off the actual execution and cleanup into separate threads.
        """
        # Returns True to close config dialog box, False or None otherwise.
        # Check if the user is set to save to an already-existing file.
        if self.save_path and os.path.exists(self.save_path):
            if not gui_utils.get_user_permission(
                ("The file:\n%s\nalready exists. " % self.save_path)
                + "Are you sure you want to overwrite it?"
            ):
                return False

        global last_experiment
        last_experiment = self
        self.sanity_check_environment()
        self.prepare_handlers()

        self.camera_to_readout_time = {
            c: c.get_time_between_exposures(is_exact=True) for c in self.cameras
        }
        for camera, read_time in self.camera_to_readout_time.items():
            if type(read_time) is not decimal.Decimal:
                raise RuntimeError(
                    "Camera %s did not provide an exact (decimal.Decimal) readout time"
                    % camera.name
                )

        # Indicate any frame transfer cameras for reset at start of table.
        for camera in self.cameras:
            if (
                camera.get_exposure_mode()
                == cockpit.handlers.camera.TRIGGER_AFTER
            ):
                self.camera_to_is_ready[camera] = False

        self.create_valid_action_table()
        if (
            self.num_reps > 1
            and self.rep_duration < self.table.lastActionTime / 1000
        ):
            warning = (
                "Repeat duration is less than the time required to run "
                "one repeat. Choose:"
                "\n    'OK' to run repeats as fast as possible;"
                "\n    'Cancel' to go back and change parameters."
            )
            if not gui_utils.get_user_permission(warning):
                return False
            # set rep_duration to the last table action
            self.rep_duration = float(self.table.last_action_time) / 1000.0

        if not self.last_minute_actions():
            return False

        self.run_thread = threading.Thread(
            target=self.execute, name="Experiment-execute"
        )

        save_thread = None
        if self.save_path and max(self.camera_to_image_count.values()):
            camera_to_excitation = {c: 0.0 for c in self.cameras}
            for cameras, light_time_pairs in self.exposure_settings:
                ## If there's multiple light sources for the same
                ## camera pick the one with highest wavelength.  Main
                ## case of multiple light sources is single molecule
                ## localisation where a lower wavelength is doing the
                ## pumping while but excitation for fluorescence is
                ## with the higher wavelength.
                if light_time_pairs:
                    max_wavelength = max(
                        [l.wavelength for l, t in light_time_pairs]
                    )
                else:
                    max_wavelength = 0.0
                for camera in cameras:
                    if camera not in self.cameras:
                        continue
                    camera_to_excitation[camera] = max(
                        camera_to_excitation[camera], max_wavelength
                    )

            if self.save_path.endswith((".dv", ".mrc")):
                saver = data_saver.MrcDataSaver(
                    cameras=self.cameras,
                    num_reps=self.num_reps,
                    rep_duration=self.rep_duration,
                    camera_to_images_per_rep=self.camera_to_image_count,
                    camera_to_ignored_image_indices=self.camera_to_ignored_image_indices,
                    run_thread=self.run_thread,
                    save_path=self.save_path,
                    pixel_size_z=self.slice_height,
                    titles=self.generate_titles(),
                    camera_to_excitation=camera_to_excitation,
                )
            elif self.save_path.endswith(".zarr"):
                saver = data_saver.ZarrDataSaver(
                    exposure_settings=self.exposure_settings,
                    num_reps=self.num_reps,
                    rep_duration=self.rep_duration,
                    camera_to_images_per_rep=self.camera_to_image_count,
                    camera_to_ignored_image_indices=self.camera_to_ignored_image_indices,
                    run_thread=self.run_thread,
                    save_path=self.save_path,
                    pixel_size_xy=wx.GetApp().objectives.get_pixel_size(),
                    slice_height=self.slice_height,
                )
            else:
                raise RuntimeError(
                    "Unsupported file format %s" % self.save_path
                )

            generated_filenames.append(saver.get_filenames())

        self.run_thread.start()

        cleanup_thread = threading.Thread(
            target=self.cleanup,
            args=[self.run_thread, saver.save_thread],
            name="Experiment-cleanup",
        )
        cleanup_thread.start()
        return True

    def create_valid_action_table(self):
        """Create an ActionTable by calling self.generate_actions, and give our
        Devices a chance to sign off on it."""
        self.table = self.generate_actions()
        self.table.sort()
        self.examine_actions()
        self.table.sort()
        self.table.enforce_positive_timepoints()

    def sanity_check_environment(self):
        """Perform any necessary sanity checks to ensure that the environment
        is set up properly.

        Raise an exception if anything is wrong.
        """
        pass

    def prepare_handlers(self):
        """Prepare all of the handlers needed in the experiment so that they're
        in the correct mode."""
        # Store the pre-experiment altitude.
        self.initial_altitude = cockpit.interfaces.stage_mover.get_position()[-1]
        # Ensure that we're the only ones moving things around.
        cockpit.interfaces.stage_mover.wait_for_stop()
        # TODO: Handling multiple movers on an axis is broken. Do not proceed if
        # anything but the innermost Z mover is selected. Needs a proper fix.
        if (
            cockpit.interfaces.stage_mover.mover.cur_handler_index
            < len(depot.get_sorted_stage_movers()[2]) - 1
        ):
            wx.MessageBox("Wrong axis mover selected.")
            raise Exception("Wrong axis mover selected.")
        # Prepare our position.
        cockpit.interfaces.stage_mover.go_to_z(self.alt_bottom, should_block=True)
        self.z_start = cockpit.interfaces.stage_mover.get_all_positions()[-1][-1]
        events.publish(events.PREPARE_FOR_EXPERIMENT, self)
        # Prepare cameras.
        for camera in self.cameras:
            # We set the exposure time here. This needs to be set before
            # the action table is generated, since the action table
            # uses camera.get_exposure_time to figure out timings.
            exposure_time = float(self.get_exposure_time_for_camera(camera))
            camera.set_exposure_time(exposure_time)

    def examine_actions(self):
        """Allow devices to examine the ActionTable we will be running, and
        modify it if necessary."""
        for handler in depot.get_handlers_of_type(depot.EXECUTOR):
            handler.examine_actions(self.table)

    def last_minute_actions(self):
        """Do any last-minute actions immediately before starting the
        experiment.

        Return False if anything goes wrong.
        """
        return True

    def generate_actions(self):
        """Generate an ActionTable of events to perform during the experiment.

        Return the ActionTable instance.
        """
        return None

    def execute(self):
        """Run the experiment.

        Return True if it was successful.
        """
        _logger.info("Experiment.execute started.")
        # Iteratively find the ExperimentExecutor that can tackle the largest
        # portion of self.table, have them run it, and wait for them to finish.
        executors = depot.get_handlers_of_type(depot.EXECUTOR)
        self.should_abort = False
        for rep in range(self.num_reps):
            start_time = time.time()
            rep_duration = None
            cur_index = 0
            should_stop = False
            # Need to track delay introduced by dropping back to software timing.
            delay = 0.0
            while cur_index < len(self.table):
                if cur_index > 0:
                    # Update the delay
                    next_time = (
                        delay
                        + start_time
                        + float(self.table[cur_index][0]) / 1000.0
                    )
                    delay += max(0, time.time() - next_time)

                if self.should_abort:
                    _logger.error(
                        "Cancelling on rep %d after %d actions due to user abort",
                        rep,
                        cur_index,
                    )
                    break
                best = None
                best_len = 0
                for executor in executors:
                    num_lines = executor.get_num_runnable_lines(
                        self.table, cur_index
                    )
                    if best is None or num_lines > best_len:
                        best = executor
                        best_len = num_lines
                num_reps = 1
                if best_len == len(self.table):
                    # This executor can handle the entire experiment, so we
                    # should tell them to handle the repeats as well.
                    num_reps = self.num_reps
                    should_stop = True
                    # Expand from seconds to milliseconds
                    rep_duration = self.rep_duration * 1000

                if best_len == 0:
                    # No executor can run this line. See if we can fall back to software.
                    fn = None
                    t, h, action = self.table[cur_index]
                    if (
                        h.device_type == depot.CAMERA
                        and "soft_trigger" in h.callbacks
                    ):
                        fn = lambda: h.callbacks["soft_trigger"]()
                    elif h.device_type == depot.STAGE_POSITIONER:
                        fn = lambda: h.move_absolute(action)

                    if fn is None:
                        raise RuntimeError(
                            "Found a line that no executor could handle: %s"
                            % str(self.table.actions[cur_index])
                        )
                    # Wait until this action is due.
                    if cur_index > 0:
                        time_to_next = (
                            delay
                            + start_time
                            + float(self.table[cur_index][0]) / 1000.0
                            - time.time()
                        )
                        time.sleep(max(0, time_to_next))
                    fn()
                    cur_index += 1
                else:
                    # Don't resume execution too early.
                    # TODO: would be better to pass a 'do not start before' argument
                    # to the handler, so any work it has to do does not add further
                    # delays.
                    if cur_index > 0:
                        time_to_next = (
                            delay
                            + start_time
                            + float(self.table[cur_index][0]) / 1000.0
                            - time.time()
                        )
                        time.sleep(max(0, time_to_next))

                    events.executeAndWaitFor(
                        events.EXPERIMENT_EXECUTION,
                        best.execute_table,
                        self.table,
                        cur_index,
                        cur_index + best_len,
                        num_reps,
                        rep_duration,
                    )
                    cur_index += best_len

            if should_stop:
                # All reps handled by an executor.
                _logger.debug("Stopping now at %.2f", time.time())
                break
            # Wait for the end of the rep.
            if rep != self.num_reps - 1:
                wait_time = self.rep_duration - (time.time() - start_time)
                time.sleep(max(0, wait_time))
        ## TODO: figure out how long we should wait for the last captures to complete.
        # For now, wait 1s.
        time.sleep(1.0)
        _logger.info("Experiment.execute completed.")
        return True

    def cleanup(self, run_thread=None, save_thread=None):
        """Wait for the provided thread(s) to finish, then clean up our
        handlers."""
        if run_thread is not None:
            run_thread.join()
        if save_thread is not None and save_thread.is_alive():
            events.publish(
                events.UPDATE_STATUS_LIGHT,
                "device waiting",
                "Waiting for saving to complete",
            )
            save_thread.join()
        for handler in self.all_handlers:
            handler.cleanup_after_experiment()
        events.publish(events.CLEANUP_AFTER_EXPERIMENT)
        if self.initial_altitude is not None:
            # Restore our initial altitude.
            cockpit.interfaces.stage_mover.go_to_z(
                self.initial_altitude, should_block=True
            )
        events.publish(events.EXPERIMENT_COMPLETE)
        events.publish(events.UPDATE_STATUS_LIGHT, "device waiting", "")
        # Ensure the save_thread's memory, which includes all the images
        # collected thus far, is garbage collected. Otherwise memory tends
        # to pile up and then the GC has more work to do, which can interfere
        # with future experiments.
        gc.collect()

    def generate_titles(self):
        """Generate the "titles" that provide extra miscellaneous information
        about the experiment.

        These are part of the MRC file format spec: https://web.archive.org/web
        /20190129061250/http://msg.ucsf.edu/IVE/IVE4_HTML/IM_ref2.html There
        can be up to 10 titles and they can have up to 80 characters each. We
        group them by device type.
        """
        type_to_handlers = {}
        # Include light filters for our active lights, even though they aren't
        # a part of self.all_handlers.
        type_to_handlers[depot.LIGHT_FILTER] = []
        filters = depot.get_handlers_of_type(depot.LIGHT_FILTER)
        for light in self.lights:
            wavelength = light.get_wavelength()
            for filter_handler in filters:
                if light in filter_handler.lights:
                    type_to_handlers[depot.LIGHT_FILTER].append(filter_handler)

        for handler in self.all_handlers:
            # We don't care about stage positioners because we always include
            # the complete stage position anyway.
            if handler.device_type != depot.STAGE_POSITIONER:
                if handler.device_type not in type_to_handlers:
                    type_to_handlers[handler.device_type] = []
                type_to_handlers[handler.device_type].append(handler)
        titles = [
            "Date & time: %s; pos: %s"
            % (
                time.strftime("%Y/%m/%d %H:%M:%S"),
                str(
                    [
                        "%.2f" % p
                        for p in (cockpit.interfaces.stage_mover.get_position())
                    ]
                ),
            )
        ]
        # Append the metadata we were given to start.
        for i in range(0, len(self.metadata) + 80, 80):
            substring = self.metadata[i * 80 : (i + 1) * 80]
            if substring:
                titles.append(substring)

        for device_type, handlers in type_to_handlers.items():
            handlers = sorted(handlers, key=lambda a: a.name)
            entries = []
            for handler in handlers:
                text = handler.get_savefile_info()
                if (
                    handler in self.light_to_exposure_time
                    and self.light_to_exposure_time[handler]
                ):
                    text += ": " + ",".join(
                        [
                            "%.3fms" % t
                            for t in sorted(self.light_to_exposure_time[handler])
                        ]
                    )
                    # Record the exposure duration(s) of the light source.
                    # find associated power entries (if they have them)

                    for hand in depot.get_handlers_in_group(handler.group_name):
                        if hand.device_type == depot.LIGHT_POWER:
                            text += " %3.3f mW" % hand.last_power
                if text:
                    entries.append(text)
            if entries:
                entry = "[%s: %s]" % (device_type, ";".join(entries))
                while len(entry) > 80:
                    # Must split it across lines.

                    # \todo For now doing this in an optimally-space-saving
                    # method that will result in ugly titles since we split
                    # lines in the middle of a word.
                    titles.append(entry[:80])
                    entry = entry[80:]
                titles.append(entry)
        if len(titles) > 10:
            raise RuntimeError(
                'Have too much miscellaneous information to fit into the "titles" section of the MRC file (max 10 lines). Lines are:\n%s'
                % "\n".join(titles)
            )
        return titles

    def expose(self, cur_time, cameras, light_time_pairs, table):
        """Add an exposure to the provided ActionTable.

        We're provided with the cameras and lights to use for the exposure,
        as well as how long to expose each light for and when we're allowed
        to start. We need to enforce that all of the cameras are ready to go
        before we trigger them. We also need to enforce that any
        frame-transfer cameras have not seen any light since the last time
        they were blanked.

        Args:
            cur_time (decimal.Decimal): Time at which the exposure may start.
            cameras (list): Cameras to trigger for the exposure.
            light_time_pairs (list): (light, exposure time) tuples describing
                how long to expose each light for.
            table (ActionTable): Table to which the exposure actions are
                added.

        Returns:
            The time at which all exposures are complete.
        """
        # First, determine which cameras are not ready to be exposed, because
        # they may have seen light they weren't supposed to see (due to
        # bleed through from other cameras' exposures). These need
        # to be triggered (and we need to record that we want to throw away
        # those images) before we can proceed with the real exposure.
        cams_to_reset = set()
        for camera in cameras:
            if not self.camera_to_is_ready[camera]:
                cams_to_reset.add(camera)
        if cams_to_reset:
            cur_time = self.reset_cams(cur_time, cams_to_reset, table)
        # Figure out when we can start the exposure, based on the cameras
        # involved: their exposure modes, readout times, and last trigger
        # times determine how soon we can next trigger them (see
        # get_time_when_camera_can_expose() for more information).
        exposure_start_time = cur_time
        # Adjust the exposure start based on when the cameras are ready.
        for camera in cameras:
            cam_exposure_ready_time = self.get_time_when_camera_can_expose(
                table, camera
            )
            exposure_start_time = max(exposure_start_time, cam_exposure_ready_time)

        # Determine the maximum exposure time, which depends on our light
        # sources as well as how long we have to wait for the cameras to be
        # ready to be triggered.
        max_exposure_time = 0
        if light_time_pairs:
            max_exposure_time = max(light_time_pairs, key=lambda a: a[1])[1]
        # Check cameras to see if they have minimum exposure times; take them
        # into account for when the exposure can end. Additionally, if they
        # are frame-transfer cameras, then we need to adjust max_exposure_time
        # to ensure that our triggering of the camera does not come too soon
        # (while it is still reading out the previous frame).

        for camera in cameras:
            max_exposure_time = max(
                max_exposure_time, camera.get_min_exposure_time(is_exact=True)
            )
            if (
                camera.get_exposure_mode()
                == cockpit.handlers.camera.TRIGGER_AFTER
            ):
                next_ready_time = self.get_time_when_camera_can_expose(table, camera)
                # Ensure camera is exposing for long enough to finish reading
                # out the last frame.
                max_exposure_time = max(
                    max_exposure_time, next_ready_time - exposure_start_time
                )

            # If the camera has a rolling shutter we need to add to the camera exposure time the readout time
            # to ensure that all the pixels are exposed when we turn on the lights
            elif (
                    camera.get_shuttering_mode() == ElectronicShutteringMode.ROLLING
            ):
                max_exposure_time += self.camera_to_readout_time[
                    camera
                ] + decimal.Decimal(0.1)

        # Open the shutters for the specified exposure times, centered within
        # the max exposure time. If camera has a rolling shutter, centered in the time frame
        # where all pixels are exposed
        # Note that a None value here means the user wanted to expose the
        # cameras without any special light.
        exposure_end_time = exposure_start_time + max_exposure_time
        for (
            light,
            exposure_time,
        ) in light_time_pairs:
            if (
                light is not None and light.name != "Ambient"
            ):  # i.e. not ambient light
                if (
                    camera.get_shuttering_mode()
                    == ElectronicShutteringMode.ROLLING
                ):
                    # Center with all pixels exposed
                    offset = decimal.Decimal(
                        0.05
                    )  # This is half of the time that was added for security to max_exposure_time
                else:
                    # Center the light exposure.
                    time_slop = max_exposure_time - exposure_time
                    offset = time_slop / 2
                table.add_action(
                    exposure_end_time - exposure_time - offset, light, True
                )
                table.add_action(exposure_end_time - offset, light, False)
            # Record this exposure time.
            if exposure_time not in self.light_to_exposure_time[light]:
                self.light_to_exposure_time[light].add(exposure_time)

        # Trigger the cameras. Keep track of which cameras we *aren't* using
        # here; if they are continuous-exposure cameras, then they may have
        # seen light that they shouldn't have, and need to be invalidated.

        used_cams = set()
        for camera in cameras:
            used_cams.add(camera)
            mode = camera.get_exposure_mode()
            if mode == cockpit.handlers.camera.TRIGGER_AFTER:
                table.add_toggle(exposure_end_time, camera)
            elif mode == cockpit.handlers.camera.TRIGGER_DURATION:
                table.add_action(exposure_start_time, camera, True)
                table.add_action(exposure_end_time, camera, False)
            elif mode == cockpit.handlers.camera.TRIGGER_BEFORE:
                table.add_toggle(exposure_start_time, camera)
            elif mode == cockpit.handlers.camera.TRIGGER_SOFT:
                table.add_action(exposure_start_time, camera, True)
            else:
                raise Exception("%s has no trigger mode set." % camera)
            self.camera_to_image_count[camera] += 1
        for camera in self.cameras:
            if (
                camera not in used_cams
                and camera.get_exposure_mode()
                == cockpit.handlers.camera.TRIGGER_AFTER
            ):
                # Camera is a continuous-exposure/frame-transfer camera
                # and therefore saw light it shouldn't have; invalidate it.
                self.camera_to_is_ready[camera] = False

        return exposure_end_time

    def reset_cams(self, cur_time, cameras, table):
        """Given a set of cameras and a time, trigger the cameras and record
        that we want to throw away the resulting image.

        This blanks the camera sensors so they don't record light that we don't
        care about.
        """
        reset_end_time = cur_time
        for camera in cameras:
            exposure_start = max(
                cur_time, self.get_time_when_camera_can_expose(table, camera)
            )
            # Cameras that have a pre-set exposure time can only use that
            # exposure time for clearing the sensor, hence why we take the
            # maximum of the min exposure time and the current exposure time.
            # \todo Is it possible for get_exposure_time() to be less than
            # get_min_exposure_time()? That would be a bug, right?
            min_exposure_time = max(
                decimal.Decimal(".1"),
                camera.get_min_exposure_time(is_exact=True),
                camera.get_exposure_time(is_exact=True),
            )
            exposure_mode = camera.get_exposure_mode()
            if exposure_mode == cockpit.handlers.camera.TRIGGER_AFTER:
                table.add_toggle(exposure_start + min_exposure_time, camera)
            elif exposure_mode == cockpit.handlers.camera.TRIGGER_DURATION:
                table.add_action(exposure_start, camera, True)
                table.add_action(exposure_start + min_exposure_time, camera, False)
            else:  # TRIGGER_BEFORE case
                table.add_toggle(exposure_start, camera)
            reset_end_time = max(reset_end_time, exposure_start + min_exposure_time)
            self.camera_to_image_count[camera] += 1
            self.camera_to_ignored_image_indices[camera].add(
                self.camera_to_image_count[camera]
            )
            self.camera_to_is_ready[camera] = True
        return reset_end_time + decimal.Decimal("1e-6")

    def get_time_when_camera_can_expose(self, table, camera):
        """Given a camera handle, return the next time that it will be safe to
        start an exposure with that camera, based on its last trigger time,
        its readout time, and its exposure mode:
        - For TRIGGER_AFTER cameras (i.e. frame transfer cameras), the exposure
          starts immediately after the last trigger event for the camera, but
          the next trigger time may be postponed to ensure the camera is ready
          to readout at the end of the exposure.
        - For TRIGGER_DURATION cameras (i.e. external exposure), the camera
          must wait (camera readout time) after the last trigger event before
          it can be triggered again.
        - For TRIGGER_BEFORE cameras (non-frame-transfer external trigger),
          the camera must wait (camera readout time + camera exposure time)
          after the last trigger event before it can be triggered again."""
        last_use_time, action = table.get_last_action_for(camera)
        if last_use_time is None:
            # No actions yet; assume camera is ready at the start of the
            # experiment.
            return 0

        next_use_time = last_use_time
        if camera.get_exposure_mode() in [
            cockpit.handlers.camera.TRIGGER_BEFORE,
            cockpit.handlers.camera.TRIGGER_SOFT,
        ]:
            # The camera actually finished exposing (and started reading
            # out) some time after last_use_time, depending on its declared
            # exposure time.
            next_use_time += camera.get_exposure_time(is_exact=True)
        next_use_time += self.camera_to_readout_time[camera] + decimal.Decimal(0.1)
        return next_use_time

    def get_exposure_time_for_camera(self, camera):
        """Return a calculated exposure time for the specified camera handler,
        based on our exposure settings."""
        exposure_time = 0
        for cameras, light_time_pairs in self.exposure_settings:
            if camera in cameras and light_time_pairs:
                exposure_time = max(
                    exposure_time, max(light_time_pairs, key=lambda a: a[1])[1]
                )
        return exposure_time

    def is_running(self):
        """Is this experiment running?"""
        if self.run_thread is None:
            return False
        else:
            return self.run_thread.is_alive()


def get_last_filenames():
    """Return a list of the files generated by the most recent experiment."""
    if generated_filenames:
        return generated_filenames[-1]
    return None
