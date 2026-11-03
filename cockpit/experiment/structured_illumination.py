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

"""Runs SIM experiments."""

import decimal
import math
import os
import shutil
import tempfile

import numpy as np
import wx

import cockpit.util.datadoc
import cockpit.util.mrc
import cockpit.util.user_config
from cockpit import depot
from cockpit.experiment import action_table, experiment
from cockpit.gui import gui_utils


## Provided so the UI knows what to call this experiment.
EXPERIMENT_NAME = "Structured Illumination"

## Maps possible collection orders to their ordering (0: angle, 1: phase, 2: z).
COLLECTION_ORDERS = {
    #        "Angle, Phase, Z": (0, 1, 2),
    #        "Angle, Z, Phase": (0, 2, 1),
    #        "Phase, Angle, Z": (1, 0, 2),
    #       "Phase, Z, Angle": (1, 2, 0),
    "Z, Angle, Phase": (2, 0, 1),
    "Z, Phase, Angle": (2, 1, 0),
}


def collection_order_tuple(order_str):
    """Return collection order in a tuple of 1 character c-style index.

    A bit more sensible, lower-case, one character, tuple.
    """
    to_one_char = {
        0: "a",
        1: "p",
        2: "z",
    }
    z_order = [to_one_char[x] for x in COLLECTION_ORDERS[order_str]]

    return tuple(z_order)


def postpad_data(data, shape):
    """Return padded data at end ofp each dimension and reshape.

    This is to handle truncated files when it is required to add blank
    values to obtain a specific shape.  The blank values are zero or
    NaN when supported by the datatype.  See cockpit bug #289.
    """
    postpad_length = shape - np.array(data.shape)
    pad_width = list(zip([0] * len(shape), postpad_length))
    ## Let numpy figure out what to convert NaN into for blank values
    return np.pad(
        data, pad_width, mode="constant", constant_values=[np.nan]
    )


def reorder_z_dim(data, order_packed, z_lengths, z_order, z_wanted):
    """Reorder the Z dimension of a numpy array.

    To fix the order of the Z dimension, we reshape the numpy array
    into the real 7 dimensions array that it is, tranpose it as
    necessary, and then reshape it back into the fake 5 dimensions.

    Args:
        data - np.array
        order_packed - tuple of 1 character
        z_lengths - tuple of 3 elements with the length of each of the
            dimensions packed in z, same order as z_order
        z_order - a 3 element tuple of 1 character, the order of the z
            dimension.
        z_wanted - a 3 element tuple of 1 character, with the wanted
            order of the z dimension.
    """
    assert data.ndim == len(
        order_packed
    ), "DATA ndims different from lenght of ORDER_PACKED"
    assert sorted(z_order) == [
        "a",
        "p",
        "z",
    ], "Z_ORDER does not have only 'a, z, p'"
    assert sorted(z_order) == sorted(
        z_wanted
    ), "Z_ORDER not same elements as Z_WANTED"

    z_idx = order_packed.index("z")
    order_in = order_packed[0:z_idx] + z_order + order_packed[z_idx + 1 :]
    order_out = order_packed[0:z_idx] + z_wanted + order_packed[z_idx + 1 :]

    packed_shape = data.shape
    unpacked_shape = (
        packed_shape[0:z_idx] + z_lengths + packed_shape[z_idx + 1 :]
    )

    ## If we are dealing with truncated files we may need to add blank
    ## planes into the data.  See cockpit bug #289.
    if np.prod(z_lengths) != packed_shape[z_idx]:
        packed_shape = list(packed_shape)
        packed_shape[z_idx] = np.prod(z_lengths)
        packed_shape = tuple(packed_shape)
        data = postpad_data(data, packed_shape)

    ## The new order for the array axes
    dim_map = dict(zip(order_in, range(len(order_in))))
    axes_order = [dim_map[i] for i in order_out]

    data = data.reshape(unpacked_shape)
    data = np.transpose(data, axes_order)
    data = data.reshape(packed_shape)
    return data


class SIExperiment(experiment.Experiment):
    """This class handles SI experiments."""

    def __init__(
        self,
        collection_order,
        bleach_compensations,
        num_angles,
        num_phases,
        angle_handler=None,
        phase_handler=None,
        polarizer_handler=None,
        slm_handler=None,
        *args,
        **kwargs,
    ):
        """Set up the structured illumination experiment.

        Each handler (angle, phase, and slm) will be used if present.

        Args:
            collection_order (str): Key from COLLECTION_ORDERS that indicates
                what order we change the angle, phase, and Z step in.
            bleach_compensations (dict): Maps light handlers to how much to
                increase their exposure times on successive angles, to
                compensate for bleaching.
            num_angles (int): How many angles to perform. Sometimes we only
                want to do 1 angle, for example.
            num_phases (int): How many phases to perform.
            angle_handler (DeviceHandler): Handler for the device that handles
                rotations of the illumination pattern.
            phase_handler (DeviceHandler): Handler for the device that handles
                phase changes in the illumination pattern.
            polarizer_handler (DeviceHandler): Handler for the polarizer.
            slm_handler (DeviceHandler): Optionally, both angle and phase can
                be handled by an SLM or similar pattern-generating device
                instead.
        """
        # Store the collection order in the MRC header.
        metadata = "SI order: %s" % collection_order
        # Store the diffraction angle in MRC metadata
        slmdev = depot.get_device_with_name("slm")
        if slmdev:
            diff_angle = slmdev.connection.get_sim_diffraction_angle()
            metadata += ": SLM diff_angle %.3f" % diff_angle
        if "metadata" in kwargs:
            # Augment the existing string.
            kwargs["metadata"] += "; %s" % metadata
        else:
            kwargs["metadata"] = metadata
        super().__init__(*args, **kwargs)
        self.num_angles = num_angles
        self.num_phases = num_phases
        self.num_z_slices = int(math.ceil(self.z_height / self.slice_height))
        if self.z_height > 1e-6:
            # Non-2D experiment; tack on an extra image to hit the top of
            # the volume.
            self.num_z_slices += 1
        self.collection_order = collection_order
        self.angle_handler = angle_handler
        self.phase_handler = phase_handler
        self.polarizer_handler = polarizer_handler
        self.slm_handler = slm_handler
        self.handler_to_bleach_compensation = bleach_compensations

    def gen_si_positions(self):
        """Generate a sequence of (angle, phase, Z) positions for SI experiments,
        based on the order the user specified."""
        ordering = COLLECTION_ORDERS[self.collection_order]
        max_vals = (self.num_angles, self.num_phases, self.num_z_slices)
        for i in range(max_vals[ordering[0]]):
            for j in range(max_vals[ordering[1]]):
                for k in range(max_vals[ordering[2]]):
                    vals = (i, j, k)
                    angle = vals[ordering.index(0)]
                    phase = vals[ordering.index(1)]
                    z = vals[ordering.index(2)]
                    yield (angle, phase, self.z_start + z * self.slice_height)

    def generate_actions(self):
        """Create the ActionTable needed to run the experiment.

        We do three Z-stacks for three different angles, and take five images
        at each Z-slice, one for each phase.
        """
        table = action_table.ActionTable()
        cur_time = 0
        prev_angle, prev_z, prev_phase = None, None, None

        # Set initial angle and phase, if relevant. We assume the SLM (if any)
        # is already showing the correct pattern for the first image set.
        # Increment the time slightly after each "motion" so that actions are well-ordered.
        if self.angle_handler is not None:
            theta = self.angle_handler.indexed_position(0)
            table.add_action(cur_time, self.angle_handler, theta)
            cur_time += decimal.Decimal("1e-6")
        if self.phase_handler is not None:
            table.add_action(cur_time, self.phase_handler, 0)
            cur_time += decimal.Decimal("1")
        table.add_action(cur_time, self.z_positioner, self.z_start)
        cur_time += decimal.Decimal("1")

        if self.slm_handler is not None:
            # Add a first trigger of the SLM to get first new image.
            table.add_action(cur_time, self.slm_handler, 0)
            # Wait a few ms for any necessary SLM triggers.
            cur_time = decimal.Decimal("5e-3")

        for angle, phase, z in self.gen_si_positions():
            delay_before_imaging = 0
            # Figure out which positions changed. They need to be held flat
            # up until the move, then spend some amount of time moving,
            # then have some time to stabilize. Or, if we have an SLM, then we
            # need to trigger it and then wait for it to stabilize.
            # Ensure we truly are doing this after all exposure events are done.
            cur_time = max(
                cur_time,
                table.get_first_and_last_action_times()[1]
                + decimal.Decimal("1e-6"),
            )
            if angle != prev_angle and prev_angle is not None:
                if self.angle_handler is not None:
                    theta = self.angle_handler.indexedPosition(angle)
                    (
                        motion_time,
                        stabilization_time,
                    ) = self.angle_handler.getMovementTime(prev_angle, theta)
                    # Move to the next position.
                    table.add_action(
                        cur_time + motion_time, self.angle_handler, theta
                    )
                    delay_before_imaging = max(
                        delay_before_imaging, motion_time + stabilization_time
                    )
                # Advance time slightly so all actions are sorted (e.g. we
                # don't try to change angle and phase in the same timestep).
                cur_time += decimal.Decimal(".001")

            if phase != prev_phase and prev_phase is not None:
                if self.phase_handler is not None:
                    (
                        motion_time,
                        stabilization_time,
                    ) = self.phase_handler.get_movement_time(prev_phase, phase)
                    # Hold flat.
                    table.add_action(cur_time, self.phase_handler, prev_phase)
                    # Move to the next position.
                    table.add_action(
                        cur_time + motion_time, self.phase_handler, phase
                    )
                    delay_before_imaging = max(
                        delay_before_imaging, motion_time + stabilization_time
                    )
                # Advance time slightly so all actions are sorted (e.g. we
                # don't try to change angle and phase in the same timestep).
                cur_time += decimal.Decimal(".001")

            if z != prev_z:
                if prev_z is not None:
                    (
                        motion_time,
                        stabilization_time,
                    ) = self.z_positioner.get_movement_time(prev_z, z)
                    # Hold flat.
                    table.add_action(cur_time, self.z_positioner, prev_z)
                    # Move to the next position.
                    table.add_action(cur_time + motion_time, self.z_positioner, z)
                    delay_before_imaging = max(
                        delay_before_imaging, motion_time + stabilization_time
                    )
                # Advance time slightly so all actions are sorted (e.g. we
                # don't try to change angle and phase in the same timestep).
                cur_time += decimal.Decimal(".001")

            prev_angle = angle
            prev_phase = phase
            prev_z = z

            cur_time += delay_before_imaging
            # Image the sample.
            # expose handles the SLM triggers. This may result in an additional
            # short delay before exposure, but is the best way to support SIM
            # in a series of exposures at different wavelengths, with the SIM
            # pattern optimised for each wavelength.
            for cameras, light_time_pairs in self.exposure_settings:
                cur_time = self.expose(
                    cur_time, cameras, light_time_pairs, angle, phase, table
                )

        # Hold Z, angle, and phase steady through to the end, then ramp down
        # to 0 to prep for the next experiment.
        table.add_action(cur_time, self.z_positioner, prev_z)
        motion_time, stabilization_time = self.z_positioner.get_movement_time(
            self.z_height, self.z_start
        )
        table.add_action(cur_time + motion_time, self.z_positioner, self.z_start)
        final_wait_time = motion_time + stabilization_time

        # Ramp down Z
        table.add_action(cur_time + final_wait_time, self.z_positioner, self.z_start)

        if self.angle_handler is not None:
            # Ramp down angle
            theta = self.angle_handler.indexedPosition(0)
            motion_time, stabilization_time = self.angle_handler.get_movement_time(
                prev_angle, theta
            )
            table.add_action(cur_time + motion_time, self.angle_handler, theta)
            final_wait_time = max(final_wait_time, motion_time + stabilization_time)
        if self.phase_handler is not None:
            # Ramp down phase
            table.add_action(cur_time, self.phase_handler, prev_phase)
            motion_time, stabilization_time = self.phase_handler.get_movement_time(
                prev_phase, 0
            )
            table.add_action(cur_time + motion_time, self.phase_handler, 0)
            final_wait_time = max(final_wait_time, motion_time + stabilization_time)
        if self.polarizer_handler is not None:
            # Return to idle voltage.
            table.add_action(cur_time, self.polarizer_handler, (0, "default"))
            final_wait_time = final_wait_time + decimal.Decimal(1e-6)

        # Set SLM back to 0th image ready for next measurement in timelapse or multi-site.
        if self.slm_handler is not None:
            # Toggle the slm_handler's digital line handler to advance one frame.
            table.add_toggle(cur_time, self.slm_handler)

        return table

    def expose(self, cur_time, cameras, light_time_pairs, angle, phase, table):
        """Wrapper around Experiment.expose() that: 1: adjusts exposure times
        based on the current angle, to compensate for bleaching; 2: uses an SLM
        (if available) to optimise SIM for each exposure."""
        # new light_time_pairs with exposure times adjusted for bleaching.
        new_pairs = []
        # If a SIM pattern puts the 1st-order spots for a given wavelength at
        # the edge of the back pupil, the 1st-order spots from longer wave-
        # lengths will fall beyond the edge of the pupil. Therefore, we use the
        # longest wavelength in a given exposure to determine the SIM pattern.
        wavelengths = []
        # Using t_exp rather than 'time' to avoid confusion between table event
        # times and exposure durations.
        for light, t_exp in light_time_pairs:
            # SIM wavelength
            wavelengths.append(light.wavelength)
            # Bleaching compensation
            t_exp_new = t_exp * (
                1
                + decimal.Decimal(self.handler_to_bleach_compensation[light])
                * angle
            )
            new_pairs.append((light, t_exp_new))
        longest_wavelength = max(wavelengths)
        # Pre-exposure delay due to polarizer and SLM settling times.
        delay = decimal.Decimal(0.0)
        # Set polarizer position
        if self.polarizer_handler is not None:
            pos = self.polarizer_handler.indexed_position(
                angle, longest_wavelength
            )
            ## Need to check last position to calculate move time. This
            # is currently only tracked outside this function, so we need to inspect
            # the table.
            # FIXME maybe: are there better ways to do this? e.g.
            # i)   track state in some attribute on Experiment
            # ii)  pass state between to this function and back again;
            # iii) have handlers track state as table is built.
            last_pos = table.get_last_action_for(self.polarizer_handler)[1]
            if last_pos is None:
                last_pos = 0
            table.add_action(
                cur_time, self.polarizer_handler, (angle, longest_wavelength)
            )
            dt = decimal.Decimal(
                sum(self.polarizer_handler.get_movement_time(last_pos, pos))
            )
            delay = max(delay, dt)
        # SLM trigger
        if self.slm_handler is not None:
            ## Add SLM event ot set pattern for phase, angle and longest_wavelength.
            table.add_action(
                cur_time, self.slm_handler, (angle, phase, longest_wavelength)
            )
            delay = max(delay, self.slm_handler.get_movement_time())
        cur_time += delay
        return super().expose(cur_time, cameras, new_pairs, table)

    def reorder_img_file(self):
        """Reorder the Z dimension in the file.

        Priism and Softworx are only capable to handle five dimensions
        so angle and phase get mixed in the Z dimension.  In addition,
        their reconstruction programs are only capable to handle them
        in angle-z-phase order.
        """
        z_order = collection_order_tuple(self.collection_order)
        z_wanted = ("a", "z", "p")
        if z_order == z_wanted:
            # Already in order; don't do anything.
            return

        doc = cockpit.util.datadoc.DataDoc(self.save_path)
        order_in = tuple(doc.image.Mrc.axis_order_str())

        length_getters = {
            "a": self.num_angles,
            "z": self.num_z_slices,
            "p": self.num_phases,
        }
        ## If data is truncated, the number of Z planes in the header
        ## may differ from the number of Z planes in the data.  So
        ## these are checked separetely.  See cockpit bug #289.
        header_z_lengths = tuple([length_getters[d] for d in z_order])
        nz_in_header = np.prod(header_z_lengths)
        nz_in_data = doc.image.shape[doc.image.Mrc.axis_order_str().index("z")]
        if nz_in_data == nz_in_header:
            z_lengths = header_z_lengths
        else:
            z_lengths = cockpit.util.mrc.adjusted_data_shape(
                nz_in_data, header_z_lengths
            )

        ## Read the extended header again separately.  It our case,
        ## this is a struct made of int32 and float32, one per plane.
        ## Its order also needs to be corrected.  We just fake a dtype
        ## with the right number of bytes per plane for the extended
        ## header.  We fake the struct by using multiple u1 per
        ## element.
        ext_header_stride = 4 * (
                doc.image_header.NumIntegers + doc.image_header.NumFloats
        )
        ext_header_dtype = ",".join(["u1"] * ext_header_stride)
        with open(self.save_path, "rb") as fh:
            fh.seek(1024)  # skip base header
            ext_header = np.fromfile(
                fh, count=doc.get_n_planes(), dtype=ext_header_dtype
            )
        assert (
                doc.image_header.next == ext_header.nbytes
        ), "next value from datadoc differs from computed length"
        assert order_in[-2:] == (
            "y",
            "x",
        ), "ORDER_IN two last dimensions are not Y and X"

        ## The file may be truncated if the experiment was aborted.
        ## In that case, the number of planes in file may be different
        ## from what is in the header so we get the shape header and
        ## not the shape from the data in the datadoc instance.  See
        ## cockpit bug #289.
        header_shape = [
            int(n) for n in cockpit.util.mrc.shape_from_hdr(doc.image_header)
        ]
        ext_header = ext_header.reshape(header_shape[:-2])

        ## Finally, reorder the things.
        img_data = reorder_z_dim(
            doc.image, order_in, z_lengths, z_order, z_wanted
        )
        ext_header = reorder_z_dim(
            ext_header, order_in[:-2], header_z_lengths, z_order, z_wanted
        )

        ## Build a new header from old doc data

        ## Save to a new file, reusing the original base header since
        ## we don't actually changed anything there.
        tmp_fh = tempfile.NamedTemporaryFile(delete=False)
        cockpit.util.datadoc.write_mrc_header(doc.image_header, tmp_fh)
        ## Make sure the data is actually ordered for writing,
        ## otherwise at this point the arrays might be making use of
        ## views.
        ext_header = np.ascontiguousarray(ext_header)
        img_data = np.ascontiguousarray(img_data)
        tmp_fh.write(ext_header)
        tmp_fh.write(img_data)
        tmp_fh.close()

        ## We are going to swap the files now, so destroy the old
        ## datadoc which memmaps the old file or we won't be able to
        ## overwrite.
        del doc
        del img_data

        # Copy permissions to new file
        shutil.copymode(self.save_path, tmp_fh.name)

        ## Windows needs to have the file removed first.
        if os.name == "nt":
            os.remove(self.save_path)
        shutil.move(tmp_fh.name, self.save_path)
        return

    def cleanup(self, run_thread=None, save_thread=None):
        super().cleanup(run_thread, save_thread)
        if self.save_path:
            self.reorder_img_file()
        return

    def last_minute_actions(self):
        if self.slice_height != 0.125:
            warning = (
                "Slice height must be 0.125 for softWoRx 3D "
                "reconstruction. Choose:"
                "\n    'OK' to run as is;"
                "\n    'Cancel' to go back and change parameters."
            )
            if not gui_utils.get_user_permission(warning):
                return False
        return True


## A consistent name to use to refer to the class itself.
EXPERIMENT_CLASS = SIExperiment


class BaseSIMExperimentUI(wx.Panel):
    """Base Experiment UI for SIM experiments.

    Generate the UI for special parameters used by this experiment.

    Subclasses must implement class property `_CONFIG_KEY_SUFFIX`.
    """

    def __init__(self, parent, config_key):
        super().__init__(parent=parent)

        self.config_key = config_key + self._CONFIG_KEY_SUFFIX
        self.all_lights = depot.get_handlers_of_type(depot.LIGHT_TOGGLE)
        self.settings = self.load_settings()

        sizer = wx.BoxSizer(wx.VERTICAL)
        row_sizer = wx.BoxSizer(wx.HORIZONTAL)

        text = wx.StaticText(self, -1, "Exposure bleach compensation (%):")
        row_sizer.Add(text, 0, wx.ALL, 5)
        ## Ordered list of bleach compensation percentages.
        self.bleach_compensations, sub_sizer = gui_utils.make_lights_controls(
            self,
            [str(l.name) for l in self.all_lights],
            self.settings["bleach_compensations"],
        )
        row_sizer.Add(sub_sizer)
        sizer.Add(row_sizer)
        # Now a row for the collection order.
        row_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.si_collection_order = gui_utils.add_labeled_input(
            self,
            row_sizer,
            label="Collection order",
            control=wx.Choice(self, choices=sorted(COLLECTION_ORDERS.keys())),
            helper_string='What order to change the angle, phase, and Z step of the experiment. E.g. for "Angle, Phase, Z" Angle will change most slowly and Z will change fastest.',
        )
        self.si_collection_order.SetSelection(self.settings["si_collection_order"])
        sizer.Add(row_sizer)
        self.SetSizerAndFit(sizer)

    def augment_params(self, params):
        """Given a parameters dict (parameter name to value) to hand to the
        experiment instance, augment them with our special parameters."""
        self.save_settings()
        params["num_angles"] = 3
        params["num_phases"] = 5
        params["collection_order"] = self.si_collection_order.GetStringSelection()
        params["angle_handler"] = depot.get_handler_with_name("SI angle")
        params["phase_handler"] = depot.get_handler_with_name("SI phase")
        params["polarizer_handler"] = depot.get_handler_with_name("SI polarizer")
        params["slm_handler"] = depot.get_handler("slm", depot.EXECUTOR)
        compensations = {}
        for i, light in enumerate(self.all_lights):
            val = gui_utils.try_parse_num(self.bleach_compensations[i], float)
            if val:
                # Convert from percentage to multiplier
                compensations[light] = 0.01 * float(val)
            else:
                compensations[light] = 0
        params["bleach_compensations"] = compensations
        return params

    def _get_default_settings(self):
        all_lights = depot.get_handlers_of_type(depot.LIGHT_TOGGLE)
        default = {
            "bleach_compensations": ["" for l in self.all_lights],
            "si_collection_order": 0,
        }
        return default

    def load_settings(self):
        """Load the saved experiment settings, if any."""
        result = cockpit.util.user_config.get_value(
            self.config_key, default=self._get_default_settings()
        )

        all_lights = depot.get_handlers_of_type(depot.LIGHT_TOGGLE)
        if len(result["bleach_compensations"]) != len(self.all_lights):
            # Number of light sources has changed; invalidate the config.
            result["bleach_compensations"] = ["" for light in self.all_lights]
        return result

    def get_settings_dict(self):
        """Generate a dict of our settings."""
        return {
            "bleach_compensations": [
                c.GetValue() for c in self.bleach_compensations
            ],
            "si_collection_order": self.si_collection_order.GetSelection(),
        }

    def save_settings(self, settings=None):
        """Save the current experiment settings to config."""
        if settings is None:
            settings = self.get_settings_dict()
        cockpit.util.user_config.set_value(self.config_key, settings)


class ExperimentUI(BaseSIMExperimentUI):
    _CONFIG_KEY_SUFFIX = "SIExperimentSettings"

    def __init__(self, parent, config_key):
        super().__init__(parent, config_key)

        self.should_only_do_one_angle = wx.CheckBox(
            self, label="Do only one angle"
        )
        self.should_only_do_one_angle.SetValue(
            self.settings["should_only_do_one_angle"]
        )

        top_row_sizer = self.Sizer.GetItem(0).Sizer
        top_row_sizer.Prepend(self.should_only_do_one_angle, 0, wx.ALL, 5)
        self.Sizer.SetSizeHints(self)

    def augment_params(self, params):
        params = super().augment_params(params)
        if self.should_only_do_one_angle.GetValue():
            params["num_angles"] = 1
        return params

    def _get_default_settings(self):
        default = super()._get_default_settings()
        default.update(
            {
                "should_only_do_one_angle": False,
            }
        )
        return default

    def get_settings_dict(self):
        all_settings = super().get_settings_dict()
        all_settings.update(
            {
                "should_only_do_one_angle": self.should_only_do_one_angle.GetValue(),
            }
        )
        return all_settings
