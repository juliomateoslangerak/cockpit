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

"""Standard Z stack experiment."""

import decimal
import math

from cockpit.experiment import action_table, experiment


## Provided so the UI knows what to call this experiment.
EXPERIMENT_NAME = "Z-stack"


class ZStackExperiment(experiment.Experiment):
    """This class handles classic Z-stack experiments."""

    def generate_actions(self):
        """Create the ActionTable needed to run the experiment.

        We simply move to each Z-slice in turn, take an image, then move to the
        next.
        """
        table = action_table.ActionTable()
        cur_time = 0
        prev_altitude = None
        num_z_slices = int(math.ceil(self.z_height / self.slice_height))
        if self.z_height > 1e-6:
            # Non-2D experiment; tack on an extra image to hit the top of
            # the volume.
            num_z_slices += 1
        for z_index in range(num_z_slices):
            # Move to the next position, then wait for the stage to
            # stabilize.
            z_target = self.z_start + self.slice_height * z_index
            motion_time, stabilization_time = 0, 0
            if prev_altitude is not None:
                (
                    motion_time,
                    stabilization_time,
                ) = self.z_positioner.get_movement_time(
                    prev_altitude, z_target
                )
            cur_time += motion_time
            table.add_action(cur_time, self.z_positioner, z_target)
            cur_time += stabilization_time
            prev_altitude = z_target

            # Image the sample.
            for cameras, light_time_pairs in self.exposure_settings:
                cur_time = self.expose(
                    cur_time, cameras, light_time_pairs, table
                )
                # Advance the time very slightly so that all exposures
                # are strictly ordered.
                cur_time += decimal.Decimal("1e-10")
            # Hold the Z motion flat during the exposure.
            table.add_action(cur_time, self.z_positioner, z_target)

        # Move back to the start so we're ready for the next rep.
        motion_time, stabilization_time = self.z_positioner.get_movement_time(
            self.z_height, 0
        )
        cur_time += motion_time
        table.add_action(cur_time, self.z_positioner, self.z_start)
        # Hold flat for the stabilization time, and any time needed for
        # the cameras to be ready. Only needed if we're doing multiple
        # reps, so we can proceed immediately to the next one.
        camera_ready_time = 0
        if self.num_reps > 1:
            for cameras, light_time_pairs in self.exposure_settings:
                for camera in cameras:
                    camera_ready_time = max(
                        camera_ready_time,
                        self.get_time_when_camera_can_expose(table, camera),
                    )
        table.add_action(
            max(cur_time + stabilization_time, camera_ready_time),
            self.z_positioner,
            self.z_start,
        )

        return table


## A consistent name to use to refer to the class itself.
EXPERIMENT_CLASS = ZStackExperiment
