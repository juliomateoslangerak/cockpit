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

import logging
import traceback

import wx
from OpenGL.GL import *

import cockpit.interfaces.stage_mover
import cockpit.util.user_config
from cockpit import events
from cockpit.gui.macro_stage import macro_stage_base


_logger = logging.getLogger(__name__)


## Width of an altitude line.
HEIGHT_LINE_WIDTH = 3

## Distance in microns to include beneath the Z safety min in the histogram
HISTOGRAM_MIN_PADDING = 25
## Thickness of each line in the histogram
HISTOGRAM_LINE_WIDTH = 3
## Size of buckets in microns to use when generating altitude histogram
ALTITUDE_BUCKET_SIZE = 3

## Amount, in microns, of padding to add on either end of the mini-histogram
MINI_HISTOGRAM_PADDING = 1

# Size of secondary histogram if no fine motion stage in microns
SECONDARY_HISTOGRAM_SIZE = 50


class Histogram:
    r"""This is a simple container class for histogram display info.
    \todo Refactor histogram drawing logic into this class."""

    def __init__(
        self,
        min_altitude,
        max_altitude,
        x_offset,
        min_y,
        max_y,
        width,
        should_label,
        margin,
        data=None,
    ):
        """Altitude in microns below which we do not display."""
        self.min_altitude = min_altitude
        ## Altitude in microns above which we do not display
        self.max_altitude = max_altitude
        ## Horizontal position, as distance from the left edge
        # of the canvas
        self.x_offset = x_offset
        ## Minimum Y position of the canvas (lower edge).
        self.min_y = min_y
        ## Maximum Y position of the canvas (upper edge).
        self.max_y = max_y
        ## Maximum displayed width of any one bucket.
        self.width = width
        ## Whether or not altitudes should be labeled
        self.should_label = should_label
        ## Vertical padding on top and bottom
        self.margin = margin

        ## Size of the largest bucket (most frequent
        # data point) in our range.
        self.max_bucket_size = 1
        if data:
            for y in range(
                int(self.min_altitude),
                    int(self.max_altitude) + 1,
                ALTITUDE_BUCKET_SIZE,
            ):
                slot = int(y - self.min_altitude) // ALTITUDE_BUCKET_SIZE
                # MAP - this can crash here when Z-range is small, so bounds-check slot.
                if slot < len(data):
                    self.max_bucket_size = max(self.max_bucket_size, data[slot])

    def scale(self, altitude):
        """Rescale an altitude to our min and max, so that our min maps to
        self.min_y and our max to self.max_y, modulo our margin."""
        altitude = float(altitude - self.min_altitude) / (
                self.max_altitude - self.min_altitude
        )
        altitude = (
            altitude * (self.max_y - self.min_y - self.margin * 2)
            + self.min_y
            + self.margin
        )
        return altitude


class MacroStageZ(macro_stage_base.MacroStageBase):
    """This class shows a high-level view of where the stage is in Z space.

    It includes the current stage position, hard and soft motion limits, and Z
    tower position information.
    """

    def __init__(self, parent, *args, **kwargs):
        """Instantiate the MacroStageZ."""
        super().__init__(parent, *args, **kwargs)
        ## Backlink to parent for accessing one of its datastructures
        self.parent = parent
        ## Previous value of the Z safety min; when it changes we have to redo
        # our histograms.
        self.prev_z_safety = None

        min_z, max_z = cockpit.interfaces.stage_mover.get_hard_limits_for_axis(2)
        ## Total size of the stage's range of motion.
        self.stage_extent = max_z - min_z
        ## Vertical size of the canvas in microns -- slightly larger than the
        # stage's range of motion.
        # Note that since our width has no direct meaning, we'll just make it
        # the same as our height, and do everything proportionally.
        self.min_y = min_z - self.stage_extent * 0.05
        self.max_y = max_z + self.stage_extent * 0.05
        self.min_x = self.min_y
        self.max_x = self.max_y
        ## Amount of vertical space to allot to one line of text.
        self.text_line_height = self.stage_extent * 0.05
        ## Size of text to draw. Magic numbers ahoy.
        self.text_size = 0.005
        ## Horizontal position of the main Z position indicator
        self.z_horiz_offset = self.max_x - self.stage_extent * 0.25
        ## Horizontal length of a "marker line" indicating the position of
        # something on the Z scale.
        self.horiz_line_length = self.stage_extent * 0.1

        self.step_size = cockpit.interfaces.stage_mover.get_cur_step_sizes()[2]

        ## List of altitudes at which experiments have occurred
        self.experiment_altitudes = []
        ## List of histograms for drawing: one zoomed-out, one zoomed-in.
        self.histograms = []
        ## Dummy histogram that matches the range for the Z macro stage
        self.dummy_histogram = Histogram(
            self.min_y,
            self.max_y,
            self.z_horiz_offset,
            self.min_y,
            self.max_y,
            0,
            False,
            0,
        )

        self.calculate_histogram()

        self.Bind(wx.EVT_MOUSE_EVENTS, self.on_mouse)
        events.subscribe(events.EXPERIMENT_COMPLETE, self.on_experiment_complete)
        events.subscribe(events.STAGE_TOP_BOTTOM, self.Refresh)
        events.subscribe(events.SOFT_SAFETY_LIMIT, self.on_safety_change)
        events.subscribe(events.STAGE_STEP_SIZE, self.on_step_size_change)
        self.SetToolTip(wx.ToolTip("Double-click to move in Z"))

    def calculate_histogram(self):
        """Calculate the histogram buckets and min/max settings based on
        self.experiment_altitudes."""
        self.experiment_altitudes = list(
            cockpit.util.user_config.get_value("experiment_altitudes", default=[])
        )
        ## Set of buckets, by altitude, of the experiments
        self.altitude_buckets = [
            0
            for i in range(
                int(self.min_y), int(self.max_y + 1), ALTITUDE_BUCKET_SIZE
            )
        ]
        for altitude in self.experiment_altitudes:
            slot = int((altitude - self.min_y) // ALTITUDE_BUCKET_SIZE)
            if slot < 0 or slot > len(self.altitude_buckets):
                # This should, of course, be impossible.
                _logger.warning(
                    "Impossible experiment altitude %f (min %f, max %f)",
                    altitude,
                    self.min_y,
                    self.max_y,
                )
            else:
                # bounds check slot
                if slot < len(self.altitude_buckets):
                    self.altitude_buckets[slot] += 1

    def on_experiment_complete(self, *args):
        """Handle a new experiment completing -- requires us to update our
        histogram."""
        # Update histogram data
        self.experiment_altitudes = list(
            cockpit.util.user_config.get_value("experiment_altitudes", default=[])
        )
        self.experiment_altitudes.append(float(self.cur_stage_position[2]))
        cockpit.util.user_config.set_value(
            "experiment_altitudes", self.experiment_altitudes
        )
        self.calculate_histogram()

    def on_safety_change(self, axis, position, is_max):
        """Handle a soft safety limit being changed."""
        if axis != 2 or is_max:
            # We don't care about anything other than the Z safety min.
            return
        if self.prev_z_safety is None or self.prev_z_safety != position:
            # Update primary histogram display settings
            self.prev_z_safety = position

    def on_step_size_change(self, axis: int, newSize: float) -> None:
        r"""Step sizes have changed, which means we get to redraw.
        \todo Redrawing *everything* at this stage seems a trifle excessive."""
        if axis == 2 and self.step_size != newSize:
            self.step_size = newSize
            self.should_force_redraw = True

    def make_big_histogram(self, altitude):
        """Generate the larger of the two histograms."""
        minor_limits = cockpit.interfaces.stage_mover.get_individual_soft_limits(2)
        # Add the max range of motion of the first fine-motion controller.
        # And subtract the lower limit if minor controller exists.
        if len(minor_limits) > 1:
            minor_pos = cockpit.interfaces.stage_mover.get_all_positions()[1][2]
            histogram_min = (
                altitude
                - (minor_pos - minor_limits[1][0])
                - HISTOGRAM_MIN_PADDING
            )
            histogram_max = (
                altitude
                + (-minor_pos + minor_limits[1][1])
                + HISTOGRAM_MIN_PADDING
            )
        else:
            histogram_min = (
                altitude
                - (SECONDARY_HISTOGRAM_SIZE / 2.0)
                - HISTOGRAM_MIN_PADDING
            )
            histogram_max = (
                altitude
                + (SECONDARY_HISTOGRAM_SIZE / 2.0)
                + HISTOGRAM_MIN_PADDING
            )
        histogram = Histogram(
            histogram_min,
            histogram_max,
            self.z_horiz_offset - self.stage_extent * 0.4,
            self.min_y,
            self.max_y,
            self.stage_extent * 0.2,
            True,
            self.stage_extent * 0.05,
            self.altitude_buckets,
        )
        if not self.histograms:
            self.histograms.append(histogram)
        else:
            self.histograms[0] = histogram
        wx.CallAfter(self.Refresh)

    def on_motion(self, axis, position):
        """Overrides the parent function, since we may need to also generate a
        histogram based on the new Z position."""
        if axis != 2:
            # We only care about the Z axis.
            return
        super().on_motion(axis, position)
        # Ensure there's a histogram to work with based around current pos.
        self.make_big_histogram(cockpit.interfaces.stage_mover.get_position()[2])
        if self.should_draw:
            try:
                motor_pos = self.cur_stage_position[2]
                if motor_pos != self.prev_stage_position[2]:
                    # Check if we need to draw a new mini histogram (that
                    # scrolls with the stage's Z position).
                    z_min = motor_pos - 25
                    z_max = motor_pos + 25
                    if z_max < self.histograms[0].max_altitude:
                        # stage has entered into the area covered by the
                        # histogram, so show a zoomed-in version.
                        histogram = Histogram(
                            z_min - MINI_HISTOGRAM_PADDING,
                            z_max + MINI_HISTOGRAM_PADDING,
                            self.z_horiz_offset - self.stage_extent * 0.8,
                            self.min_y,
                            self.max_y,
                            self.stage_extent * 0.2,
                            False,
                            self.stage_extent * 0.05,
                            self.altitude_buckets,
                        )
                        # \todo There must surely be a better way to do this.
                        if len(self.histograms) == 2:
                            self.histograms[1] = histogram
                        else:
                            self.histograms.append(histogram)
                    elif len(self.histograms) > 1:
                        # Don't show the mini histogram since it's not in-range
                        self.histograms = self.histograms[:-1]

            except Exception as e:
                _logger.error("Error updating macro stage Z status: %s", e)
                _logger.error(traceback.format_exc())
                self.should_draw = False

    def on_paint(self, event=None):
        """Draw the following:
        - A red line at the current stage height
        - A purple line at the current Z tower height (or at a ceiling height
          if its position is off the scale).
        - A pair of dotted blue lines for the hard stage motion limits at
          [4000, 25000]
        - A dotted green line for the soft stage motion min limit
          (no max limit)
        - A red line indicating the stage motion delta
        - When moving, a blue arrow indicating our direction of motion
        - A scale indicator."""
        if not self.should_draw:
            return
        try:
            if not self.have_inited_gl:
                self.init_gl()
                self.have_inited_gl = True

            dc = wx.PaintDC(self)
            self.SetCurrent(self.context)
            width, height = self.GetClientSize() * self.GetContentScaleFactor()

            glViewport(0, 0, width, height)

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

            motor_pos = self.cur_stage_position[2]
            major_pos = cockpit.interfaces.stage_mover.get_all_positions()[0][2]
            minor_limits = (
                cockpit.interfaces.stage_mover.get_individual_soft_limits(2)
            )
            # Add the max range of motion of the first fine-motion controller.
            # And subtract the lower limit
            if len(minor_limits) > 1:
                minor_pos = cockpit.interfaces.stage_mover.get_all_positions()[1][
                    2
                ]
                z_max = major_pos + minor_limits[1][1]
                z_min = major_pos - (minor_pos - minor_limits[1][0])
            else:
                z_max = major_pos + (SECONDARY_HISTOGRAM_SIZE / 2.0)
                z_max = major_pos - (SECONDARY_HISTOGRAM_SIZE / 2.0)
            # Add the max range of motion of the first fine-motion controller.
            if len(minor_limits) > 1:
                z_max = major_pos + minor_limits[1][1]

            # Draw histograms. We do this first so that other lines can be drawn
            # on top.
            self.draw_histograms()

            # Draw scale bar
            glColor3f(0, 0, 0)
            glLineWidth(1)
            glBegin(GL_LINES)
            min_y, max_y = cockpit.interfaces.stage_mover.get_hard_limits_for_axis(2)
            scale_x = self.z_horiz_offset
            self.scaled_vertex(scale_x, min_y)
            self.scaled_vertex(scale_x, max_y)
            # Draw notches in the scale bar, one every 1mm.
            for scale_y in range(int(min_y), int(max_y) + 1000, 1000):
                width = self.stage_extent * 0.025
                if scale_y % 5000 == 0:
                    width = self.stage_extent * 0.05
                x1 = scale_x - width / 2
                x2 = scale_x + width / 2
                self.scaled_vertex(x1, scale_y)
                self.scaled_vertex(x2, scale_y)
            glEnd()
            glLineWidth(1)

            glLineWidth(HEIGHT_LINE_WIDTH)

            # Draw spikes for the histogram peaks.
            config = wx.GetApp().config
            spike_height = self.stage_extent * 0.02
            spike_length = self.stage_extent * 0.2
            for altitude in (
                config["stage"].getfloat("slideAltitude"),
                config["stage"].getfloat("dishAltitude"),
            ):
                glColor3f(0, 0, 0)
                glBegin(GL_POLYGON)
                self.scaled_vertex(scale_x, altitude - spike_height / 2)
                self.scaled_vertex(scale_x - spike_length, altitude)
                self.scaled_vertex(scale_x, altitude + spike_height / 2)
                glEnd()

            # Draw top and bottom positions of stack in blue.
            self.stack_def = [
                cockpit.interfaces.stage_mover.mover.saved_top,
                cockpit.interfaces.stage_mover.mover.saved_bottom,
            ]
            for pos in self.stack_def:
                if pos is not None:
                    self.draw_line(pos, color=(0, 0, 1))

            # Draw current stage position
            self.draw_line(
                motor_pos,
                color=(1, 0, 0),
                label=str(int(motor_pos)),
                is_label_on_left=True,
            )

            # Draw hard stage motion limits
            self.draw_line(
                min_y, stipple=0xAAAA, color=(0, 0, 1), label="%d" % min_y
            )
            self.draw_line(
                max_y, stipple=0xAAAA, color=(0, 0, 1), label="%d" % max_y
            )

            # Draw soft stage motion limit
            if self.prev_z_safety is not None:
                self.draw_line(
                    self.prev_z_safety,
                    stipple=0x5555,
                    color=(0, 0.8, 0),
                    label=str(int(self.prev_z_safety)),
                )

            # Draw stage motion delta
            glLineWidth(1)
            glColor3f(1, 0, 0)
            glBegin(GL_LINES)
            self.scaled_vertex(
                scale_x + self.horiz_line_length / 2, motor_pos + self.step_size
            )
            self.scaled_vertex(
                scale_x + self.horiz_line_length / 2, motor_pos - self.step_size
            )
            glEnd()

            # Draw direction of stage motion, if any
            if abs(motor_pos - self.prev_stage_position[2]) > 0.01:
                delta = motor_pos - self.prev_stage_position[2]
                if abs(delta) > self._min_delta_to_display:
                    line_center_x = scale_x + self.horiz_line_length / 2
                    self.draw_arrow(
                        (line_center_x, motor_pos),
                        (0, delta),
                        (0, 0, 1),
                        self.stage_extent * 0.15,
                        self.stage_extent * 0.03,
                    )

            glFlush()
            self.SwapBuffers()
            # Set the event, so our refreshWaiter() can update
            # our stage position info.
            self.draw_event.set()
        except Exception as e:
            _logger.error("Error drawing Z macro stage: %s", e)
            traceback.print_exc()
            self.should_draw = False

    def draw_histograms(self):
        """Draw all our histograms."""
        prev_histogram = self.dummy_histogram
        min_y, max_y = cockpit.interfaces.stage_mover.get_hard_limits_for_axis(2)
        width, height = self.GetClientSize()
        for histogram in self.histograms:
            glColor3f(0, 0, 0)
            glLineWidth(HISTOGRAM_LINE_WIDTH)
            glBegin(GL_LINES)
            for pixelOffset in range(0, height):
                # Convert pixel offset to altitude inside our histogram
                # min/max values
                altitude = float(pixelOffset) / height
                altitude = (
                    altitude * (histogram.max_altitude - histogram.min_altitude)
                    + histogram.min_altitude
                )
                # Map that altitude to a bucket
                bucket_index = int(altitude - self.min_y) // ALTITUDE_BUCKET_SIZE
                if bucket_index < len(self.altitude_buckets):
                    count = self.altitude_buckets[bucket_index]
                else:
                    continue
                width = int(
                    float(count) / histogram.max_bucket_size * histogram.width
                )
                draw_altitude = histogram.scale(altitude)
                self.scaled_vertex(histogram.x_offset, draw_altitude)
                self.scaled_vertex(histogram.x_offset - width, draw_altitude)
            glEnd()
            glLineWidth(1)

            # HACK: For some reason the left edge of the histogram is jagged.
            # Cover it up.
            glBegin(GL_LINES)
            self.scaled_vertex(histogram.x_offset, min_y)
            self.scaled_vertex(histogram.x_offset, max_y)
            glEnd()

            # Draw histogram min/max
            if histogram.should_label:
                line_height = self.stage_extent * 0.05
                self.draw_text_at(
                    (
                        histogram.x_offset + self.stage_extent * 0.05,
                        min_y - self.text_line_height / 4,
                    ),
                    str(int(histogram.min_altitude)),
                    size=self.text_size,
                )
                self.draw_text_at(
                    (
                        histogram.x_offset + self.stage_extent * 0.05,
                        max_y - self.text_line_height / 4,
                    ),
                    str(int(histogram.max_altitude)),
                    size=self.text_size,
                )

            # Draw histogram stage motion delta
            motor_pos = self.cur_stage_position[2]
            glLineWidth(1)
            glBegin(GL_LINES)
            glColor3f(1, 0, 0)
            self.scaled_vertex(
                histogram.x_offset + self.stage_extent * 0.01,
                histogram.scale(motor_pos + self.step_size),
            )
            self.scaled_vertex(
                histogram.x_offset + self.stage_extent * 0.01,
                histogram.scale(motor_pos - self.step_size),
            )
            glEnd()

            # Draw lines showing how the histogram relates to the
            # less zoomed-in view to its left.
            left_bottom = prev_histogram.scale(histogram.min_altitude)
            left_top = prev_histogram.scale(histogram.max_altitude)
            glLineWidth(1)
            glColor3f(0.5, 0.5, 0.5)
            glBegin(GL_LINES)
            self.scaled_vertex(prev_histogram.x_offset, left_bottom)
            self.scaled_vertex(histogram.x_offset, min_y)
            self.scaled_vertex(prev_histogram.x_offset, left_top)
            self.scaled_vertex(histogram.x_offset, max_y)
            self.scaled_vertex(prev_histogram.x_offset, left_bottom)
            self.scaled_vertex(prev_histogram.x_offset, left_top)
            glEnd()

            prev_histogram = histogram

    def draw_line(
        self,
        altitude,
        line_length=None,
        stipple=None,
        color=(0, 0, 0),
        width=2,
        label="",
        is_label_on_left=False,
        should_label_main_view=True,
        should_label_histogram=True,
    ):
        """Draw a horizontal line at the specified altitude."""
        if line_length is None:
            line_length = self.horiz_line_length
        draw_altitude = altitude
        left_edge = self.z_horiz_offset + self.horiz_line_length / 2
        right_edge = self.z_horiz_offset - self.horiz_line_length / 2

        if stipple is not None:
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(3, stipple)
        glColor3fv(color)
        glLineWidth(width)
        glBegin(GL_LINES)
        # Draw on the main view.
        self.scaled_vertex(left_edge, draw_altitude)
        self.scaled_vertex(right_edge, draw_altitude)
        # Draw in the histograms too, if in range
        for histogram in self.histograms:
            if histogram.min_altitude <= altitude <= histogram.max_altitude:
                # This line should be visible in the histogram.
                x_offset = histogram.x_offset + line_length / 2.0
                histogram_altitude = histogram.scale(altitude)
                self.scaled_vertex(x_offset, histogram_altitude)
                self.scaled_vertex(x_offset - line_length, histogram_altitude)
        glEnd()
        if stipple is not None:
            glDisable(GL_LINE_STIPPLE)

        if label:
            glLineWidth(1)
            label_x = right_edge - self.stage_extent * 0.01
            if is_label_on_left:
                label_x = left_edge + self.stage_extent * 0.1
            if should_label_main_view:
                # Draw a text label next to the line, in the line's color.
                self.draw_text_at(
                    (label_x, draw_altitude - self.text_line_height / 4),
                    label,
                    size=self.text_size,
                    color=color,
                )
            if should_label_histogram:
                # Draw the label for the histograms too.
                for histogram in self.histograms:
                    if histogram.should_label:
                        histogram_altitude = histogram.scale(altitude)
                        histogram_label_x = (
                                label_x
                                - histogram.x_offset
                                + self.horiz_line_length / 2
                        )
                        self.draw_text_at(
                            (
                                histogram_label_x,
                                histogram_altitude - self.text_line_height / 4,
                            ),
                            label,
                            size=self.text_size,
                            color=color,
                        )

    def on_mouse(self, event):
        """Double click moves the stage in Z."""
        if event.LeftDClick():
            click_loc = event.GetPosition()
            canvas_loc = self.map_click_to_canvas(click_loc)
            # Map the click location to one of our histograms or to the main scale.
            scale = (self.min_y, self.max_y)
            # if we are on the first hist then use the coarsest mover if not
            # then use the currently selected one.
            original_mover = cockpit.interfaces.stage_mover.mover.cur_handler_index
            cockpit.interfaces.stage_mover.mover.cur_handler_index = 0

            for histogram in self.histograms:
                if canvas_loc[0] < histogram.x_offset + self.horiz_line_length:
                    scale = (histogram.min_altitude, histogram.max_altitude)
                    # not first hist so revert to current mover whatever that
                    # might be
                    cockpit.interfaces.stage_mover.mover.cur_handler_index = (
                        original_mover
                    )
                    break  # fall through as we have found which
                    # histogram we clicked on

            width, height = self.GetClientSize()
            weight = 1.0 - float(click_loc[1]) / height
            altitude = (scale[1] - scale[0]) * weight + scale[0]
            z_hard_max = cockpit.interfaces.stage_mover.get_individual_hard_limits(
                2
            )[0][1]
            cockpit.interfaces.stage_mover.go_to_z(min(z_hard_max, altitude))
            # make sure we are back to the expected mover
            cockpit.interfaces.stage_mover.mover.cur_handler_index = original_mover

    def map_click_to_canvas(self, loc):
        """Remap an XY tuple to stage coordinates."""
        width, height = self.GetClientSize()
        x = float(width - loc[0]) / width * (self.max_y - self.min_y) + self.min_y
        y = (
            float(height - loc[1]) / height * (self.max_y - self.min_y)
            + self.min_y
        )
        return (x, y)
