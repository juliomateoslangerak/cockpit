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

import logging
import traceback
import typing

import numpy
import wx
from OpenGL.GL import *

import cockpit.interfaces
import cockpit.interfaces.stage_mover
from cockpit import events
from cockpit.gui.macro_stage import macro_stage_base
from cockpit.gui.primitive import Primitive


_logger = logging.getLogger(__name__)


class _StagePositionEntryDialog(wx.Dialog):
    def __init__(self, parent: wx.Window) -> None:
        super().__init__(parent, title="Move stage")

        self._spins = []
        position = cockpit.interfaces.stage_mover.get_position()
        limits = cockpit.interfaces.stage_mover.get_soft_limits()
        for i in range(3):
            self._spins.append(
                wx.SpinCtrlDouble(
                    self,
                    min=limits[i][0],
                    max=limits[i][1],
                    initial=position[i],
                )
            )

        sizer = wx.BoxSizer(wx.VERTICAL)

        sizer.Add(
            wx.StaticText(self, label="Select stage position"),
            wx.SizerFlags().Border().Centre(),
        )

        spins_sizer = wx.FlexGridSizer(cols=2, gap=(0, 0))
        for spin, name in zip(self._spins, ["X", "Y", "Z"]):
            spins_sizer.Add(
                wx.StaticText(self, label=name),
                wx.SizerFlags().Border().Centre(),
            )
            spins_sizer.Add(spin, wx.SizerFlags().Border().Expand())
        sizer.Add(spins_sizer, wx.SizerFlags().Border().Centre())

        buttons_sizer = self.CreateStdDialogButtonSizer(wx.OK | wx.CANCEL)
        sizer.Add(buttons_sizer, wx.SizerFlags().Border().Centre())

        self.SetSizerAndFit(sizer)

    def get_value(self) -> typing.Tuple[float, float, float]:
        return [s.GetValue() for s in self._spins]


def go_to_xyz_dialog(parent: wx.Window) -> None:
    position_dialog = _StagePositionEntryDialog(parent)
    if position_dialog.ShowModal() != wx.ID_OK:
        return

    new_pos = position_dialog.get_value()
    # Work out if we will be outside the limits of the current stage
    # This should be a method of the StageMover interface (see #616)
    position = cockpit.interfaces.stage_mover.get_position()
    pos_delta = [
        new_pos[0] - position[0],
        new_pos[1] - position[1],
        new_pos[2] - position[2],
    ]
    original_handler_index = wx.GetApp().stage.cur_handler_index
    current_handler_index = original_handler_index
    all_positions = cockpit.interfaces.stage_mover.get_all_positions()
    for axis in range(3):
        if pos_delta[axis] ** 2 > 0.001:
            limits = cockpit.interfaces.stage_mover.get_individual_hard_limits(
                axis
            )
            current_pos = all_positions[current_handler_index][axis]
            if (
                # off bottom
                current_pos + pos_delta[axis]
                < (limits[current_handler_index][0])
            ) or (
                # off top
                current_pos + pos_delta[axis]
                > (limits[current_handler_index][1])
            ):
                current_handler_index -= 1  # go to a bigger handler index
            if current_handler_index < 0:
                return
    wx.GetApp().stage.cur_handler_index = current_handler_index
    cockpit.interfaces.stage_mover.go_to(new_pos)
    wx.GetApp().stage.cur_handler_index = original_handler_index


class MacroStageXY(macro_stage_base.MacroStageBase):
    """This class shows a high-level view of where the stage is in XY space,
    and how it will move when controlled by the keypad.

    It includes displays of where saved sites are, where mosaic tiles are, the
    current XY coordinates, and so on.
    """

    def __init__(self, *args, **kwargs):
        """Instantiate the object.

        Just calls the parent constructor and sets up the mouse event.
        """
        super().__init__(*args, **kwargs)
        ## Objective offset info to get correct position and limits
        self.offset = wx.GetApp().objectives.get_offset()
        ## Whether or not to draw the mosaic tiles
        self.should_draw_mosaic = True
        ## True if we're in the processing of changing the soft motion limits.
        self.am_setting_safeties = False
        ## Position the mouse first clicked when setting safeties, or None if
        # we aren't setting safeties.
        self.first_safety_mouse_pos = None
        ## Last seen mouse position
        self.last_mouse_pos = [0, 0]

        primitive_specs = (
            wx.GetApp().config["stage"].getlines("primitives", [])
        )
        self._primitives = [
            Primitive.factory(spec) for spec in primitive_specs
        ]

        hard_limits = cockpit.interfaces.stage_mover.get_hard_limits()
        self.min_x, self.max_x = hard_limits[0]
        self.min_y, self.max_y = hard_limits[1]
        ## X extent of the stage, in microns.
        stage_width = self.max_x - self.min_x
        ## Y extent of the stage, in microns.
        stage_height = self.max_y - self.min_y
        ## Max of X or Y stage extents.
        self.max_extent = max(stage_width, stage_height)
        ## X and Y view extent.
        if stage_height > stage_width:
            self.view_extent = 1.2 * stage_height
            self.view_delta_y = stage_height * 0.1
        else:
            self.view_extent = 1.05 * stage_width
            self.view_delta_y = stage_height * 0.05
        # Push out the min and max values a bit to give us some room around
        # the stage to work with. In particular we need space below the display
        # to show our legend.
        self.centre_x = ((self.max_x - self.min_x) / 2) + self.min_x
        self.centre_y = ((self.max_y - self.min_y) / 2) + self.min_y
        self.min_x = self.centre_x - self.view_extent / 2
        self.max_x = self.centre_x + self.view_extent / 2
        self.min_y = self.centre_y - self.view_extent / 2 - self.view_delta_y
        self.max_y = self.centre_y + self.view_extent / 2 - self.view_delta_y

        ## Size of text to draw. I confess I don't really understand how this
        # corresponds to anything, but it seems to work out.
        self.text_size = 0.004

        self.Bind(wx.EVT_MOTION, self.on_mouse_motion)
        self.Bind(wx.EVT_LEFT_UP, self.on_left_click)
        self.Bind(wx.EVT_LEFT_DCLICK, self.on_left_double_click)
        self.Bind(wx.EVT_RIGHT_UP, self.on_right_click)
        self.Bind(wx.EVT_RIGHT_DCLICK, self.on_right_double_click)
        # Bind context menu event to None to prevent main window context menu
        # being displayed in preference to our own.
        self.Bind(wx.EVT_CONTEXT_MENU, lambda event: None)
        events.subscribe(events.SOFT_SAFETY_LIMIT, self.on_safety_change)
        self.SetToolTip(
            wx.ToolTip(
                "Left double-click to move the stage. "
                + "Right click for gotoXYZ and double-click to toggle displaying of mosaic "
                + "tiles."
            )
        )

        wx.GetApp().objectives.Bind(
            cockpit.interfaces.EVT_OBJECTIVE_CHANGED,
            self._on_objective_changed,
        )

        # many stages have an external control that cockpit knows nothing about
        # eg an xy(z) joystick. So setup a wx timer to poll the position
        # and update if it changes.
        self._position_cache = [
            0.0
            for x in range(len(cockpit.interfaces.stage_mover.get_position()))
        ]
        self._timer = wx.Timer(self)
        # poll every 1 s (1000 ms)
        self._timer.Start(1000)
        self.Bind(wx.EVT_TIMER, self.on_timer)
        # bind to window destroy to stop timer
        self.Bind(wx.EVT_WINDOW_DESTROY, self.on_destroy)

    def on_timer(self, event):
        """code that the wx timer calls to check postion on a regular basisis."""
        position = cockpit.interfaces.stage_mover.get_position()
        for i, pos in enumerate(position):
            if pos != self._position_cache[i]:
                events.publish(events.STAGE_POSITION, i, pos)
                self._position_cache[i] = pos

    def on_destroy(self, event):
        self._timer.Stop()
        event.Skip()

    def on_safety_change(self, axis, value, is_max):
        r"""Safety limits have changed, which means we need to force a refresh.
        \todo Redrawing everything just to tackle the safety limits is a bit excessive."""
        # We only care about the X and Y axes.
        if axis in [0, 1]:
            wx.CallAfter(self.Refresh)

    def on_paint(self, event=None):
        """Draw the canvas.

        We draw the following: - A blue dotted square representing the hard
        stage limits of [(4000, 4000), (25000, 25000)] - A green dotted square
        representing the soft stage limits - A red square centered on the
        current stage position - A red crosshairs representing the stage motion
        delta - When moving, a blue arrow indicating our direction of motion -
        A purple dot for each saved site - A black dot for each tile in the
        mosaic - Device-defined primitives, e.g. to show individual sample
        locations.
        """
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

            square_offsets = [(0, 0), (0, 1), (1, 1), (1, 0)]
            step_sizes = cockpit.interfaces.stage_mover.get_cur_step_sizes()[:2]

            # Draw hard stage motion limits
            hard_limits = cockpit.interfaces.stage_mover.get_hard_limits()[:2]
            # Rearrange limits to (x, y) tuples.
            hard_limits = list(zip(hard_limits[0], hard_limits[1]))
            stage_height = abs(hard_limits[1][0] - hard_limits[1][1])

            # Set up transform from stage to screen units
            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
            glOrtho(self.max_x, self.min_x, self.min_y, self.max_y, -1.0, 1.0)

            # Loop over objective offsets to draw limist in multiple colours.
            for obj in wx.GetApp().objectives.get_handlers():
                offset = obj.offset
                colour = obj.colour
                glLineWidth(4)
                if obj is not wx.GetApp().objectives.get_current():
                    colour = (
                        min(1, colour[0] + 0.7),
                        min(1, colour[1] + 0.7),
                        min(1, colour[2] + 0.7),
                    )
                    glLineWidth(2)
                glEnable(GL_LINE_STIPPLE)
                glLineStipple(3, 0xAAAA)
                glColor3f(*colour)
                glBegin(GL_LINE_LOOP)
                for x_index, y_index in square_offsets:
                    glVertex2f(
                        hard_limits[x_index][0] - offset[0],
                        hard_limits[y_index][1] + offset[1],
                    )
                glEnd()
                glDisable(GL_LINE_STIPPLE)

                # Draw soft stage motion limits -- a dotted box, solid black
                # corners, and coordinates. If we're currently setting safeties,
                # then the second corner is the current mouse position.
                safeties = cockpit.interfaces.stage_mover.get_soft_limits()[:2]
                x1, x2 = safeties[0]
                y1, y2 = safeties[1]
                if self.first_safety_mouse_pos is not None:
                    x1, y1 = self.first_safety_mouse_pos
                    x2, y2 = self.last_mouse_pos
                    if x1 > x2:
                        x1, x2 = x2, x1
                    if y1 > y2:
                        y1, y2 = y2, y1
                soft_limits = [(x1, y1), (x2, y2)]

                # First the dotted green box.
                glEnable(GL_LINE_STIPPLE)
                glLineWidth(2)
                glLineStipple(3, 0x5555)
                glColor3f(0, 1, 0)
                glBegin(GL_LINE_LOOP)
                for x, y in [(x1, y1), (x1, y2), (x2, y2), (x2, y1)]:
                    glVertex2f(x - offset[0], y + offset[1])
                glEnd()
                glDisable(GL_LINE_STIPPLE)

                # Now the corners.
                glColor3f(0, 0, 0)
                glBegin(GL_LINES)
                for (vx, vy), (dx, dy) in [
                    (soft_limits[0], (self.max_extent * 0.1, 0)),
                    (soft_limits[0], (0, self.max_extent * 0.1)),
                    (soft_limits[1], (-self.max_extent * 0.1, 0)),
                    (soft_limits[1], (0, -self.max_extent * 0.1)),
                ]:
                    second_vertex = [vx + dx, vy + dy]
                    glVertex2f(vx - offset[0], vy + offset[1])
                    glVertex2f(
                        second_vertex[0] - offset[0],
                        second_vertex[1] + offset[1],
                    )
                glEnd()
                glLineWidth(1)
            # Now the coordinates. Only draw them if the soft limits aren't
            # the hard limits, to avoid clutter.
            if safeties != hard_limits:
                for i, (dx, dy) in enumerate([(4000, -700), (2000, 400)]):
                    x = soft_limits[i][0]
                    y = soft_limits[i][1]
                    self.draw_text_at(
                        (x + dx, y + dy),
                        "(%d, %d)" % (x, y),
                        size=self.text_size * 0.75,
                    )

            glDisable(GL_LINE_STIPPLE)

            # Draw device-specific primitives.
            # This uses vertex buffers with vertices in stage co-ordinates,
            # so need to update the modelview matrix to render them in the
            # right place.
            glEnable(GL_LINE_STIPPLE)
            glLineStipple(1, 0xAAAA)
            glColor3f(0.4, 0.4, 0.4)
            for primitive in self._primitives:
                primitive.render()
            glDisable(GL_LINE_STIPPLE)

            # Draw possible stage positions for current objective
            offset = wx.GetApp().objectives.get_offset()
            colour = wx.GetApp().objectives.get_colour()
            glLineWidth(2)
            # Draw stage position
            motor_pos = self.cur_stage_position[:2]
            square_size = self.max_extent * 0.025
            glColor3f(*colour)
            glBegin(GL_LINE_LOOP)
            for x, y in square_offsets:
                glVertex2f(
                    motor_pos[0] - offset[0] + square_size * x - square_size / 2,
                    motor_pos[1] + offset[1] + square_size * y - square_size / 2,
                )
            glEnd()

            # Draw motion crosshairs
            glColor3f(1, 0, 0)
            glBegin(GL_LINES)
            for i, step_size in enumerate(step_sizes):
                if step_size is None:
                    # No step control along this axis.
                    continue
                offset = [0, 0]
                offset[i] = step_size
                glVertex2f(motor_pos[0] - offset[0], motor_pos[1] - offset[1])
                glVertex2f(motor_pos[0] + offset[0], motor_pos[1] + offset[1])
            glEnd()

            # Draw direction of motion
            delta = motor_pos - self.prev_stage_position[:2]

            if sum(numpy.fabs(delta)) > self._min_delta_to_display:
                self.draw_arrow(
                    (
                        motor_pos[0] - self.offset[0],
                        motor_pos[1] + self.offset[1],
                    ),
                    delta,
                    (0, 0, 1),
                    arrow_size=self.max_extent * 0.1,
                    arrow_head_size=self.max_extent * 0.025,
                )
                glLineWidth(1)

            # The crosshairs don't always draw large enough to show,
            # so ensure that at least one pixel in the middle
            # gets drawn.
            glBegin(GL_POINTS)
            glVertex2f(
                motor_pos[0] - self.offset[0], motor_pos[1] + self.offset[1]
            )
            glEnd()

            # Draw scale bar
            glColor3f(0, 0, 0)
            glLineWidth(1)
            glBegin(GL_LINES)
            y_offset = self.min_y + 0.9 * (
                    self.view_delta_y + 0.5 * (self.view_extent - stage_height)
            )
            glVertex2f(hard_limits[0][0], y_offset)
            glVertex2f(hard_limits[0][1], y_offset)
            # Draw notches in the scale bar every 1mm.
            for scale_x in range(
                int(hard_limits[0][0]), int(hard_limits[0][1]) + 1000, 1000
            ):
                width = self.view_extent * 0.015
                if scale_x % 5000 == 0:
                    width = self.view_extent * 0.025
                y1 = y_offset - width / 2
                y2 = y_offset + width / 2
                glVertex2f(scale_x, y1)
                glVertex2f(scale_x, y2)
            glEnd()
            glLineWidth(1)

            events.publish(events.MACRO_STAGE_XY_DRAW, self)

            glFlush()
            self.SwapBuffers()
            # Set the event, so our refreshWaiter() can update
            # our stage position info.
            self.draw_event.set()
        except Exception as e:
            _logger.error("Exception drawing XY macro stage: %s", e)
            _logger.error(traceback.format_exc())
            self.should_draw = False

    def set_xy_limit(self, pos=None):
        """Set one part of the stage motion limits."""
        if pos is None:
            # Use current stage position
            pos = self.cur_stage_position
        if self.first_safety_mouse_pos is None:
            # This is the first click for setting safeties.
            self.first_safety_mouse_pos = [pos[0], pos[1]]
        else:
            # Set the second corner, then ensure that the "min" and "max"
            # values are in the right order.
            x1, y1 = self.first_safety_mouse_pos
            x2, y2 = pos
            if x1 > x2:
                x1, x2 = x2, x1
            if y1 > y2:
                y1, y2 = y2, y1
            cockpit.interfaces.stage_mover.set_soft_min(0, x1)
            cockpit.interfaces.stage_mover.set_soft_max(0, x2)
            # Add 1 to prevent rounding issues relative to current position.
            cockpit.interfaces.stage_mover.set_soft_min(1, y1 + 1)
            cockpit.interfaces.stage_mover.set_soft_max(1, y2 + 1)
            self.am_setting_safeties = False
            self.first_safety_mouse_pos = None
            self.Refresh()

    def on_mouse_motion(self, event):
        """Moved the mouse.

        Record its position
        """
        if self.am_setting_safeties and self.first_safety_mouse_pos:
            # Need to redraw to show the new safeties.
            self.last_mouse_pos = self.remap_click(event.GetPosition())
            self.Refresh()

    def on_left_click(self, event):
        """Clicked the left mouse button.

        Set safeties if we're in that mode.
        """
        if self.am_setting_safeties:
            safe_loc = self.remap_click(event.GetPosition())
            self.set_xy_limit(safe_loc)

    def on_left_double_click(self, event):
        """Double-clicked the left mouse button.

        Move to the clicked location.
        """
        original_mover = cockpit.interfaces.stage_mover.mover.cur_handler_index
        # Quick hack to get deepsim working need to check if we can do it
        # properly.  Should really check to see if we can move, and by that
        # distance with existing mover
        cockpit.interfaces.stage_mover.mover.cur_handler_index = 0

        cockpit.interfaces.stage_mover.go_to_xy(
            self.remap_click(event.GetPosition())
        )

        # make sure we are back to the expected mover
        cockpit.interfaces.stage_mover.mover.cur_handler_index = original_mover

    def on_right_click(self, event) -> None:
        del event
        go_to_xyz_dialog(self.GetParent())

    def on_right_double_click(self, event):
        """Right-clicked the mouse.

        Toggle drawing of the mosaic tiles
        """
        self.should_draw_mosaic = not self.should_draw_mosaic

    def remap_click(self, click_location):
        """Remap a click location from pixel coordinates to realspace coordinates."""
        width, height = self.GetClientSize()
        x = (
                float(width - click_location[0]) / width * (self.max_x - self.min_x)
                + self.min_x
        )
        y = (
                float(height - click_location[1]) / height * (self.max_y - self.min_y)
                + self.min_y
        )
        return [x + self.offset[0], y - self.offset[1]]

    def set_safeties(self, event=None):
        """Switch mode so that clicking sets the safeties."""
        self.am_setting_safeties = True

    def _on_objective_changed(self, event: wx.CommandEvent) -> None:
        """Refresh display on objective change."""
        self.offset = wx.GetApp().objectives.get_offset()
        self.Refresh()
        event.Skip()
