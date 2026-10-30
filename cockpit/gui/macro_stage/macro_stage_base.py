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

import numpy as np
import wx
from OpenGL.GL import *

import cockpit.gui
import cockpit.gui.freetype
import cockpit.interfaces.stage_mover
from cockpit import events


## @package cockpit.gui.macro_stage
# This module contains the MacroStageBase base class, used by the MacroStageXY
# and MacroStageZ classes, as well as some shared constants.

## Line thickness for the arrow
ARROW_LINE_THICKNESS = 3.5
## Bluntness of the arrowhead (pi/2 == totally blunt)
ARROWHEAD_ANGLE = np.pi / 6.0


class MacroStageBase(wx.glcanvas.GLCanvas):
    """This class handles some common code for the MacroStageXY and MacroStageZ
    classes."""

    def __init__(self, parent, size, id=-1, *args, **kwargs):
        """Create the MacroStage.

        Mostly, attach a timer to the main window so that we can use it to
        trigger updates.
        """
        super().__init__(
            parent, id, size=tuple(map(int, size)), *args, **kwargs
        )

        ## WX context for drawing.
        self.context = wx.glcanvas.GLContext(self)
        ## Whether or not we have done some one-time-only logic.
        self.have_inited_gl = False
        ## Whether or not we should try to draw
        self.should_draw = True
        ## Font for drawing text
        self.face = cockpit.gui.freetype.Face(self, 18)

        ## X values below this are off the canvas. We leave it up to children
        # to fill in proper values for these.
        self.min_x = 0
        ## X values above this are off the canvas
        self.max_x = 1000
        ## Y values below this are off the canvas
        self.min_y = 0
        ## Y values above this are off the canvas
        self.max_y = 1000

        ## Don't bother showing a movement arrow for movements smaller
        ## than this.
        self._min_delta_to_display = (
            wx.GetApp().config["stage"].getfloat("min-delta-to-display", 0.01)
        )

        ## (X, Y, Z) vector describing the stage position as of the last
        # time we drew ourselves. We need this to display motion deltas.
        self.prev_stage_position = np.zeros(3)
        ## As above, but for the current position.
        self.cur_stage_position = np.zeros(3)
        ## Event used to indicate when drawing is done, so we can update
        # the above.
        self.draw_event = threading.Event()

        ## Boolean to just force a redraw.
        self.should_force_redraw = False

        ## Thread that ensures we don't spam redisplaying ourselves.
        self.redraw_timer_thread = threading.Thread(
            target=self.refresh_waiter, name="macrostage-refresh", daemon=True
        )
        self.redraw_timer_thread.start()

        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.Bind(wx.EVT_SIZE, lambda event: event)
        self.Bind(
            wx.EVT_ERASE_BACKGROUND, lambda event: event
        )  # Do nothing, to avoid flashing
        events.subscribe(events.STAGE_POSITION, self.on_motion)
        events.subscribe(events.STAGE_STEP_INDEX, self.on_step_index_change)

    def init_gl(self):
        """Set up some set-once things for OpenGL."""
        self.SetCurrent(self.context)
        glClearColor(1.0, 1.0, 1.0, 0.0)

    def on_motion(self, axis, position):
        """Update our marker of where the stage currently is.

        This will indirectly cause us to redisplay ourselves in a bit, thanks
        to self.refreshWaiter.
        """
        self.cur_stage_position[axis] = position

    def on_step_index_change(self, index):
        r"""Step index has changed, so the highlighting on our step displays is
        different.
        \todo Redrawing *everything* at this stage seems a trifle excessive."""
        self.should_force_redraw = True

    def refresh_waiter(self):
        """Wait until a minimum amount of time has passed since our last redraw
        before we redraw again.

        Since we redraw when the stage moves and these events may happen
        rapidly, this prevents us from spamming OpenGL calls.
        """
        while True:
            if not self:
                # Our window has been deleted; we're done here.
                return
            if (
                np.any(self.cur_stage_position != self.prev_stage_position)
                or self.should_force_redraw
            ):
                self.draw_event.clear()
                wx.CallAfter(self.Refresh)
                self.draw_event.wait()
                self.prev_stage_position[:] = self.cur_stage_position
                # Draw again after a delay, so that motion arrows get
                # cleared.
                time.sleep(0.25)
                self.draw_event.clear()
                wx.CallAfter(self.Refresh)
                self.draw_event.wait()
                self.should_force_redraw = False
            time.sleep(0.1)

    def scaled_vertex(self, x, y, should_return=False):
        """Rescale the input value to be in the range (self.min[X|Y],
        self.max[X|Y]), flip the X axis, and call glVertex2f on the result.

        Or return the value instead if should_return is true. The basic idea
        here is to go from stage coordinates to view coordinates.
        """
        new_x = -1 * ((x - self.min_x) / float(self.max_x - self.min_x) * 2 - 1)
        new_y = (y - self.min_y) / float(self.max_y - self.min_y) * 2 - 1
        if should_return:
            return (new_x, new_y)
        else:
            glVertex2f(new_x, new_y)

    def draw_arrow(self, base_location, vector, color, arrow_size, arrow_head_size):
        """Draw an arrow from the first point along the specified vector."""
        # Normalize.
        delta = vector / np.sqrt(np.vdot(vector, vector)) * arrow_size
        # Calculate angle, for the head of the arrow
        angle = np.arctan2(delta[1], delta[0])

        point_location = base_location + delta
        head_location_1 = (
                point_location
                - np.array(
                [
                    np.cos(angle + ARROWHEAD_ANGLE),
                    np.sin(angle + ARROWHEAD_ANGLE),
                ]
            )
                * arrow_head_size
        )
        head_location_2 = (
                point_location
                - np.array(
                [
                    np.cos(angle - ARROWHEAD_ANGLE),
                    np.sin(angle - ARROWHEAD_ANGLE),
                ]
            )
                * arrow_head_size
        )

        # Draw
        glColor3f(color[0], color[1], color[2])
        glLineWidth(ARROW_LINE_THICKNESS)
        glBegin(GL_LINES)
        glVertex2f(base_location[0], base_location[1])
        glVertex2f(point_location[0], point_location[1])
        glEnd()
        # Prevent the end of the line from showing through the
        # arrowhead by moving the arrowhead further along.
        point_location += delta * 0.1
        glBegin(GL_POLYGON)
        glVertex2f(head_location_1[0], head_location_1[1])
        glVertex2f(head_location_2[0], head_location_2[1])
        glVertex2f(point_location[0], point_location[1])
        glEnd()

    def draw_text_at(self, location, text, size, color=(0, 0, 0)):
        """Draw some text at the specified location."""
        width, height = self.GetClientSize() * self.GetContentScaleFactor()
        aspect = float(height) / width
        location = self.scaled_vertex(location[0], location[1], True)

        glPushMatrix()
        glLoadIdentity()
        glTranslatef(location[0], location[1], 0)
        glScalef(size * aspect, size, size)
        glColor3fv(color)
        self.face.render(text)
        glPopMatrix()
