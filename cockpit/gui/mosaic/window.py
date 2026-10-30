#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2021 Frederik Lange
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

import collections
import math
import sys
import threading
import time
from functools import wraps

import numpy as np
import scipy.ndimage
import wx
from OpenGL.GL import *

import cockpit.gui
import cockpit.gui.camera.window
import cockpit.gui.dialogs.get_number_dialog
import cockpit.gui.dialogs.grid_sites_dialog
import cockpit.gui.dialogs.offset_sites_dialog
import cockpit.gui.freetype
import cockpit.gui.gui_utils
import cockpit.gui.keyboard
import cockpit.interfaces
import cockpit.interfaces.stage_mover
import cockpit.util.user_config
from cockpit import depot, events
from cockpit.gui.mosaic import canvas
from cockpit.gui.primitive import Primitive


## Valid colors to use for site markers.
SITE_COLORS = [
    ("green", (0, 1, 0)),
    ("red", (1, 0, 0)),
    ("blue", (0, 0, 1)),
    ("orange", (1, 0.6, 0)),
]

## Timeout for mosaic new image events
CAMERA_TIMEOUT = 1

## Simple structure for marking potential beads.
BeadSite = collections.namedtuple("BeadSite", ["pos", "size", "intensity"])


def _pause_mosaic_loop(func):
    @wraps(func)
    def wrapped(self, *args, **kwargs):
        is_running = self.should_continue.is_set()
        if not is_running:
            return func(self, *args, **kwargs)
        else:
            self.should_continue.clear()
            try:
                return func(self, *args, **kwargs)
            finally:
                self.should_continue.set()

    return wrapped


class MosaicCommon:
    # A class to house methods that are common to both the Mosaic
    # and TouchScreen windows. Previously, these were dynamically
    # rebound on the TouchScreen window, which worked fine in
    # python 3, but threw TypeErrors in python 2.
    # TODO: refactor to eliminate this class; see notes on each method.

    def go_to(self, target, should_block=False):
        """Go to the specified XY position.

        If we have a focus plane defined, go to the appropriate Z position to
        maintain focus. Refactoring: If the stage_mover dealt with the focal
        plane params and mover switching, this method could probably be
        eliminated.
        """
        if self.focal_plane_params:
            target_z = self.get_focus_z(target)
            cockpit.interfaces.stage_mover.go_to(
                (target[0], target[1], target_z), should_block
            )
        else:
            # IMD 20150306 Save current mover, change to coarse to generate mosaic
            # do move, and change mover back.
            original_mover = cockpit.interfaces.stage_mover.mover.cur_handler_index
            cockpit.interfaces.stage_mover.mover.cur_handler_index = 0
            cockpit.interfaces.stage_mover.go_to_xy(target, should_block)
            cockpit.interfaces.stage_mover.mover.cur_handler_index = original_mover

    def draw_overlay(self):
        """Draw the overlay.

        This largely consists of a crosshairs indicating the current stage
        position, and any sites the user has saved. Refactoring: this could
        probably move to the Canvas class by eliminating references to data on
        the MosaicWindow as follows: roll self.scalebar and self.drawCrosshairs
        into this method; put self.scalefont onto the canvas; pull self.offset
        straight from the objective; move self.selected_sites to the stage_mover
        or some other space manager;
        """
        site_line_width = max(1, self.canvas.scale * 1.5)
        site_font_scale = 3 / max(5.0, self.canvas.scale)
        for site in cockpit.interfaces.stage_mover.get_all_sites():
            # Draw a crude circle.
            x, y = site.position[:2]
            x = -x
            glLineWidth(site_line_width)
            glColor3f(*site.color)
            glBegin(GL_LINE_LOOP)
            for i in range(8):
                glVertex3f(
                    x + site.size * np.cos(np.pi * i / 4.0),
                    y + site.size * np.sin(np.pi * i / 4.0),
                    0,
                )
            glEnd()
            glLineWidth(1)

            glPushMatrix()
            glTranslatef(x, y, 0)
            glScalef(site_font_scale, site_font_scale, 1)
            self.site_face.render(str(site.unique_id))
            glPopMatrix()

        self.draw_crosshairs(
            cockpit.interfaces.stage_mover.get_position()[:2],
            (1, 0, 0),
            offset=True,
        )

        if self.display_trails and len(self.trails) > 1:
            glLineWidth(max(1, self.canvas.scale * 1.5))
            # trails colour is green
            glColor3f(0, 1, 0)
            glBegin(GL_LINES)
            last_x = -self.trails[0][0] - self.offset[0]
            last_y = self.trails[0][1] - self.offset[1]
            for vertex in self.trails[1:]:
                glVertex2f(last_x, last_y)
                glVertex2f(
                    -vertex[0] + self.offset[0], vertex[1] - self.offset[1]
                )
                last_x = -vertex[0] + self.offset[0]
                last_y = vertex[1] - self.offset[1]
            glEnd()

        # If we're selecting tiles, draw the box the user is selecting.
        if self.select_tiles_func is not None and self.last_click_pos is not None:
            start = self.canvas.map_screen_to_canvas(self.last_click_pos)
            end = self.canvas.map_screen_to_canvas(self.prev_mouse_pos)
            glColor3f(0, 0, 1)
            glBegin(GL_LINE_LOOP)
            glVertex2f(-start[0], start[1])
            glVertex2f(-start[0], end[1])
            glVertex2f(-end[0], end[1])
            glVertex2f(-end[0], start[1])
            glEnd()

        # Highlight selected sites with crosshairs.
        for site in self.selected_sites:
            self.draw_crosshairs(
                site.position[:2], (0, 0, 1), 10000, offset=False
            )

        # Draw the soft and hard stage motion limits
        glEnable(GL_LINE_STIPPLE)
        glLineWidth(2)
        soft_safeties = cockpit.interfaces.stage_mover.get_soft_limits()[:2]
        hard_safeties = cockpit.interfaces.stage_mover.get_hard_limits()[:2]
        for safeties, color, stipple in [
            (soft_safeties, (0, 1, 0), 0x5555),
            (hard_safeties, (0, 0, 1), 0xAAAA),
        ]:
            x1, x2 = safeties[0]
            y1, y2 = safeties[1]
            if hasattr(self, "offset"):
                # once again consistency of offset calculations.
                x1 -= self.offset[0]
                x2 -= self.offset[0]
                y1 -= self.offset[1]
                y2 -= self.offset[1]
            glLineStipple(3, stipple)
            glColor3f(*color)
            glBegin(GL_LINE_LOOP)
            glVertex2f(-x1, y1)
            glVertex2f(-x2, y1)
            glVertex2f(-x2, y2)
            glVertex2f(-x1, y2)
            glEnd()
        glLineWidth(1)
        glDisable(GL_LINE_STIPPLE)

        # Draw a scale bar if the scalebar size is not zero.
        if self.scalebar != 0:
            # Scale bar width.
            self.scalebar = 100 * (
                10 ** math.floor(math.log(1 / self.canvas.scale, 10))
            )
            # Scale bar position, near the top left-hand corner.
            scale_factor = self.GetContentScaleFactor()
            scalebar_pos = [30 * scale_factor, -10 * scale_factor]

            # Scale bar vertices.
            x1 = scalebar_pos[0] / self.canvas.scale
            x2 = (
                scalebar_pos[0] + self.scalebar * self.canvas.scale
            ) / self.canvas.scale
            y1 = scalebar_pos[1] / self.canvas.scale
            canvas_pos = self.canvas.map_screen_to_canvas((0, 0))
            x1 -= canvas_pos[0]
            x2 -= canvas_pos[0]
            y1 += canvas_pos[1]

            # Do the actual drawing
            glColor3f(255, 0, 0)
            # The scale bar itself.
            glLineWidth(8 * scale_factor)
            glBegin(GL_LINES)
            glVertex2f(x1, y1)
            glVertex2f(x2, y1)
            glEnd()
            glLineWidth(1)
            # The scale label.
            glPushMatrix()
            label_pos_x = x1
            label_pos_y = y1 - (20.0 * scale_factor / self.canvas.scale)
            glTranslatef(label_pos_x, label_pos_y, 0)
            font_scale = scale_factor / self.canvas.scale
            glScalef(font_scale, font_scale, 1.0)
            if self.scalebar > 1.0:
                self.scale_face.render("%d um" % self.scalebar)
            else:
                self.scale_face.render("%.3f um" % self.scalebar)
            glPopMatrix()

        # Draw stage primitives.
        glEnable(GL_LINE_STIPPLE)
        glLineStipple(1, 0xAAAA)
        glColor3f(0.4, 0.4, 0.4)
        glColor3f(0.4, 0.4, 0.4)
        glMatrixMode(GL_MODELVIEW)
        glPushMatrix()
        # Reflect x-coordinates.
        glMultMatrixf([-1.0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1])
        for primitive in self.primitives:
            primitive.render()
        glPopMatrix()
        glDisable(GL_LINE_STIPPLE)

    def draw_crosshairs(self, position, color, size=None, offset=False):
        """Draw a crosshairs at the specified position with the specified color.

        By default make the size of the crosshairs be really big. Refactor:
        this could move to Canvas by eliminating self.crosshair_box_size, which
        is only set and used here, and in centerCanvas.
        """
        xSize = ySize = size
        if size is None:
            xSize = ySize = 100000
        x, y = position
        # offset applied for stage position but not marks!
        if offset:
            # if no offset defined we can't apply it!
            if hasattr(self, "offset"):
                # sign consistency! Here we have -(x-offset) = -x + offset!
                x = x - self.offset[0]
                y = y - self.offset[1]

        # Draw the crosshairs
        glColor3f(*color)
        glBegin(GL_LINES)
        glVertex2d(-x - xSize, y)
        glVertex2d(-x + xSize, y)
        glVertex2d(-x, y - ySize)
        glVertex2d(-x, y + ySize)
        glEnd()

        glBegin(GL_LINE_LOOP)
        # Draw the box.
        cams = wx.GetApp().depot.get_active_cameras()
        # if there is a camera us its real pixel count
        if len(cams) > 0:
            pixel_size = wx.GetApp().objectives.get_pixel_size()
            width, height = cams[0].get_image_size()
            self.crosshair_box_size = width * pixel_size
            width = self.crosshair_box_size
            height = height * pixel_size
        else:
            # else use the default which is 512Xpixel size from objective
            width = self.crosshair_box_size
            height = self.crosshair_box_size

        for i, j in [(-1, -1), (-1, 1), (1, 1), (1, -1)]:
            glVertex2d(-x + i * width / 2, y + j * height / 2)
        glEnd()


class MosaicWindow(wx.Frame, MosaicCommon):
    """This class handles the UI of the mosaic."""

    SHOW_DEFAULT = True
    LIST_AS_COCKPIT_WINDOW = True

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        ## Prevent r-click on buttons displaying main_window context menu.
        self.Bind(wx.EVT_CONTEXT_MENU, lambda event: None)
        ## Last known location of the mouse.
        self.prev_mouse_pos = None
        ## Last click position of the mouse.
        self.last_click_pos = None
        ## Function to call when tiles are selected.
        self.select_tiles_func = None
        ## True if we're generating a mosaic.
        self.am_generating_mosaic = False
        ## Offset (set by objective change)
        self.offset = (0, 0)
        ## Event object to control run state of mosaicLoop.
        self.should_continue = threading.Event()
        self.should_exit = threading.Event()
        primitive_specs = (
            wx.GetApp().config["stage"].getlines("primitives", [])
        )
        self.primitives = [Primitive.factory(spec) for spec in primitive_specs]

        ## Camera used for making a mosaic
        self.camera = None
        # toggle and array for trails
        self.trails = []
        self.display_trails = False

        ## Mosaic tile overlap
        self.overlap = cockpit.util.user_config.get_value(
            "mosaic_tile_overlap", default=0.0
        )

        ## Size of the box to draw at the center of the crosshairs.
        self.crosshair_box_size = 0
        ## Color to use when making new Site instances.
        self.site_color = SITE_COLORS[0][1]
        ## Current selected sites for highlighting with crosshairs.
        self.selected_sites = set()

        ## Parameters defining the focal plane -- a tuple of
        # (point on plane, normal vector to plane).
        self.focal_plane_params = None

        # Fonts to use for site labels and scale bar.  Keep two
        # separate fonts instead of dynamically changing the font size
        # because changing the font size would mean discarding the
        # glyph textures for that size.
        self.site_face = cockpit.gui.freetype.Face(self, 96)
        self.scale_face = cockpit.gui.freetype.Face(self, 18)

        # default scale bar size is Zero
        self.scalebar = cockpit.util.user_config.get_value(
            "mosaicScaleBar", default=0
        )
        ## Maps button names to wx.Button instances.
        self.name_to_button = {}

        side_sizer = wx.BoxSizer(wx.VERTICAL)
        for args in [
            (
                "Run mosaic",
                self.display_mosaic_menu,
                self.continue_mosaic,
                "Generate a map of the sample by stitching together "
                + "images collected with the current lights and one "
                + "camera. Click the Abort button to stop. Right-click "
                + "to continue a previous mosaic.",
            ),
            (
                "Find stage",
                self.center_canvas,
                None,
                "Center the mosaic view on the stage and reset the "
                + "zoom level",
            ),
            (
                "Delete tiles",
                self.on_delete_tiles,
                self.on_delete_all_tiles,
                "Left-click and drag to select mosaic tiles to delete. "
                + "This can free up graphics memory on the computer. Click "
                + "this button again when you are done. Right-click to "
                + "delete every tile in the mosaic.",
            ),
            (
                "Rescale tiles",
                self.autoscale_tiles,
                self.display_rescale_menu,
                "Rescale each tile's black- and white-point. Left-click "
                + "to scale each tile individually. Right-click to select "
                + "a camera's scaling to use instead.",
            ),
            (
                "Save mosaic",
                self.save_mosaic,
                None,
                "Save the mosaic to disk, so it can be recovered later. "
                + "This will generate two files: a .txt file and a .mrc "
                + "file. Load the .txt file to recover the mosaic.",
            ),
            (
                "Load mosaic",
                self.load_mosaic,
                None,
                "Load a mosaic file that was previously saved. Make "
                + "certain you load the .txt file, not the .mrc file.",
            ),
            (
                "Calculate focal plane",
                self.set_focal_plane,
                self.clear_focal_plane,
                "Calculate the focal plane of the sample, assuming that "
                + "the currently-selected sites are all in focus, and that "
                + "the sample is flat. Right-click to clear the focal plane settings."
                + "Once the focal plane is set, all motion in the mosaic "
                + "window (including when making mosaics) will stay in the "
                + "focal plane.",
            ),
            (
                "Mark bead centers",
                self.select_tiles_for_beads,
                None,
                "Allows you to select mosaic tiles and search for isolated "
                + "beads in them. A site will be placed over each one. This "
                + "is useful for collecting PSFs.",
            ),
        ]:
            button = self.make_button(self, *args)
            side_sizer.Add(button, 0, wx.EXPAND)

        ## Panel for controls dealing with specific sites.
        self.sites_panel = wx.Panel(self, style=wx.BORDER_SUNKEN)
        sites_sizer = wx.BoxSizer(wx.VERTICAL)
        ## Holds a list of sites.
        self.sites_box = wx.ListBox(
            self.sites_panel, style=wx.LB_EXTENDED | wx.LB_SORT
        )
        self.sites_box.Bind(wx.EVT_LISTBOX, self.on_select_site)
        self.sites_box.Bind(wx.EVT_LISTBOX_DCLICK, self.on_double_click_site)

        # these are gui events so use the EvtEmitter to run in the main
        # thread.
        cockpit.gui.EvtEmitter(self, cockpit.events.NEW_SITE).Bind(
            cockpit.gui.EVT_COCKPIT,
            self._on_new_site,
        )
        cockpit.gui.EvtEmitter(self, cockpit.events.DELETE_SITE).Bind(
            cockpit.gui.EVT_COCKPIT,
            self._on_site_delete,
        )
        sites_sizer.Add(self.sites_box, 1, wx.EXPAND)

        for args in [
            (
                "Mark site",
                self.save_site,
                self.display_site_maker_menu,
                "Remember the current stage position for later. "
                + "Right-click to change the marker color.",
            ),
            (
                "Make grid of sites",
                lambda *args: cockpit.gui.dialogs.grid_sites_dialog.show_dialog(
                    self
                ),
                None,
                "Generate a 2D array of sites.",
            ),
            (
                "Delete selected sites",
                self.delete_selected_sites,
                None,
                "Delete the selected sites.",
            ),
            (
                "Adjust selected sites",
                self.offset_selected_sites,
                None,
                "Move the selected sites by some offset.",
            ),
            (
                "Save sites to file",
                self.save_sites_to_file,
                None,
                "Save the site positions to a file for later recovery.",
            ),
            (
                "Load saved sites",
                self.load_saved_sites,
                None,
                'Load sites from a file previously generated by the "Save sites to file" button.',
            ),
        ]:
            button = self.make_button(self.sites_panel, *args)
            sites_sizer.Add(button, 0, wx.EXPAND)

        self.sites_panel.SetSizerAndFit(sites_sizer)
        side_sizer.Add(self.sites_panel, 1, wx.EXPAND)
        sizer.Add(side_sizer, 0, wx.EXPAND)

        # The MosaicCanvas can't figure out its own best size so it
        # just disappears after Fit.  We suggest its width to be 3/4
        # of the window width.
        side_panel_size = side_sizer.ComputeFittingClientSize(self)
        canvas_size = (side_panel_size[0] * 3, side_panel_size[1])

        ## MosaicCanvas instance.
        limits = cockpit.interfaces.stage_mover.get_hard_limits()[:2]
        self.canvas = canvas.MosaicCanvas(
            self, limits, self.draw_overlay, self.on_mouse, size=canvas_size
        )
        sizer.Add(self.canvas, 1, wx.EXPAND)

        self.SetSizerAndFit(sizer)

        events.subscribe(events.STAGE_POSITION, self.on_axis_refresh)
        events.subscribe(events.SOFT_SAFETY_LIMIT, self.on_axis_refresh)
        events.subscribe(events.MOSAIC_UPDATE, self.mosaic_update)
        events.subscribe(events.UPDATE_ROI, self.update_roi)

        abort_emitter = cockpit.gui.EvtEmitter(self, events.USER_ABORT)
        abort_emitter.Bind(cockpit.gui.EVT_COCKPIT, self.on_abort)

        wx.GetApp().objectives.Bind(
            cockpit.interfaces.EVT_OBJECTIVE_CHANGED,
            self._on_objective_changed,
        )
        self.Bind(wx.EVT_MOUSE_EVENTS, self.on_mouse)
        for item in [self, self.canvas, self.sites_panel]:
            cockpit.gui.keyboard.set_keyboard_handlers(item)

        self.mosaic_thread = None

        ## Dont continue mosaics if we change objective
        events.subscribe(events.OBJECTIVE_CHANGE, self.on_objective_change)

        self.Bind(wx.EVT_WINDOW_DESTROY, self.on_destroy)

    def on_destroy(self, event: wx.WindowDestroyEvent) -> None:
        if self.mosaic_thread is not None and self.mosaic_thread.is_alive():
            # This should signal the mosaic thread...
            self.should_exit.set()
            # ... but if it is paused we need to unpause it as well.
            self.should_continue.set()

            self.mosaic_thread.join()
        event.Skip()

    def on_objective_change(self, objective):
        """Objective change sets the should_restart flag so we dont continue in
        the wrong place."""
        self.should_restart = True

    def make_button(
        self, parent, label, left_action, right_action, help_text, size=(-1, -1)
    ):
        """Create a button with the appropriate properties."""
        button = wx.Button(parent, -1, label, size=size)
        button.SetToolTip(wx.ToolTip(help_text))
        button.Bind(wx.EVT_BUTTON, lambda event: left_action())
        if right_action is not None:
            button.Bind(wx.EVT_RIGHT_DOWN, lambda event: right_action())
        self.name_to_button[label] = button
        return button

    def center_canvas(self, event=None):
        """Now that we've been created, recenter the canvas."""
        cur_position = cockpit.interfaces.stage_mover.get_position()[:2]
        # As we read a position propagate a stage event to ensure
        # other interface components are in synch.
        events.publish(events.STAGE_POSITION, 0, cur_position[0])
        events.publish(events.STAGE_POSITION, 1, cur_position[1])
        # Calculate the size of the box at the center of the crosshairs.
        # \todo Should we necessarily assume a 512x512 area here?
        # if we haven't previously set crosshair_box_size (maybe no camera active)
        if self.crosshair_box_size == 0:
            self.crosshair_box_size = 512 * wx.GetApp().objectives.get_pixel_size()
        self.offset = wx.GetApp().objectives.get_offset()
        scale = 150.0 / self.crosshair_box_size
        self.canvas.zoom_to(
            -cur_position[0] + self.offset[0],
            cur_position[1] - self.offset[1],
            scale,
        )

    def on_axis_refresh(self, axis, *args):
        """Get updated about new stage position info.

        This requires redrawing the display, if the axis is the X or Y axes.
        """
        if axis in [0, 1]:
            # Only care about the X and Y axes.
            # append new position
            self.trails.append(cockpit.interfaces.stage_mover.get_position()[:2])
            wx.CallAfter(self.canvas.Refresh)

    def _on_objective_changed(self, event: wx.CommandEvent) -> None:
        """User changed the objective in use; resize our crosshair box to suit."""
        self.crosshair_box_size = 512 * wx.GetApp().objectives.get_pixel_size()
        self.offset = wx.GetApp().objectives.get_offset()
        # force a redraw so that the crosshairs are properly sized
        # Refresh this and other mosaic views.
        events.publish(events.MOSAIC_UPDATE)
        event.Skip()

    def on_mouse(self, event):
        """Handle mouse events."""
        if self.prev_mouse_pos is None:
            # We can't perform some operations without having a prior mouse
            # position, so if it doesn't exist yet, we short-circuit the
            # function. Normally we'll set this at the end of the function.
            self.prev_mouse_pos = event.GetPosition()
            return

        mouse_pos = event.GetPosition()
        if event.LeftDown():
            self.last_click_pos = event.GetPosition()
        elif event.LeftUp() and self.select_tiles_func is not None:
            # Call the specified function with the given range.
            start = self.canvas.map_screen_to_canvas(self.last_click_pos)
            end = self.canvas.map_screen_to_canvas(self.prev_mouse_pos)
            self.select_tiles_func((-start[0], start[1]), (-end[0], end[1]))
            self.last_click_pos = None
            self.Refresh()
        # Skip all other inputs while we select tiles.
        if self.select_tiles_func is None:
            if event.LeftDClick():
                # Double left-click; move to the target position.
                current_target = self.canvas.map_screen_to_canvas(mouse_pos)
                new_target = (
                    current_target[0] + self.offset[0],
                    current_target[1] + self.offset[1],
                )
                # Stop mosaic if we are running one.
                if self.am_generating_mosaic:
                    self.on_abort()
                self.go_to(new_target)
            elif event.LeftIsDown() and not event.LeftDown():
                # Dragging the mouse with the left mouse button: drag or
                # zoom, as appropriate.
                delta = (
                    mouse_pos[0] - self.prev_mouse_pos[0],
                    mouse_pos[1] - self.prev_mouse_pos[1],
                )
                if event.ShiftDown():
                    # Use the vertical component of mouse motion to zoom.
                    zoom_factor = 1 - delta[1] / 100.0
                    self.canvas.multiply_zoom(zoom_factor)
                else:
                    self.canvas.drag_view(delta)
                # Clear the currently-selected sites so the user doesn't have
                # to see crosshairs all the time.
                self.selected_sites = set()
            elif event.GetWheelRotation():
                # Adjust zoom, based on the zoom rate.
                delta = event.GetWheelRotation()
                multiplier = 1.002
                if delta < 0:
                    # Invert the scaling direction.
                    multiplier = 2 - multiplier
                    delta *= -1
                self.canvas.zoom_at_point(multiplier ** delta, mouse_pos)
        if event.RightDown():
            # Display a context menu.
            menu = wx.Menu()
            menu_id = 1
            for label, color in SITE_COLORS:
                menu.Append(menu_id, "Mark site with %s marker" % label)
                self.Bind(
                    wx.EVT_MENU,
                    lambda event, color=color: self.save_site(color),
                    id=menu_id,
                )
                menu_id += 1
            menu.AppendSeparator()
            menu.Append(menu_id, "Set mosaic tile overlap")
            self.Bind(
                wx.EVT_MENU, lambda event: self.set_tile_overlap(), id=menu_id
            )
            menu_id += 1
            menu.Append(menu_id, "Toggle mosaic scale bar")
            self.Bind(
                wx.EVT_MENU, lambda event: self.toggle_scalebar(), id=menu_id
            )
            menu_id += 1
            menu.Append(menu_id, "Toggle trails display")
            self.Bind(
                wx.EVT_MENU,
                lambda event: self.toggle_display_trails(),
                id=menu_id,
            )
            menu_id += 1
            menu.Append(menu_id, "Clear old trails")
            self.Bind(wx.EVT_MENU, lambda event: self.clear_trails(), id=menu_id)

            cockpit.gui.gui_utils.place_menu_at_mouse(self, menu)

        self.prev_mouse_pos = mouse_pos

        if self.select_tiles_func is not None:
            # Need to draw the box the user is drawing.
            self.Refresh()

        # HACK: switch focus to the canvas away from our listbox, otherwise
        # it will seize all future scrolling events.
        if self.IsActive():
            self.canvas.SetFocus()

    def mosaic_stepper(self):
        """This generator function creates a clockwise spiral pattern."""
        directions = [(0, -1), (-1, 0), (0, 1), (1, 0)]
        cur_spiral_size = 1
        last_x = last_y = 0
        i = 0
        while True:
            dx, dy = directions[i % 4]
            dx *= 1 - (self.overlap / 100.0)
            dy *= 1 - (self.overlap / 100.0)
            for j in range(1, cur_spiral_size + 1):
                yield (last_x + dx * j, last_y + dy * j)
            last_x += dx * cur_spiral_size
            last_y += dy * cur_spiral_size
            if i % 2:
                cur_spiral_size += 1
            i += 1

    def toggle_mosaic(self):
        """Toggle run / stop state of the mosaicLoop."""
        if self.mosaic_thread is None or not self.mosaic_thread.is_alive():
            self.should_reconfigure = True
            self.should_restart = True
            self.mosaic_thread = threading.Thread(
                target=self.mosaic_loop, name="mosaic"
            )
            self.mosaic_thread.start()
        if self.should_continue.is_set():
            self.should_continue.clear()
        else:
            self.should_continue.set()

    def on_stage_move_when_paused(self, axis, position):
        """Detect stage movement when paused and flag that the mosaic loop
        should start a new spiral if the stage has moved."""
        if axis == 2:
            return
        events.unsubscribe(events.STAGE_POSITION, self.on_stage_move_when_paused)
        self.should_restart = True

    def generate_mosaic(self, camera):
        """Generate a spiral mosaic.

        Args:
            camera (CameraHandler): Handler of the camera we're collecting
                images from.
        """
        self.camera = camera
        self.toggle_mosaic()

    def mosaic_loop(self):
        """Move the stage in a spiral pattern, stopping to take images at
        regular intervals, to generate a stitched-together high-level view of
        the stage contents."""
        stepper = self.mosaic_stepper()
        target = None
        while True:
            if self.should_exit.is_set():
                return
            if not self.should_continue.is_set():
                ## Enter idle state.
                # Update button label in main thread.
                events.publish(events.MOSAIC_STOP)
                wx.CallAfter(
                    self.name_to_button["Run mosaic"].SetLabel, "Run mosaic"
                )
                # Detect stage movement so know whether to start new spiral on new position.
                events.subscribe(
                    events.STAGE_POSITION, self.on_stage_move_when_paused
                )
                # Wait for should_continue event.
                self.should_continue.wait()
                # Check if we should exit the thread.
                if self.should_exit.is_set():
                    return

                # Clear subscription
                events.unsubscribe(
                    events.STAGE_POSITION, self.on_stage_move_when_paused
                )
                # Update button label in main thread.
                wx.CallAfter(
                    self.name_to_button["Run mosaic"].SetLabel, "Stop mosaic"
                )
                # Set reconfigure flag: cameras or objective may have changed.
                self.should_reconfigure = True
                events.publish(events.MOSAIC_START)
                # Catch case that stage has moved but user wants to continue mosaic.
                if not self.should_restart and target is not None:
                    self.go_to(target, True)

            if self.should_restart:
                # Start a new spiral about current stage position.
                stepper = self.mosaic_stepper()
                position = cockpit.interfaces.stage_mover.get_position()
                center_x = position[0] - self.offset[0]
                center_y = position[1] + self.offset[1]
                self.should_restart = False

            if self.should_reconfigure:
                #  Check that camera is valid
                active = wx.GetApp().depot.get_active_cameras()
                if len(active) == 0:
                    self.should_continue.clear()
                    sys.stderr.write("Mosaic stopping: no active cameras.\n")
                    continue
                camera = self.camera
                # Fallback to 0th active camera.
                if camera not in active:
                    camera = active[0]
                # Set image width and height based on camera and objective.
                pixel_size = wx.GetApp().objectives.get_pixel_size()
                width, height = camera.get_image_size()
                width *= pixel_size
                height *= pixel_size
                self.offset = wx.GetApp().objectives.get_offset()
                # Successfully reconfigured: clear the flag.
                self.should_reconfigure = False

            position = cockpit.interfaces.stage_mover.get_position()
            cur_z = position[2] - self.offset[2]
            # Take an image. Use timeout to prevent getting stuck here.
            try:
                data, metadata = events.execute_and_wait_for_or_timeout(
                    events.NEW_IMAGE % camera.name,
                    wx.GetApp().imager.take_image,
                    camera.get_exposure_time() / 1000
                    + camera.get_time_between_exposures() / 1000
                    + CAMERA_TIMEOUT,
                    should_block=True,
                )
            except Exception as e:
                # Go to idle state.
                self.should_continue.clear()
                sys.stderr.write(
                    "Mosaic stopping - problem taking image: %s\n" % str(e)
                )
                continue

            # Get the scaling for the camera we're using, since they may
            # have changed.
            try:
                min_val, max_val = cockpit.gui.camera.window.get_camera_scaling(
                    camera
                )
                # HACK: If this is the first image being acquired the
                # viewCanvas has not yet set the scaling.  Its default
                # of [0 1] is unlikely to be appropriate for images
                # that are likely uint8/16.  So wait a bit and read it
                # again.  We should either be deciding the scaling
                # ourselves, or get an image with associated scaling
                # information or after the scaling information has
                # been set.  See issue #718.
                if (min_val == 0.0) and (max_val == 1.0):
                    time.sleep(0.1)
                    (
                        min_val,
                        max_val,
                    ) = cockpit.gui.camera.window.get_camera_scaling(camera)
            except Exception as e:
                # Go to idle state.
                self.should_continue.clear()
                sys.stderr.write(
                    "Mosaic stopping - problem in get_camera_scaling: %s\n"
                    % str(e)
                )
                continue

            # Paint the tile at the stage position at which image was captured.
            self.canvas.add_image(
                data,
                (
                    -position[0] + self.offset[0] - width / 2,
                    position[1] - self.offset[1] - height / 2,
                    cur_z,
                ),
                (width, height),
                scalings=(min_val, max_val),
                metadata=metadata,
            )
            # Move to the next position in shifted coords.
            dx, dy = next(stepper)
            target = (
                center_x + self.offset[0] + dx * width,
                center_y - self.offset[1] + dy * height,
            )
            try:
                self.go_to(target, True)
            except Exception as e:
                self.should_continue.clear()
                sys.stderr.write(
                    "Mosaic stopping - problem in target calculation: %s\n"
                    % str(e)
                )
                continue

    def set_tile_overlap(self):
        """Display dialogue box to set tile overlap."""
        value = cockpit.gui.dialogs.get_number_dialog.getNumberFromUser(
            self.GetParent(),
            "Set mosaic tile overlap.",
            "Tile overlap in %",
            self.overlap,
            atMouse=True,
        )
        self.overlap = float(value)
        cockpit.util.user_config.set_value("mosaic_tile_overlap", self.overlap)

    def transfer_camera_image(self):
        """Transfer an image from the active camera (or first camera) to the
        mosaic at the current stage position."""
        camera = self.camera
        if camera is None or not camera.get_is_enabled():
            # Select the first active camera.
            for cam in wx.GetApp().depot.get_handlers_of_type(depot.CAMERA):
                if cam.get_is_enabled():
                    camera = cam
                    break
        # Get image size in microns.
        # This data should be grabbed from the metadata as the system
        # settings, stage pos etc may have changed
        data, metadata = cockpit.gui.camera.window.get_image_for_camera(camera)
        pixel_size = metadata["pixel_size"]
        width, height = camera.get_image_size()
        width *= pixel_size
        height *= pixel_size
        x, y, z = metadata["image_position"]

        self.canvas.add_image(
            data,
            (
                -x + self.offset[0] - width / 2,
                y - self.offset[1] - height / 2,
                z - self.offset[2],
            ),
            (width, height),
            scalings=cockpit.gui.camera.window.get_camera_scaling(camera),
            metadata=metadata,
        )
        # Refresh this and other mosaic views.
        events.publish(events.MOSAIC_UPDATE)

    def toggle_scalebar(self):
        # toggle the scale bar between 0 and 1.
        if self.scalebar != 0:
            self.scalebar = 0
        else:
            self.scalebar = 1
        # store current state for future.
        cockpit.util.user_config.set_value("mosaicScaleBar", self.scalebar)
        # send a mosaic update event to update touch_screen
        events.publish(events.MOSAIC_UPDATE)

    def toggle_display_trails(self):
        # toggle Display of trails in mosaic.
        self.display_trails = not self.display_trails
        # store current state for future.
        cockpit.util.user_config.set_value(
            "mosaicDisplayTrails", self.display_trails
        )
        # send a mosaic update event to update touch_screen
        events.publish(events.MOSAIC_UPDATE)

    def clear_trails(self):
        # clear all existing trails
        self.trails = []
        events.publish(events.MOSAIC_UPDATE)

    def mosaic_update(self):
        self.Refresh()

    def update_roi(self, camera_name):
        # camera roi has updated so check if we are using this camera and
        # if so update imaging region
        if self.camera is not None:
            if camera_name == self.camera.name:
                # publish event toi update this mosaic and touch_screen
                events.publish(events.MOSAIC_UPDATE)

    def save_site(self, color=None):
        """Save the current stage position as a new site with the specified
        color (or our currently-selected color if none is provided)."""
        if color is None:
            color = self.site_color
        position = cockpit.interfaces.stage_mover.get_position()
        position[0] = position[0] - self.offset[0]
        position[1] = position[1] - self.offset[1]
        position[2] = position[2] - self.offset[2]
        cockpit.interfaces.stage_mover.save_site(
            cockpit.interfaces.stage_mover.Site(
                position, None, color, size=self.crosshair_box_size
            )
        )
        # Publish mosaic update event to update this and other views (e.g. touch_screen).
        events.publish(events.MOSAIC_UPDATE)

    def set_site_color(self, color):
        """Set the site marker color."""
        self.site_color = color
        for label, alt_color in SITE_COLORS:
            if alt_color == color:
                self.name_to_button["Mark site"].SetLabel(
                    "Mark site (%s)" % label
                )
                break

    def display_site_maker_menu(self, event=None):
        """Display a menu that allows the user to control the appearance of
        the markers used to mark sites."""
        menu = wx.Menu()
        for i, (label, color) in enumerate(SITE_COLORS):
            menu.Append(i + 1, "Mark sites in %s" % label)
            self.Bind(
                wx.EVT_MENU,
                lambda event, color=color: self.set_site_color(color),
                id=i + 1,
            )
        cockpit.gui.gui_utils.place_menu_at_mouse(self, menu)

    def set_focal_plane(self, event=None):
        """Calculate the focal plane of the sample."""
        sites = self.get_selected_sites()
        positions = [s.position for s in sites]
        if len(positions) < 3:
            wx.MessageBox(
                "Please select at least 3 in-focus sites.",
                caption="Insufficient input.",
                parent=self,
            )
            return
        positions = np.array(positions)
        # Pick a point in the plane, as the average of all site positions.
        center = positions.mean(axis=0)
        # Try every combinations of points, and average their resulting normal
        # vectors together.
        normals = []
        for i in range(len(positions)):
            p1 = positions[i] - center
            for j in range(i + 1, len(positions)):
                p2 = positions[j] - center
                for k in range(j + 1, len(positions)):
                    p3 = positions[k] - center
                    # Calculate normal vector, and normalize
                    normal = np.cross(p2 - p1, p3 - p1)
                    magnitude = np.sqrt(sum(normal * normal))
                    normals.append(normal / magnitude)

        # Ensure all normals point in the same direction. If they oppose,
        # their sum should be ~0; if they are aligned, it should be
        # ~2.
        normals = np.array(normals)
        base = normals[0]
        for normal in normals[1:]:
            if sum(base + normal) < 0.5:
                # Opposed normals.
                normal *= -1
        self.focal_plane_params = (center, normals.mean(axis=0))
        deltas = []
        for site in sites:
            pos = np.array(site.position)
            z = self.get_focus_z(pos)
            deltas.append(pos[2] - z)
            print("Delta for", pos, "is", (pos[2] - z))
        print(
            "Average delta is",
            np.mean(deltas),
            "with std",
            np.std(deltas),
        )

    def clear_focal_plane(self):
        """Clear the focal plane settings."""
        self.focal_plane_params = None

    def get_focus_z(self, point):
        """Calculate the Z position in focus for a given XY position, according
        to our focal plane parameters."""
        center, normal = self.focal_plane_params
        point = np.array(point)
        z = (
            -np.dot(normal[:2], point[:2] - center[:2]) / normal[2]
            + center[2]
        )
        return z

    def on_select_site(self, event=None):
        r"""User clicked on a site in the sites box; draw a crosshairs on it.
        \todo Enforcing int site IDs here."""
        self.selected_sites = set()
        for item in self.sites_box.GetSelections():
            text = self.sites_box.GetString(item)
            siteID = int(text.split(":")[0])
            self.selected_sites.add(
                cockpit.interfaces.stage_mover.get_site(siteID)
            )
        # Refresh this and other mosaic views.
        events.publish(events.MOSAIC_UPDATE)

    def on_double_click_site(self, event):
        r"""User double-clicked on a site in the sites box; go to that site.
        \todo Enforcing int site IDs here."""
        item = event.GetString()
        site_id = int(item.split(":")[0])
        cockpit.interfaces.stage_mover.go_to_site(site_id)

    def get_selected_sites(self):
        """Return a list of of the currently-selected Sites."""
        result = []
        for item in self.sites_box.GetSelections()[::-1]:
            text = self.sites_box.GetString(item)
            site_id = int(text.split(":")[0])
            result.append(cockpit.interfaces.stage_mover.get_site(site_id))
        return result

    def delete_selected_sites(self, event=None):
        """Delete the sites the user has selected in our sitebox."""
        # Go in reverse order so that removing items from the box doesn't
        # invalidate future indices.
        for item in self.sites_box.GetSelections()[::-1]:
            text = self.sites_box.GetString(item)
            site_id = int(text.split(":")[0])
            self.selected_sites.remove(
                cockpit.interfaces.stage_mover.get_site(site_id)
            )
            cockpit.interfaces.stage_mover.delete_site(site_id)
            self.sites_box.Delete(item)
        ## Deselect everything to work around issue #408 (under gtk,
        ## deleting items will move the selection to the next item)
        self.sites_box.SetSelection(wx.NOT_FOUND)
        # Refresh this and other mosaic views.
        events.publish(events.MOSAIC_UPDATE)

    def offset_selected_sites(self, event=None):
        """Move the selected sites by an offset."""
        items = self.sites_box.GetSelections()
        if not items:
            # No selected sites.
            return
        offset = cockpit.gui.dialogs.offset_sites_dialog.showDialogModal(self)
        if offset is not None:
            for item in items:
                site_id = int(self.sites_box.GetString(item).split(":")[0])
                site = cockpit.interfaces.stage_mover.get_site(site_id)
                # Account for the fact that the site position may be a
                # (non-mutable) tuple; cast it to a list before modifying it.
                position = list(site.position)
                for axis, value in enumerate(offset):
                    position[axis] += value
                site.position = tuple(position)
            # Redisplay the sites in the sitesbox.
            self.sites_box.Clear()
            for site in cockpit.interfaces.stage_mover.get_all_sites():
                self._add_site_to_list(site, should_refresh=False)
            # Refresh this and other mosaic views.
            events.publish(events.MOSAIC_UPDATE)

    def save_sites_to_file(self, event=None):
        """Save sites to a file."""
        dialog = wx.FileDialog(
            self,
            style=wx.FD_SAVE,
            wildcard="*.txt",
            message="Please select where to save the file.",
            defaultDir=wx.GetApp().config.getpath("global", "data-dir"),
        )
        if dialog.ShowModal() != wx.ID_OK:
            return
        cockpit.interfaces.stage_mover.write_sites_to_file(dialog.GetPath())

    def load_saved_sites(self, event=None):
        """Load sites from a file."""
        dialog = wx.FileDialog(
            self,
            style=wx.FD_OPEN,
            wildcard="*.txt",
            message="Please select the file to load.",
            defaultDir=wx.GetApp().config.getpath("global", "data-dir"),
        )
        if dialog.ShowModal() != wx.ID_OK:
            return
        cockpit.interfaces.stage_mover.load_sites(dialog.GetPath())

    def _on_new_site(self, event: wx.CommandEvent) -> None:
        """A new site was created (from any source); add it to our sites box."""
        site = event.EventData[0]
        self._add_site_to_list(site, should_refresh=True)

    def _add_site_to_list(self, site, should_refresh=True):
        # This display is a bit compressed, so that all positions are visible
        # even if there's a scrollbar in the sites box.
        position = ",".join(["%d" % p for p in site.position])
        label = site.unique_id
        # HACK: most unique_id instances will be ints, which we zero-pad
        # so that they stay in appropriate order.
        if type(label) is int:
            label = "%04d" % label
        self.sites_box.Append("%s: %s" % (label, position))
        if should_refresh:
            # Refresh this and other mosaic views.
            events.publish(events.MOSAIC_UPDATE)

    def _on_site_delete(self, event: wx.CommandEvent) -> None:
        """A site was deleted; remove it from our sites box."""
        site = event.EventData[0]
        for item in self.sites_box.GetItems():
            if site.unique_id == item:
                self.sites_box.Delete(item)
                break

    def display_mosaic_menu(self):
        """Display a menu to the user letting them choose which camera to use
        to generate a mosaic.

        Of course, if only one camera is available, then we just do the mosaic.
        """
        # If we're already running a mosaic, stop it instead.
        if self.should_continue.is_set():
            self.should_continue.clear()
            return

        self.show_camera_menu("Make mosaic with %s camera", self.generate_mosaic)

    def continue_mosaic(self):
        """Force continuation of mosaic after stage move."""
        self.should_restart = False
        self.toggle_mosaic()

    def show_camera_menu(self, text, action):
        """Generate a menu where the user can select a camera to use to perform
        some action.

        Args:
            text (str): String template to use for entries in the menu.
            action (callable): Function to call with the selected camera as a
                parameter.
        """
        cameras = wx.GetApp().depot.get_active_cameras()
        if len(cameras) == 0:
            wx.MessageBox(
                "Please enable a camera to run a mosaic.",
                caption="No cameras are enabled",
            )
        elif len(cameras) == 1:
            action(cameras[0])
        else:
            menu = wx.Menu()
            for i, camera in enumerate(cameras):
                menu.Append(i + 1, text % camera.descriptive_name)
                self.Bind(
                    wx.EVT_MENU,
                    lambda event, camera=camera: action(camera),
                    id=i + 1,
                )
            cockpit.gui.gui_utils.place_menu_at_mouse(self, menu)

    def set_select_func(self, func):
        """Set the function to use when the user selects tiles."""
        self.select_tiles_func = func
        self.last_click_pos = None

    def on_delete_tiles(self, event=None, should_force_stop=None):
        """User clicked the "delete tiles" button; start/stop deleting tiles."""
        am_deleting = "Stop" not in self.name_to_button["Delete tiles"].GetLabel()
        if should_force_stop:
            am_deleting = False
        label = ["Delete tiles", "Stop deleting"][am_deleting]
        self.name_to_button["Delete tiles"].SetLabel(label)
        if am_deleting:
            self.set_select_func(self.canvas.delete_tiles_intersecting)
        else:
            self.set_select_func(None)

    @_pause_mosaic_loop
    def on_delete_all_tiles(self, event=None):
        """Delete all tiles in the mosaic, after prompting the user for
        confirmation."""
        if not cockpit.gui.gui_utils.get_user_permission(
            "Are you sure you want to delete every tile in the mosaic?",
            "Delete confirmation",
        ):
            return
        self.canvas.delete_all()
        self.should_restart = True

    @_pause_mosaic_loop
    def autoscale_tiles(self, event=None):
        """Rescale each tile according to that tile's own values."""
        self.canvas.rescale(None)

    def display_rescale_menu(self, event=None):
        """Let the user select a camera to use to rescale the tiles."""
        self.show_camera_menu(
            "Rescale according to %s camera", self.rescale_with_camera
        )

    @_pause_mosaic_loop
    def rescale_with_camera(self, camera):
        """Given a camera handler, rescale the mosaic tiles based on that
        camera's display's black- and white-points."""
        self.canvas.rescale(cockpit.gui.camera.window.get_camera_scaling(camera))

    def save_mosaic(self, event=None):
        """Save the mosaic to disk.

        We generate a text file describing the locations of the mosaic tiles,
        and an MRC file of the tiles themselves.
        """
        dialog = wx.FileDialog(
            self,
            style=wx.FD_SAVE,
            wildcard="*.txt",
            message="Please select where to save the file.",
            defaultDir=wx.GetApp().config.getpath("global", "data-dir"),
        )
        if dialog.ShowModal() != wx.ID_OK:
            return
        self.canvas.save_tiles(dialog.GetPath())

    def load_mosaic(self, event=None):
        """Load a mosaic that was previously saved to disk."""
        dialog = wx.FileDialog(
            self,
            style=wx.FD_OPEN,
            wildcard="*.txt",
            message="Please select the .txt file the mosaic was saved to.",
        )
        if dialog.ShowModal() != wx.ID_OK:
            return
        self.canvas.load_tiles(dialog.GetPath())

    def select_tiles_for_beads(self):
        """Prepare to mark bead centers."""
        self.set_select_func(self.mark_bead_centers)

    def mark_bead_centers(self, start, end):
        """Examine the mosaic, trying to find isolated bead centers, and
        putting a site marker on each one.

        We partition each tile of the mosaic into subsections, find connected
        components, and mark them if they are isolated.
        """
        # Cancel selecting beads now that we have what we need.
        self.set_select_func(None)
        tiles = self.canvas.get_tiles_intersecting(start, end)
        status_dialog = wx.ProgressDialog(
            parent=self,
            title="Finding bead centers",
            message="Scanning mosaic...",
            maximum=len(tiles),
            style=wx.PD_CAN_ABORT,
        )
        status_dialog.Show()
        region_size = 300
        # List of BeadSite instances for potential beads
        bead_sites = []
        for i, tile in enumerate(tiles):
            # NB should_skip is always false because we don't provide a skip
            # button.
            should_continue, should_skip = status_dialog.Update(i)
            if not should_continue:
                # User cancelled.
                break
            try:
                data = self.canvas.get_composite_tile_data(tile, tiles)
            except Exception as e:
                print("Failed to get tile data at %s: %s" % (tile.position, e))
                break
            pixel_size = tile.get_pixel_size()
            median = np.median(data)
            std = np.std(data)
            # Threshold the data so that background becomes 0 and signal
            # becomes 1 -- admittedly the threshold value is somewhat
            # arbitrary.
            thresholded = np.zeros(data.shape, dtype=np.uint16)
            thresholded[np.where(data > median + std * 15)] = 1

            # Slice up into overlapping regions. Only examine the center
            # portion of the composite image.
            for j in range(
                data.shape[0] / 3, 2 * data.shape[0] / 3, region_size / 4
            ):
                for k in range(
                    data.shape[1] / 3, 2 * data.shape[1] / 3, region_size / 4
                ):
                    region = thresholded[
                        j : j + region_size, k : k + region_size
                    ]
                    # Skip overly small regions (on the off-chance that
                    # region_size is a significant portion of the tile size).
                    if (
                        region.shape[0] < region_size
                        or region.shape[1] < region_size
                    ):
                        continue
                    # Find connected components in data.
                    num_components = scipy.ndimage.label(region)[1]
                    if num_components != 1:
                        # More than one bead visible, or no beads at all.
                        continue
                    # Find the centroid of the component
                    y_vals, x_vals = np.where(region == 1)
                    x, y = np.mean(x_vals), np.mean(y_vals)
                    # Ensure that the bead is not near the edge, where
                    # it might be close to a bead in a different region. Note
                    # that our region iteration overlaps, so if a bead is truly
                    # isolated we'll pick it up on a different loop.
                    if (
                        x < region_size * 0.25
                        or x > region_size * 0.75
                        or y < region_size * 0.25
                        or y > region_size * 0.75
                    ):
                        continue
                    # Ensure that the bead is circular, by comparing the area
                    # of the bead to the area of a circle containing all of
                    # the bead's pixels.
                    x_dists = [(x - xi) ** 2 for xi in x_vals]
                    y_dists = [(y - yi) ** 2 for yi in y_vals]
                    max_dist_squared = max(map(sum, list(zip(x_dists, y_dists))))
                    # Area of a circle containing all pixels
                    area = np.pi * max_dist_squared
                    # Reject beads whose area is less than 60% of the area of
                    # the circle.
                    if len(x_vals) / area < 0.6:
                        continue

                    # Go from the subregion coordinates to full-tile coordinates
                    x += k - data.shape[1] / 3
                    y += j - data.shape[0] / 3
                    pos = np.array(
                        [
                            -tile.position[0] - x * pixel_size[0],
                            tile.position[1] + y * pixel_size[1],
                            tile.position[2],
                        ]
                    )
                    # Check for other marked beads that are close to this one.
                    # \todo This process makes the entire system N^2
                    # (where N is the number of sites), so it's moderately
                    # expensive.
                    can_keep = True
                    for site in bead_sites:
                        distance = np.sqrt(sum((pos - site.position) ** 2))
                        if distance < 40:
                            # Within 40 microns of another bead; skip it.
                            can_keep = False
                            break
                    if not can_keep:
                        continue
                    # Record this potential bead. Its "size" is the number
                    # of pixels in the component, and its intensity is the
                    # average intensity of pixels in the component.
                    new_site = BeadSite(
                        pos, len(x_vals), np.mean(data[(y_vals, x_vals)])
                    )
                    bead_sites.append(new_site)

        # Examine our bead sites and reject ones that are:
        # - too large (probably conjoined or overlapping beads)
        # - too bright (ditto)
        # - too dim (bad signal:noise ratio)
        # - too small (Could just be autoflourescing dust or something)
        # Part of the trick here is that many beads may be slightly out of
        # focus, so these constraints can't actually be all that tight.
        sizes = np.array([b.size for b in bead_sites])
        size_median = np.median(sizes)
        size_std = np.std(sizes)
        intensities = np.array([b.intensity for b in bead_sites])
        inten_median = np.median(intensities)
        inten_std = np.std(intensities)
        site_queue = []
        for i, site in enumerate(bead_sites):
            if (
                site.size < size_median - size_std * 0.5
                or site.size > size_median + size_std * 2
            ):
                # Too big or too small.
                continue
            if abs(site.intensity - inten_median) > inten_std * 5:
                # Wrong brightness.
                continue
            site_queue.append(site.position)

        # Scan each site in Z to get perfect focus. Look up/down +- 1 micron,
        # and pick the Z altitude with the brightest image.
        # HACK: use the first active camera we find.
        cameras = wx.GetApp().depot.get_handlers_of_type(depot.CAMERA)
        camera = None
        for alt in cameras:
            if alt.get_is_enabled():
                camera = alt
                break
        for x, y, z in site_queue:
            best_offset = 0
            best_intensity = None
            for offset in np.arange(-1, 1.1, 0.1):
                cockpit.interfaces.stage_mover.go_to(
                    (x, y, z + offset), should_block=True
                )
                image, metadata = events.execute_and_wait_for(
                    events.NEW_IMAGE % camera.name,
                    wx.GetApp().imager.take_image,
                    should_block=True,
                )
                timestamp = metadata["timestamp"]
                if best_intensity is None or image.max() > best_intensity:
                    best_intensity = image.max()
                    best_offset = offset
            new_site = cockpit.interfaces.stage_mover.Site(
                (x, y, z + best_offset), group="beads", size=2
            )
            wx.CallAfter(cockpit.interfaces.stage_mover.save_site, new_site)
        status_dialog.Destroy()
        wx.CallAfter(self.Refresh)

    def on_abort(self, *args):
        """Handle the user clicking the abort button."""
        self.should_continue.clear()
        # Stop deleting tiles, while we're at it.
        self.on_delete_tiles(should_force_stop=True)


## Global window singleton.
window = None


def make_window(parent):
    global window
    window = MosaicWindow(parent, title="Mosaic view")
    window.center_canvas()


def transfer_camera_image():
    """Transfer a camera image to the mosaic."""
    window.transfer_camera_image()
