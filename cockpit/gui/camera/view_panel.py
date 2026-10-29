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

import wx

import cockpit.gui.gui_utils
import cockpit.gui.image_viewer.view_canvas
import cockpit.interfaces.stage_mover
import cockpit.util.threads
from cockpit import depot, events
from cockpit.experiment import experiment


## Default viewer dimensions.
VIEW_WIDTH, VIEW_HEIGHT = (512, 552)


class ViewPanel(wx.Panel):
    """Interface for a single camera display.

    It includes a button at the top to select which camera to use, a
    viewing area to display the image the camera sees, and a histogram
    at the bottom.

    It depends heavily on :mod:`cockpit.gui.image_viewer`.

    """

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        ## Handle of the current camera we're controlling.
        self.cur_camera = None
        ## Position of the currently displayed image - only updated on dbl-click.
        self.image_position = None

        column_sizer = wx.BoxSizer(wx.VERTICAL)
        ## Clickable text box showing the name of the currently-selected
        # camera.
        self.selector = wx.StaticText(
            self,
            style=wx.RAISED_BORDER | wx.ALIGN_CENTRE | wx.ST_NO_AUTORESIZE,
            size=(VIEW_WIDTH, 30),
        )
        self.selector.Bind(wx.EVT_LEFT_DOWN, self.on_selector)
        self.selector.SetDoubleBuffered(True)

        column_sizer.Add(self.selector, 0)

        ## Panel for holding our canvas.
        self.canvas_panel = wx.Panel(self)
        self.canvas_panel.SetMinSize((VIEW_WIDTH, VIEW_HEIGHT))
        column_sizer.Add(self.canvas_panel)

        self.SetSizerAndFit(column_sizer)

        ## Canvas we paint the camera's view onto. Created when we connect a
        # camera, and destroyed after.
        self.canvas = None

        self.disable()

        events.subscribe(events.FILTER_CHANGE, self.on_filter_change)
        self.Bind(wx.EVT_LEFT_DCLICK, self.on_mouse)

    def on_mouse(self, event):
        """User interacted with our current image.

        On double-click, we move the stage to centre the feature under the
        mouse.
        """
        if event.LeftDClick():
            if self.image_position is None:
                self.image_position = cockpit.interfaces.stage_mover.get_position()
            x, y = event.GetPosition() * self.GetContentScaleFactor()
            pixel_size = wx.GetApp().objectives.get_pixel_size()
            x0, y0 = self.canvas.glToIndices(0, 0)
            dy, dx = self.canvas.canvasToIndices(x, y)
            dx -= x0
            dy -= y0
            dx *= pixel_size
            dy *= pixel_size
            target = (self.image_position[0] - dx, self.image_position[1] - dy)
            cockpit.interfaces.stage_mover.go_to_xy(target)
        else:
            event.Skip()

    def on_selector(self, event):
        """User clicked on the selector.

        Pop up a menu to let them either activate a camera or, if we're already
        activated, deactivate the current one. We also let them set the
        camera's readout size here, if a camera is active.
        """
        # TODO: fix focus issue so that key bindings work immediately after camera enable.
        # Currently, have to mouse-over the bitmap area, or click in another window.
        menu = wx.Menu()
        if self.cur_camera is not None:
            item = menu.Append(
                -1, "Disable %s" % self.cur_camera.descriptive_name
            )
            self.Bind(
                wx.EVT_MENU, lambda event: self.cur_camera.toggle_state(), item
            )
        else:
            # Get all inactive cameras.
            cameras = wx.GetApp().depot.get_handlers_of_type(depot.CAMERA)
            cameras.sort(key=lambda c: c.descriptive_name)
            for camera in cameras:
                if not camera.get_is_enabled():
                    item = menu.Append(
                        -1, "Enable %s" % camera.descriptive_name
                    )
                    self.Bind(
                        wx.EVT_MENU,
                        lambda event, cam=camera: cam.toggle_state(),
                        item,
                    )
        cockpit.gui.gui_utils.place_menu_at_mouse(self, menu)

    def disable(self):
        """Deactivate the view."""
        self.selector.SetLabel("No camera")
        self.selector.SetBackgroundColour((180, 180, 180))
        self.selector.Refresh()
        if self.cur_camera is not None:
            # Wrap this in a try/catch since it will fail if the initial
            # camera enabling failed.
            events.unsubscribe(
                events.NEW_IMAGE % self.cur_camera.name, self.on_image
            )
            self.cur_camera = None
        if self.canvas is not None:
            # Destroy the canvas.
            self.canvas.clear(should_destroy=True)
            self.canvas = None

    def enable(self, camera):
        """Activate the view and connect to a data source."""
        self.selector.SetLabel(camera.descriptive_name)
        self.selector.SetBackgroundColour(camera.color)
        self.selector.Refresh()
        self.cur_camera = camera

        # NB the 512 here is the largest texture size our graphics card can
        # gracefully handle.
        self.canvas = cockpit.gui.image_viewer.view_canvas.ViewCanvas(
            self.canvas_panel, size=(VIEW_WIDTH, VIEW_HEIGHT)
        )
        self.canvas.SetSize((VIEW_WIDTH, VIEW_HEIGHT))
        self.canvas.reset_view()

        # Subscribe to new image events only after canvas is prepared.
        events.subscribe(events.NEW_IMAGE % self.cur_camera.name, self.on_image)

    def change_size(self, size=wx.Size(VIEW_WIDTH, VIEW_HEIGHT - 40)):
        """TODO: This needs revision, too many sizes are being set."""
        size_corrected = wx.Size(size[0], size[1] + 30)
        self.SetSize(size_corrected)
        self.SetMinSize(size_corrected)
        if self.canvas:
            self.canvas.SetSize(size)
            self.canvas.SetMinSize(size)
            self.canvas.clear()
            self.canvas.reset_view()
            # self.canvas.setSize(size)
        self.canvas_panel.SetSize(size)
        self.canvas_panel.SetMinSize(size)
        self.selector.SetSize(wx.Size(size[0], 30))
        self.selector.SetMinSize(wx.Size(size[0], 30))

    @cockpit.util.threads.call_in_main_thread
    def on_filter_change(self):
        """React to the drawer changing, by updating our labels and colors."""
        if self.get_is_enabled():
            self.selector.SetLabel(self.cur_camera.descriptive_name)
            self.selector.SetBackgroundColour(self.cur_camera.color)
            self.Refresh()

    def on_image(self, data, metadata, *args):
        """Receive a new image and send it to our canvas."""
        self.canvas.set_image(data)
        if not experiment.is_running():
            self.metadata = metadata
            self.image_position = self.metadata["image_position"]

    def get_is_enabled(self):
        """Return True if we currently display a camera."""
        return self.cur_camera is not None

    def get_scaling(self):
        """Get the black- and white-point for the view."""
        return self.canvas.get_scaling()

    def get_relative_scaling(self):
        """As above, but the relative values used to generate them instead."""
        return self.canvas.get_relative_scaling()

    def get_pixel_data(self):
        """Get the current pixel data for the view."""
        return (self.canvas.image_data, self.metadata)

    def __repr__(self):
        """Debugging: convert to string."""
        desc_string = ", disabled"
        if self.cur_camera is not None:
            desc_string = "for %s" % self.cur_camera.name
        return "<Camera ViewPanel %s>" % desc_string
