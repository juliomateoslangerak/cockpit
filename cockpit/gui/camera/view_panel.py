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
        self.curCamera = None
        ## Position of the currently displayed image - only updated on dbl-click.
        self.imagePos = None

        columnSizer = wx.BoxSizer(wx.VERTICAL)
        ## Clickable text box showing the name of the currently-selected
        # camera.
        self.selector = wx.StaticText(
            self,
            style=wx.RAISED_BORDER | wx.ALIGN_CENTRE | wx.ST_NO_AUTORESIZE,
            size=(VIEW_WIDTH, 30),
        )
        self.selector.Bind(wx.EVT_LEFT_DOWN, self.onSelector)
        self.selector.SetDoubleBuffered(True)

        columnSizer.Add(self.selector, 0)

        ## Panel for holding our canvas.
        self.canvasPanel = wx.Panel(self)
        self.canvasPanel.SetMinSize((VIEW_WIDTH, VIEW_HEIGHT))
        columnSizer.Add(self.canvasPanel)

        self.SetSizerAndFit(columnSizer)

        ## Canvas we paint the camera's view onto. Created when we connect a
        # camera, and destroyed after.
        self.canvas = None

        self.disable()

        events.subscribe(events.FILTER_CHANGE, self.onFilterChange)
        self.Bind(wx.EVT_LEFT_DCLICK, self.onMouse)

    def onMouse(self, event):
        """User interacted with our current image.

        On double-click, we move the stage to centre the feature under the
        mouse.
        """
        if event.LeftDClick():
            if self.imagePos is None:
                self.imagePos = cockpit.interfaces.stage_mover.getPosition()
            x, y = event.GetPosition() * self.GetContentScaleFactor()
            pixelSize = wx.GetApp().Objectives.GetPixelSize()
            x0, y0 = self.canvas.glToIndices(0, 0)
            dy, dx = self.canvas.canvasToIndices(x, y)
            dx -= x0
            dy -= y0
            dx *= pixelSize
            dy *= pixelSize
            target = (self.imagePos[0] - dx, self.imagePos[1] - dy)
            cockpit.interfaces.stage_mover.goToXY(target)
        else:
            event.Skip()

    def onSelector(self, event):
        """User clicked on the selector.

        Pop up a menu to let them either activate a camera or, if we're already
        activated, deactivate the current one. We also let them set the
        camera's readout size here, if a camera is active.
        """
        # TODO: fix focus issue so that key bindings work immediately after camera enable.
        # Currently, have to mouse-over the bitmap area, or click in another window.
        menu = wx.Menu()
        if self.curCamera is not None:
            item = menu.Append(
                -1, "Disable %s" % self.curCamera.descriptiveName
            )
            self.Bind(
                wx.EVT_MENU, lambda event: self.curCamera.toggleState(), item
            )
        else:
            # Get all inactive cameras.
            cameras = wx.GetApp().Depot.getHandlersOfType(depot.CAMERA)
            cameras.sort(key=lambda c: c.descriptiveName)
            for camera in cameras:
                if not camera.getIsEnabled():
                    item = menu.Append(
                        -1, "Enable %s" % camera.descriptiveName
                    )
                    self.Bind(
                        wx.EVT_MENU,
                        lambda event, cam=camera: cam.toggleState(),
                        item,
                    )
        cockpit.gui.gui_utils.placeMenuAtMouse(self, menu)

    def disable(self):
        """Deactivate the view."""
        self.selector.SetLabel("No camera")
        self.selector.SetBackgroundColour((180, 180, 180))
        self.selector.Refresh()
        if self.curCamera is not None:
            # Wrap this in a try/catch since it will fail if the initial
            # camera enabling failed.
            events.unsubscribe(
                events.NEW_IMAGE % self.curCamera.name, self.onImage
            )
            self.curCamera = None
        if self.canvas is not None:
            # Destroy the canvas.
            self.canvas.clear(shouldDestroy=True)
            self.canvas = None

    def enable(self, camera):
        """Activate the view and connect to a data source."""
        self.selector.SetLabel(camera.descriptiveName)
        self.selector.SetBackgroundColour(camera.color)
        self.selector.Refresh()
        self.curCamera = camera

        # NB the 512 here is the largest texture size our graphics card can
        # gracefully handle.
        self.canvas = cockpit.gui.image_viewer.view_canvas.ViewCanvas(
            self.canvasPanel, size=(VIEW_WIDTH, VIEW_HEIGHT)
        )
        self.canvas.SetSize((VIEW_WIDTH, VIEW_HEIGHT))
        self.canvas.resetView()

        # Subscribe to new image events only after canvas is prepared.
        events.subscribe(events.NEW_IMAGE % self.curCamera.name, self.onImage)

    def change_size(self, size=wx.Size(VIEW_WIDTH, VIEW_HEIGHT - 40)):
        """TODO: This needs revision, too many sizes are being set."""
        size_corrected = wx.Size(size[0], size[1] + 30)
        self.SetSize(size_corrected)
        self.SetMinSize(size_corrected)
        if self.canvas:
            self.canvas.SetSize(size)
            self.canvas.SetMinSize(size)
            self.canvas.clear()
            self.canvas.resetView()
            self.canvas.setSize(size)
        self.canvasPanel.SetSize(size)
        self.canvasPanel.SetMinSize(size)
        self.selector.SetSize(wx.Size(size[0], 30))
        self.selector.SetMinSize(wx.Size(size[0], 30))

    @cockpit.util.threads.callInMainThread
    def onFilterChange(self):
        """React to the drawer changing, by updating our labels and colors."""
        if self.getIsEnabled():
            self.selector.SetLabel(self.curCamera.descriptiveName)
            self.selector.SetBackgroundColour(self.curCamera.color)
            self.Refresh()

    def onImage(self, data, metadata, *args):
        """Receive a new image and send it to our canvas."""
        self.canvas.setImage(data)
        if not experiment.isRunning():
            self.metadata = metadata
            self.imagePos = self.metadata["imagePos"]

    def getIsEnabled(self):
        """Return True if we currently display a camera."""
        return self.curCamera is not None

    def getScaling(self):
        """Get the black- and white-point for the view."""
        return self.canvas.getScaling()

    def getRelativeScaling(self):
        """As above, but the relative values used to generate them instead."""
        return self.canvas.getRelativeScaling()

    def getPixelData(self):
        """Get the current pixel data for the view."""
        return (self.canvas.imageData, self.metadata)

    def __repr__(self):
        """Debugging: convert to string."""
        descString = ", disabled"
        if self.curCamera is not None:
            descString = "for %s" % self.curCamera.name
        return "<Camera ViewPanel %s>" % descString
