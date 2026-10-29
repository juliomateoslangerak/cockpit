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

import operator
import queue
import threading
import traceback
from collections.abc import Iterable

import numpy as np
import wx
import wx.glcanvas
from OpenGL.GL import *

import cockpit.gui
import cockpit.gui.dialogs.get_number_dialog
import cockpit.gui.freetype
import cockpit.gui.gui_utils
import cockpit.gui.mosaic.window
import cockpit.util.datadoc
import cockpit.util.threads
from cockpit import events


## @package cockpit.gui.image_viewer.viewCanvas
# This module provides a canvas for displaying camera images.

## Display height of the histogram, in pixels
HISTOGRAM_HEIGHT = 40

## Drag modes
DRAG_NONE, DRAG_CANVAS, DRAG_BLACKPOINT, DRAG_WHITEPOINT, DRAG_ROI = range(5)


class BaseGL:
    # Default vertex shader glsl source
    _VS = """
    #version 120
    attribute vec2 vXY;
    void main() {
        gl_Position = vec4(vXY, 1, 1);
        gl_FrontColor = gl_Color;
    }
    """
    _FS = None

    @staticmethod
    def _compile_shader(shaderType, source):
        """Compile a glsl shader stage."""
        shader = glCreateShader(shaderType)
        glShaderSource(shader, source)
        glCompileShader(shader)
        result = glGetShaderiv(shader, GL_COMPILE_STATUS)
        if not (result):
            raise RuntimeError(glGetShaderInfoLog(shader))
        return shader

    def get_shader(self):
        """Compile and link shader."""
        if not hasattr(self, "_shader"):
            self._shader = glCreateProgram()
            vs = self._compile_shader(GL_VERTEX_SHADER, self._VS)
            glAttachShader(self._shader, vs)
            if self._FS is not None:
                fs = self._compile_shader(GL_FRAGMENT_SHADER, self._FS)
                glAttachShader(self._shader, fs)
            glBindAttribLocation(self._shader, 0, "vXY")
            glLinkProgram(self._shader)
        return self._shader


class Image(BaseGL):
    """An class for rendering grayscale images from image data.

    GL textures are generated once. GL stores textures as floats with a range of
    0 to 1. We use the data.min and data.max of the incoming data to fill this
    range to prevent loss of detail due to quantisation when rendering low dynamic
    range images.
    """

    # Vertex shader glsl source
    _VS = """
    #version 120
    attribute vec2 vXY;
    uniform float zoom;
    uniform float angle;
    uniform vec2 pan;

    void main() {
        gl_TexCoord[0] = gl_MultiTexCoord0;
        gl_Position = vec4(zoom * (vXY + pan), 1., 1.);
    }
    """
    # Fragment shader glsl source.
    _FS = """
    #version 120
    uniform sampler2D tex;
    uniform float scale;
    uniform float offset;
    uniform bool show_clip;

    void main()
    {
        vec4 lum = clamp(offset + texture2D(tex, gl_TexCoord[0].st) / scale, 0., 1.);
        if (show_clip) {
            gl_FragColor = vec4(0., 0., lum.r == 0, 1.) + vec4(1., lum.r < 1., 1., 1.) * lum.r;
        } else {
            gl_FragColor = vec4(lum.r, lum.r, lum.r, 1.);
        }
    }
    """

    def __init__(self):
        # Maximum texture edge size
        self._max_tex_edge = 0
        # Textures used to display this image.
        self._textures = []
        # New data flag
        self._update = False
        # Geometry as number of textures along each axis.
        self.shape = (0, 0)
        ## Should we use colour to indicate range clipping?
        self.clip_highlight = False
        # Data
        self._data = None
        # Minimum and maximum data value - used for setting greyscale range.
        self.data_peak_to_peak = 1
        self.data_min = 0
        # Grayscale clipping points
        self.v_max = 1
        self.v_min = 0

    @property
    def scale(self):
        return (self.v_max - self.v_min) / (self.data_peak_to_peak)

    @property
    def offset(self):
        return -(self.v_min - self.data_min) / ((self.data_peak_to_peak * self.scale) or 1)

    def __del__(self):
        """Clean up textures."""
        try:
            # On exit, textures may have already been cleaned up.
            glDeleteTextures(len(self._textures), self._textures)
        except:
            pass

    def autoscale(self):
        """Fit grayscale to range covered by data."""
        self.v_min = float(self._data.min())
        self.v_max = float(self._data.max())

    def get_display_range(self):
        return (self.v_min, self.v_max)

    def set_display_range(self, vmin, vmax):
        """Set offset and scaling given clip points."""
        self.v_min = float(vmin)
        self.v_max = float(vmax)

    def set_data(self, data):
        self._data = data
        self._update = True

    def toggle_clip_highlight(self, event=None):
        self.clip_highlight = not self.clip_highlight

    def _create_textures(self):
        """Convert data to textures.

        Needs GL context to be set prior to call, and should only
        be called in the main thread."""
        if self._data is None:
            return
        self._max_tex_edge = glGetInteger(GL_MAX_TEXTURE_SIZE)
        data = self._data
        glPixelStorei(GL_UNPACK_SWAP_BYTES, False)
        # Ensure the right number of textures available.
        n_x = int(np.ceil(data.shape[1] / self._max_tex_edge))
        n_y = int(np.ceil(data.shape[0] / self._max_tex_edge))
        self.shape = (n_x, n_y)
        n_tex = n_x * n_y
        if n_tex > len(self._textures):
            textures = glGenTextures(n_tex - len(self._textures))
            if isinstance(textures, Iterable):
                self._textures.extend(textures)
            else:
                self._textures.append(textures)
        elif n_tex < len(self._textures):
            glDeleteTextures(len(self._textures) - n_tex)
        if n_tex == 1:
            # Data will fit into a single texture.
            # Do we need to round these up to a power of 2?
            t_y, t_x = data.shape
        else:
            # Need to use multiple textures to store data.
            t_x = t_y = self._max_tex_edge
        self.data_peak_to_peak = np.ptp(data)
        self.data_min = data.min()
        if self.data_peak_to_peak < 1e-6:
            self.data_peak_to_peak = 1
        for i, tex in enumerate(self._textures):
            x_off = t_x * (i % n_x)
            y_off = t_y * (i // n_x)
            subdata = (
                data[
                    y_off : min(data.shape[0], y_off + t_y),
                    x_off : min(data.shape[1], x_off + t_x),
                ].astype(np.float32)
                - self.data_min
            ) / self.data_peak_to_peak
            glBindTexture(GL_TEXTURE_2D, tex)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE)
            glTexImage2D(
                GL_TEXTURE_2D, 0, GL_RED, t_x, t_y, 0, GL_RED, GL_FLOAT, None
            )
            glTexSubImage2D(
                GL_TEXTURE_2D,
                0,
                0,
                0,
                subdata.shape[1],
                subdata.shape[0],
                GL_RED,
                GL_FLOAT,
                subdata,
            )
        self._update = False

    def draw(self, pan=(0, 0), zoom=1):
        """Render the textures. Caller must set context prior to call."""
        if self._data is None:
            return
        elif self._update:
            self._create_textures()
        shader = self.get_shader()
        glUseProgram(shader)
        # Vertical and horizontal modifiers for non-square images.
        h_lim = self._data.shape[1] / max(self._data.shape)
        v_lim = self._data.shape[0] / max(self._data.shape)
        # Number of x and y textures.
        n_x, n_y = self.shape
        # Quad dimensions for one texture.
        d_x = 2 * h_lim / n_x
        d_y = 2 * v_lim / n_y
        if len(self._textures) > 1:
            t_x = t_y = self._max_tex_edge
            # xy & zoom correction for incompletely-filled textures at upper & right edges.
            x_corr = ((t_x - self._data.shape[1]) % t_x) / (n_x * t_x)
            y_corr = ((t_y - self._data.shape[0]) % t_y) / (n_y * t_y)
            zoom_corr = self._data.shape[1] / (n_x * t_x)
            zoom_corr = max(zoom_corr, self._data.shape[0] / (n_y * t_y))
            zoom = zoom / zoom_corr
            pan = (zoom_corr * pan[0] + x_corr, zoom_corr * pan[1] + y_corr)
        # Update shader parameters
        glUniform2f(glGetUniformLocation(shader, "pan"), pan[0], pan[1])
        glUniform1i(glGetUniformLocation(shader, "tex"), 0)
        glUniform1f(glGetUniformLocation(shader, "scale"), self.scale)
        glUniform1f(glGetUniformLocation(shader, "offset"), self.offset)
        glUniform1f(glGetUniformLocation(shader, "zoom"), zoom)
        glUniform1i(
            glGetUniformLocation(shader, "show_clip"), self.clip_highlight
        )
        # Render
        glEnable(GL_TEXTURE_2D)
        glEnableClientState(GL_VERTEX_ARRAY)
        glEnableClientState(GL_TEXTURE_COORD_ARRAY)
        # nested loops are still quicker than itertools.product
        for j in range(n_y):
            # i and j are indices that determine left and bottom quad co-ords.
            # ii and jj determine upper quad co-ords, and may be fractional for
            # quads at the top or right of the image.
            if j > 0 and j == n_y - 1:
                jj = self._data.shape[0] / self._max_tex_edge
            else:
                jj = j + 1
            for i in range(n_x):
                if i > 0 and i == n_x - 1:
                    ii = self._data.shape[1] / self._max_tex_edge
                else:
                    ii = i + 1
                # Arrays used to create textures have top left at [0,0].
                # GL co-ords run *bottom* left to top right, so need to invert
                # vertical co-ords.
                glVertexPointerf(
                    [
                        (-h_lim + i * d_x, -v_lim + jj * d_y),
                        (-h_lim + ii * d_x, -v_lim + jj * d_y),
                        (-h_lim + ii * d_x, -v_lim + j * d_y),
                        (-h_lim + i * d_x, -v_lim + j * d_y),
                    ]
                )
                glTexCoordPointer(
                    2,
                    GL_FLOAT,
                    0,
                    [
                        (0, 0),
                        (ii % 1 or 1, 0),
                        (ii % 1 or 1, jj % 1 or 1),
                        (0, jj % 1 or 1),
                    ],
                )
                glBindTexture(GL_TEXTURE_2D, self._textures[j * n_x + i])
                glDrawArrays(GL_QUADS, 0, 4)
        glDisable(GL_TEXTURE_2D)
        glDisableClientState(GL_TEXTURE_COORD_ARRAY)
        glUseProgram(0)


class Histogram(BaseGL):
    def __init__(self):
        self.bins = None
        self.counts = None
        self.lower_bound = None
        self.upper_bound = None
        self.lower_threshold = None
        self.upper_threshold = None

    def data_to_gl(self, val):
        return -1 + 2 * (val - self.lower_bound) / (
                (self.upper_bound - self.lower_bound) or 1
        )

    def gl_to_data(self, x):
        return self.lower_bound + ((self.upper_bound - self.lower_bound) or 1) * (x + 1) / 2

    def set_data(self, data):
        # Calculate histogram.
        # Use shifted average histogram to avoid binning artefacts.
        # generate set of m histograms
        #   each has class width h
        #   start points 0, h/m, 2h/m, ..., (m-1)h/m
        # 1 < m < 64
        # sum to average
        if self.lower_bound is None:
            self.lower_bound = data.min()
        if self.upper_bound is None:
            self.upper_bound = data.max()
        if self.lower_threshold is None:
            self.lower_threshold = self.lower_bound
        if self.upper_threshold is None:
            self.upper_threshold = self.upper_bound
        nb_bins = 64
        m = 4
        self.bins = np.linspace(data.min(), data.max(), nb_bins)
        self.counts = np.zeros(nb_bins)
        h = self.bins[1] - self.bins[0]
        for i in range(m):
            these = np.bincount(
                np.digitize(data.flat, self.bins + i * h / m, right=True),
                minlength=nb_bins,
            )
            self.counts += these[0:nb_bins]

    def draw(self):
        if self.counts is None:
            return
        binw = self.bins[1] - self.bins[0]
        self.lower_bound = min(self.bins.min() - binw, self.lower_threshold - binw)
        self.upper_bound = max(self.bins.max() + binw, self.upper_threshold + binw)
        w = self.upper_bound - self.lower_bound
        glUseProgram(self.get_shader())
        v = []
        for x, y in zip(self.bins, self.counts):
            x0 = self.data_to_gl(x)
            x1 = self.data_to_gl(x + binw)
            h = -1 + 2 * y / (self.counts.max() or 1)
            v.extend([(x0, -1), (x0, h), (x1, h), (x1, -1)])
        glEnableClientState(GL_VERTEX_ARRAY)
        glVertexPointerf(v)
        glColor(0.8, 0.8, 0.8, 1)
        glDrawArrays(GL_QUADS, 0, len(v))
        glColor(1, 0, 0, 1)
        xl = self.data_to_gl(self.lower_threshold)
        xu = self.data_to_gl(self.upper_threshold)
        dx = 0.05
        glLineWidth(2)
        glVertexPointerf([(xl + dx, -1), (xl, -1), (xl, 1), (xl + dx, 1)])
        glDrawArrays(GL_LINE_STRIP, 0, 4)
        glVertexPointerf([(xu - dx, -1), (xu, -1), (xu, 1), (xu - dx, 1)])
        glDrawArrays(GL_LINE_STRIP, 0, 4)
        glUseProgram(0)


class ViewCanvas(wx.glcanvas.GLCanvas):
    """This class handles displaying multi-channel 2D images.

    Most of the actual drawing logic is handled in the image.Image class. It
    can handle arbitrarily-sized images, by cutting them up into parcels and
    tiling them together.
    """

    def __init__(self, parent, *args, **kwargs):
        """Instantiate."""
        super().__init__(parent, *args, **kwargs)

        self.image = Image()
        self.histogram = Histogram()

        ## Menu - keep reference to store state of toggle buttons.
        # Must be created after self.image.
        self._menu = wx.Menu()
        for label, action in self.get_menu_actions():
            if not label:
                self._menu.AppendSeparator()
                continue
            id = wx.NewIdRef()
            if label.lower().startswith("toggle"):
                self._menu.AppendCheckItem(id, label)
            else:
                self._menu.Append(id, label)
            self.Bind(
                wx.EVT_MENU, lambda event, action=action: action(), id=id
            )

        # Canvas geometry - will be set by InitGL, or setSize.
        self.w, self.h = None, None

        ## We set this to false if there's an error, to prevent OpenGL
        # error spew.
        self.should_draw = True

        ## Should we show a crosshair (used for alignment)?
        self.show_crosshair = False
        ##default to enabled synced view, set to flase then toggle it to
        ##do the correct subscriptions.
        self.sync_views = False
        self.toggle_sync_views()

        ## Queue of incoming images that we need to either display or discard.
        self.image_queue = queue.Queue()
        ## Current image we're working with.
        self.image_data = None
        ## Event that signals that we've finished drawing the current image.
        self.draw_event = threading.Event()
        # This spawns a new thread.
        self.process_images()
        ## Percentile scaling of min/max based on our histogram.
        self.black_point, self.white_point = 0.0, 1.0

        ## Size of image we've received, which we use for determining
        # scale.
        self.image_shape = None

        ## Overall scaling factor, separate from the above.
        self.zoom = 1.0
        ## Current mouse position
        self.cur_mouse_x = self.cur_mouse_y = None

        ## Mouse position as of most recent frame, when dragging.
        self.mouse_drag_x = self.mouse_drag_y = None

        ## Pan translation factor
        self.pan_x = 0
        self.pan_y = 0

        ## ROI
        self.roi = None  # Current roi
        self.roi_drag = None  # ROI currently being defined
        self.defining_roi = False  # Are we defining the ROI
        self.defined_roi = False  # New ROI defined but not grabbed.
        self.shift_down = False  # shift key drags a square ROI

        ## What kind of dragging we're doing.
        self.drag_mode = DRAG_NONE

        ## Whether or not we've done some one-time initialization work.
        self.have_inited_gl = False

        ## WX context that we set when we need to do GL operations.
        self.context = wx.glcanvas.GLContext(self)

        ## Font for text rendering
        self.face = cockpit.gui.freetype.Face(self, 18)

        self.Bind(wx.EVT_PAINT, self.on_paint)
        # Do nothing, to prevent flickering
        self.Bind(wx.EVT_ERASE_BACKGROUND, lambda event: 0)
        self.Bind(wx.EVT_MOUSE_EVENTS, self.on_mouse)
        self.Bind(wx.EVT_MOUSEWHEEL, self.on_mouse_wheel)
        self.Bind(wx.EVT_DPI_CHANGED, self.on_dpi_change)

        # Right click also creates context menu event, which will pass up
        # if unhandled. Bind it to None to prevent the main window
        # context menu being displayed after our own.
        self.Bind(wx.EVT_CONTEXT_MENU, lambda event: None)
        self.painting = False

        # shift key to make ROI square requirse knowing if the key is down
        self.Bind(wx.EVT_KEY_DOWN, self.on_key_down)
        self.Bind(wx.EVT_KEY_UP, self.on_key_up)

        # Initialise FFT variables
        self.show_fft = False

    def on_dpi_change(self, event):
        # rescale the glcanvas object if needed
        self.w, self.h = self.GetClientSize() * self.GetContentScaleFactor()

    def on_mouse_wheel(self, event):
        # Only respond if event originated within window.
        p = event.GetPosition()
        s = self.GetSize() * self.GetContentScaleFactor()
        if any(
            map(
                operator.or_,
                map(operator.gt, p, s),
                map(operator.lt, p, (0, 0)),
            )
        ):
            return
        rotation = event.GetWheelRotation()
        if not rotation:
            return
        factor = rotation / 1000.0
        x, y = event.GetLogicalPosition(wx.ClientDC(self))
        w, h = self.GetClientSize() * self.GetContentScaleFactor()
        h -= HISTOGRAM_HEIGHT * self.GetContentScaleFactor()
        gl_x = -(2 * (x / w) - 1) / self.zoom
        gl_y = (2 * (y / h) - 1) / self.zoom
        new_zoom = self.zoom * (1 + factor)
        if new_zoom < 0.001:
            factor = -1 + 0.001 / self.zoom
            new_zoom = 0.001
        self.zoom = new_zoom
        self.pan_x += factor * gl_x
        self.pan_y += factor * gl_y
        events.publish(events.SYNCED_VIEW, self.pan_x, self.pan_y, self.zoom)
        self.Refresh()

    def init_gl(self):
        self.w, self.h = self.GetClientSize() * self.GetContentScaleFactor()
        self.SetCurrent(self.context)
        glClearColor(0.3, 0.3, 0.3, 0.0)  ## background color

        self.have_inited_gl = True

    @cockpit.util.threads.call_in_main_thread
    def clear(self, should_destroy=False):
        """Stop displaying anything.

        Optionally destroy the canvas at the end.
        """
        # Clear out the queue of images.
        while True:
            try:
                self.image_queue.get_nowait()
            except queue.Empty:
                break
        self.image_data = None
        self.image_shape = None
        if should_destroy:
            self.should_draw = False
            self.Destroy()
        else:
            self.Refresh()

    def set_image(self, new_image):
        """Receive a new image.

        This will trigger process_images(), below, to actually display the
        image.
        """
        self.image_queue.put_nowait(new_image)
        self.defined_roi = False  # new image will have new ROI.

    @cockpit.util.threads.call_in_new_thread
    def process_images(self):
        """Consume images out of self.image_queue and either display them or
        discard them.

        Because images can arrive very rapidly at times, we want to ensure that
        we don't jam up -- if several images arrive while we process one image,
        then the extras get discarded.
        """
        while self.should_draw:
            # Grab all images out of the queue; we'll use the most recent one.
            new_image = self.image_queue.get()
            while not self.image_queue.empty():
                new_image = self.image_queue.get_nowait()
            # We want to autoscale to the image if it's our first one.
            is_first_image = self.image_data is None
            self.image_data = new_image
            # When the image shape changes, we reset back to filling the
            # display with the image.
            should_reset_view = self.image_shape != new_image.shape
            self.image_shape = new_image.shape
            self.histogram.set_data(new_image)
            if self.show_fft:
                self.image.set_data(
                    np.log(
                        np.abs(np.fft.fftshift(np.fft.fft2(self.image_data)))
                        + 1e-16
                    )
                )
            else:
                self.image.set_data(new_image)
            if should_reset_view:
                self.reset_view()
            if is_first_image:
                self.image.autoscale()
            wx.CallAfter(self.Refresh)
            # Wait for the image to be drawn before we do anything more.
            self.draw_event.wait(timeout=1)
            self.draw_event.clear()

    def get_scaling(self):
        """Return the blackpoint and whitepoint (i.e. the pixel values which
        are displayed as black and white, respectively)."""
        if self.image_data is None:
            # No image to operate on yet.
            return (None, None)
        else:
            return self.image.get_display_range()

    def get_relative_scaling(self):
        """As above, but the values used to calculate them instead of the
        absolute pixel values (e.g.

        (.1, .9) instead of (100, 400).
        """
        return (self.black_point, self.white_point)

    def on_paint(self, event):
        """@cockpit.util.threads.call_in_main_thread."""
        if not self.should_draw:
            return

        try:
            # Unused, but wx requires we create an instance of PaintDC.
            dc = wx.PaintDC(self)
        except:
            return

        if not self.have_inited_gl:
            self.init_gl()

        if self.painting:
            print("Concurrent - returning")
            return

        try:
            hist_height = int(HISTOGRAM_HEIGHT * self.GetContentScaleFactor())
            self.painting = True
            self.SetCurrent(self.context)
            glClear(GL_COLOR_BUFFER_BIT)
            glViewport(0, hist_height, self.w, self.h - hist_height)
            self.image.draw(pan=(self.pan_x, self.pan_y), zoom=self.zoom)
            if self.show_crosshair:
                self.draw_crosshair()
            if self.defining_roi:
                self.draw_roi()
            if self.defined_roi:
                self.draw_roi()
            glViewport(0, 0, self.w, hist_height // 2)
            self.histogram.draw()
            glColor(0, 1, 0, 1)

            glViewport(0, 0, self.w, self.h)
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            glOrtho(0, self.w, 0, self.h, 1.0, -1.0)
            glTranslatef(0, (hist_height) / 2 + 2, 0)
            try:
                self.face.render(
                    "%d [%-10d %10d] %d"
                    % (
                        self.image.data_min,
                        self.histogram.lower_threshold,
                        self.histogram.upper_threshold,
                        self.image.data_min + self.image.data_peak_to_peak,
                    )
                )
            except:
                pass
            glPopMatrix()

            # self.drawHistogram()

            # glFlush()
            self.SwapBuffers()
            self.draw_event.set()
        except Exception as e:
            print("Error drawing view canvas:", e)
            traceback.print_exc()
            # self.should_draw = False
        finally:
            self.painting = False

    @cockpit.util.threads.call_in_main_thread
    def draw_crosshair(self):
        glColor3f(0, 255, 255)
        glVertexPointerf(
            [
                (-1, self.zoom * self.pan_y),
                (1, self.zoom * self.pan_y),
                (self.zoom * self.pan_x, -1),
                (self.zoom * self.pan_x, 1),
            ]
        )
        glDrawArrays(GL_LINES, 0, 4)

    @cockpit.util.threads.call_in_main_thread
    def draw_roi(self):
        if self.roi_drag:
            glColor3f(255, 0, 0)

            # Get raw gl co-ordinates
            v = [
                self.indices_to_gl(self.roi_drag[1], self.roi_drag[0]),
                self.indices_to_gl(
                    self.roi_drag[1] + self.roi_drag[3], self.roi_drag[0]
                ),
                self.indices_to_gl(
                    self.roi_drag[1] + self.roi_drag[3],
                    self.roi_drag[0] + self.roi_drag[2],
                ),
                self.indices_to_gl(
                    self.roi_drag[1], self.roi_drag[0] + self.roi_drag[2]
                ),
            ]

            # Correct co-ordinates for zoom and pan
            v_zoompan = [
                (
                    (p[0] + self.pan_x) * self.zoom,
                    (p[1] + self.pan_y) * self.zoom,
                )
                for p in v
            ]

            # Draw roi box
            glVertexPointerf(v_zoompan)
            glDrawArrays(GL_LINE_LOOP, 0, 4)

    def set_size(self, size):
        """Update the size of the canvas by scaling it."""
        if self.image_data is not None:
            self.w, self.h = size * self.GetContentScaleFactor()
        self.Refresh(False)

    def on_mouse(self, event):
        if self.image_shape is None:
            return
        self.cur_mouse_x, self.cur_mouse_y = (
            event.GetPosition() * self.GetContentScaleFactor()
        )
        self.update_mouse_info(self.cur_mouse_x, self.cur_mouse_y)
        if event.LeftDClick():
            # Explicitly skip EVT_LEFT_DCLICK for parent to handle.
            event.ResumePropagation(2)
            event.Skip()
        elif event.LeftDown():
            # Started dragging
            self.mouse_drag_x, self.mouse_drag_y = self.cur_mouse_x, self.cur_mouse_y
            self.mouse_left_down_x, self.mouse_left_down_y = self.cur_mouse_x, self.cur_mouse_y
            black_point_x = (
                0.5
                * (1 + self.histogram.data_to_gl(self.histogram.lower_threshold))
                * self.w
            )
            white_point_x = (
                0.5
                * (1 + self.histogram.data_to_gl(self.histogram.upper_threshold))
                * self.w
            )
            # Set drag mode based on current window position
            if self.h - self.cur_mouse_y >= (
                HISTOGRAM_HEIGHT * self.GetContentScaleFactor() * 2
            ):
                if self.defining_roi:
                    self.drag_mode = DRAG_ROI
                else:
                    self.drag_mode = DRAG_CANVAS
            elif abs(self.cur_mouse_x - black_point_x) < abs(
                    self.cur_mouse_x - white_point_x
            ):
                self.drag_mode = DRAG_BLACKPOINT
            else:
                self.drag_mode = DRAG_WHITEPOINT
        elif event.LeftIsDown():
            # Drag mouse. Different behaviors depending on drag mode.
            if self.drag_mode == DRAG_CANVAS:
                # Pan view about.
                # Window coordinates are upside-down compared to what the
                # user expects.
                self.mod_pan(
                    self.cur_mouse_x - self.mouse_drag_x,
                    self.mouse_drag_y - self.cur_mouse_y,
                )
            elif self.drag_mode in [DRAG_BLACKPOINT, DRAG_WHITEPOINT]:
                gl_x = -1 + 2 * self.cur_mouse_x / self.w
                threshold = self.histogram.gl_to_data(gl_x)
                if self.drag_mode == DRAG_BLACKPOINT:
                    self.histogram.lower_threshold = threshold
                    self.image.v_min = threshold
                else:
                    self.histogram.upper_threshold = threshold
                    self.image.v_max = threshold
            elif self.drag_mode == DRAG_ROI:
                # Get co-ordinates in canvas units
                coords_x = [self.mouse_drag_x, self.mouse_left_down_x]
                coords_y = [self.mouse_drag_y, self.mouse_left_down_y]
                roi_x_min, roi_y_min = min(coords_x), min(coords_y)
                roi_x_max, roi_y_max = max(coords_x), max(coords_y)
                # Convert to data indices
                roi_min_ind = self.canvas_to_indices(roi_x_min, roi_y_min)
                roi_max_ind = self.canvas_to_indices(roi_x_max, roi_y_max)

                if self.shift_down:
                    # shift is down so force square ROI
                    # Get size of roi
                    roi_max_size = max(
                        (
                            roi_max_ind[0] - roi_min_ind[0],
                            roi_max_ind[1] - roi_min_ind[1],
                        )
                    )
                    roi_size = (roi_max_size, roi_max_size)
                else:
                    roi_size = (
                        roi_max_ind[0] - roi_min_ind[0],
                        roi_max_ind[1] - roi_min_ind[1],
                    )

                # Set roi (left, top, width, height)
                self.roi_drag = (
                    roi_min_ind[1],
                    roi_min_ind[0],
                    roi_size[1],
                    roi_size[0],
                )

            self.mouse_drag_x = self.cur_mouse_x
            self.mouse_drag_y = self.cur_mouse_y
        elif event.RightDown():
            cockpit.gui.gui_utils.place_menu_at_mouse(self, self._menu)
        elif event.LeftUp():
            if self.defining_roi:
                camera = self.Parent.Parent.cur_camera

                # Set ROI in camera, correcting for current roi
                if self.roi:
                    roi = (
                        self.roi[0] + self.roi_drag[0],
                        self.roi[1] + self.roi_drag[1],
                        self.roi_drag[2],
                        self.roi_drag[3],
                    )
                else:
                    roi = self.roi_drag

                camera.set_roi(roi)
                events.publish(events.UPDATE_ROI, camera.name)

                self.roi = roi
                self.defining_roi = False
                self.defined_roi = True
        elif event.Entering() and self.TopLevelParent.IsActive():
            self.SetFocus()
        else:
            event.Skip()

        # In case current mouse position has changed enough to require
        # redrawing the histogram. A bit wasteful of resources, this.
        wx.CallAfter(self.Refresh)

    def get_menu_actions(self):
        """Generate a list of (label, action) tuples to use for generating
        menus."""
        return [
            ("Reset view", self.reset_view),
            ("Set histogram parameters", self.on_set_histogram),
            ("Toggle clip highlighting", self.image.toggle_clip_highlight),
            ("", None),
            ("Set ROI", self.on_define_roi),
            ("Clear ROI", self.on_clear_roi),
            ("", None),
            ("Toggle alignment crosshair", self.toggle_crosshair),
            ("Toggle sync view", self.toggle_sync_views),
            ("Toggle FFT mode", self.toggle_fft),
            ("", None),
            (
                "Send image to mosaic",
                cockpit.gui.mosaic.window.transferCameraImage,
            ),
            ("Save image", self.save_data),
        ]

    def on_set_histogram(self, event=None):
        """Let the user specify the blackpoint and whitepoint for image scaling."""
        values = cockpit.gui.dialogs.get_number_dialog.get_many_numbers_from_user(
            parent=self,
            title="Set histogram scale parameters",
            prompts=["Blackpoint", "Whitepoint"],
            defaultValues=[self.histogram.lower_threshold, self.histogram.upper_threshold],
        )
        values = [float(v) for v in values]
        self.image.v_min = self.histogram.lower_threshold = values[0]
        self.image.v_max = self.histogram.upper_threshold = values[1]
        self.Refresh()

    def toggle_sync_views(self, event=None):
        self.sync_views = not self.sync_views
        if self.sync_views:
            events.subscribe(
                events.SYNCED_VIEW,
                self.set_view,
            )
        else:
            events.unsubscribe(
                events.SYNCED_VIEW,
                self.set_view,
            )

    def on_define_roi(self, event=None):
        self.roi_drag = None
        self.defining_roi = True

    def on_clear_roi(self, event=None):
        camera = self.Parent.Parent.cur_camera
        sensor_shape = camera.get_sensor_shape()
        roi = (0, 0, sensor_shape[0], sensor_shape[1])
        camera.set_roi(roi)
        self.roi = roi
        self.defined_roi = False
        events.publish(events.UPDATE_ROI, camera.name)
        self.Refresh()

    def on_key_down(self, event):
        keycode = event.GetKeyCode()
        if keycode == wx.WXK_SHIFT:
            self.shift_down = True
        event.Skip()
        self.Refresh()

    def on_key_up(self, event):
        keycode = event.GetKeyCode()
        if keycode == wx.WXK_SHIFT:
            self.shift_down = False
        event.Skip()
        self.Refresh()

    def toggle_crosshair(self, event=None):
        self.show_crosshair = not (self.show_crosshair)

    def toggle_fft(self, event=None):
        if self.show_fft:
            self.show_fft = False
            self.image.set_data(self.image_data)
        else:
            self.show_fft = True
            self.image.set_data(
                np.log(
                    np.abs(np.fft.fftshift(np.fft.fft2(self.image_data)))
                    + 1e-16
                )
            )

    def canvas_to_gl(self, x, y):
        """Convert window co-ordinates to gl co-ordinates."""
        gl_x = (-1 + 2 * x / self.w - self.pan_x * self.zoom) / self.zoom
        gl_y = (
            -(
                    -1
                    + 2
                    * y
                    / (self.h - (HISTOGRAM_HEIGHT * self.GetContentScaleFactor()))
                    + self.pan_y * self.zoom
            )
            / self.zoom
        )
        return (gl_x, gl_y)

    def gl_to_indices(self, gl_x, gl_y):
        """Convert gl co-ordinates to indices into the data.

        Note: pass in x,y, but returns row-major data_y, data_x
        """
        # Vertical and horizontal modifiers for non-square images.
        h_lim = self.image_shape[1] / max(self.image_shape)
        v_lim = self.image_shape[0] / max(self.image_shape)
        data_x = int((1 + gl_x / h_lim) * self.image_shape[1] // 2)
        data_y = int(
            self.image_shape[0] - ((1 + gl_y / v_lim) * self.image_shape[0] // 2)
        )
        data_x_clamped = max(0, min(data_x, self.image_shape[1]))
        data_y_clamped = max(0, min(data_y, self.image_shape[0]))
        return (data_y_clamped, data_x_clamped)

    def indices_to_gl(self, data_y, data_x):
        """Convert data indices to gl co-ordinates.

        Note: pass in row-major data_y, data_x, but returns x,y
        """
        gl_x = 2 / self.image_shape[1] * data_x - 1
        gl_y = 2 / self.image_shape[0] * (self.image_shape[0] - data_y) - 1

        return (gl_x, gl_y)

    def canvas_to_indices(self, x, y):
        """Convert window co-ordinates to indices into the data."""
        return self.gl_to_indices(*self.canvas_to_gl(x, y))

    def update_mouse_info(self, x, y):
        """Display information on the pixel under the mouse at the given
        position."""
        # Test that all required values have been populated. Use any(...),
        # because ```if None in [...]:``` will throw an exception when an
        # element in the list is an array with more than one element.
        if any(
            req is None
            for req in [self.image_data, self.image_shape, self.w, self.h]
        ):
            return
        # First we have to convert from screen- to data-coordinates.
        coords = np.array(self.canvas_to_indices(x, y), dtype=np.uint)
        shape = np.array(self.image_shape, dtype=np.uint)
        if (coords < shape).all() and (coords >= 0).all():
            value = self.image_data[coords[0], coords[1]]
            events.publish(events.IMAGE_PIXEL_INFO, coords[::-1], value)

    def mod_pan(self, dx, dy):
        """Modify our panning amount by the provided factor."""
        self.pan_x += 2 * dx / (self.w * self.zoom)
        self.pan_y += 2 * dy / (self.h * self.zoom)
        events.publish(events.SYNCED_VIEW, self.pan_x, self.pan_y, self.zoom)
        self.Refresh(False)

    def reset_view(self):
        """Reset our view mods."""
        if self.image_shape is None:
            # No image to work with.
            return
        self.pan_x = 0
        self.pan_y = 0
        self.zoom = 1.0
        self.Refresh(False)

    def set_view(self, pan_x, pan_y, zoom):
        self.pan_x = pan_x
        self.pan_y = pan_y
        self.zoom = zoom
        # setView may be called by the eventloop though the ViewCanvas (self) has been already destroyed
        if self:
            self.Refresh(False)

    def reset_pixel_scale(self):
        self.image.autoscale()
        (
            self.histogram.lower_threshold,
            self.histogram.upper_threshold,
        ) = self.image.get_display_range()
        self.Refresh()

    def save_data(self, evt=None):
        with wx.FileDialog(
            self,
            "Save image",
            wildcard="DV files (*.dv)|*.dv",
            style=wx.FD_SAVE | wx.FD_OVERWRITE_PROMPT,
        ) as file_dialog:
            if file_dialog.ShowModal() != wx.ID_OK:
                return
            path = file_dialog.GetPath()
        # TODO: add XYsize and wavelength to saved data. These can be passed as
        # kwargs, but the way per-camera pixel sizes are handled needs to be
        # addressed first. See issue #538.
        if self.Parent.Parent.cur_camera is not None:
            metadata = self.Parent.Parent.metadata
            wavelengths = [
                metadata["wavelength"],
            ]
            xy_size = metadata["pixel_size"]
            xyz_pos = metadata["image_position"]
            exposure_time = metadata["exposure_time"]
            em_wavelength = metadata["wavelength"]
            ex_wavelength = metadata["ex_wavelength"]
            lens_id = metadata["lens_id"]

            # setup a single plane of extended header metadata
            ## Size of one plane's worth of metadata in the extended header.
            num_integers = 8
            num_floats = 32

            int_metadata_buffers = np.zeros(num_integers, dtype=np.int32)
            float_metadata_buffers = np.zeros(num_floats, dtype=np.float32)
            float_metadata_buffers[12] = 1.0  # intensity scaling
            float_metadata_buffers[2:5] = xyz_pos[0:3]
            float_metadata_buffers[5] = np.min(self.image_data)
            float_metadata_buffers[6] = np.max(self.image_data)
            float_metadata_buffers[7] = np.mean(self.image_data)
            float_metadata_buffers[8] = exposure_time
            float_metadata_buffers[10] = ex_wavelength
            float_metadata_buffers[11] = em_wavelength

            # spec for extended header
            # 8 32bit signed integers, often are all set to zero.
            # Followed by 32 32bit floats. We only what the first 14 are:
            # 0 photosensor reading (typically in mV)
            # 1 elapsed time (seconds since experiment began)
            # 2 x stage coordinates
            # 3 y stage coordinates
            # 4 z stage coordinates
            # 5 minimum intensity
            # 6 maximum intensity
            # 7 mean intensity
            # 8 exposure time (seconds)
            # 9 neutral density (fraction of 1 or percentage)
            # 10 excitation wavelength
            # 11 emission wavelength
            # 12 intensity scaling (usually 1)
            # 13 energy conversion factor (usually 1)

        cockpit.util.datadoc.write_data_as_mrc_with_exthdr(
            self.image_data,
            path,
            xy_size=xy_size,
            wavelengths=wavelengths,
            zxy0=xyz_pos,
            lens_id=lens_id,
            int_metadata_buffers=int_metadata_buffers,
            float_metadata_buffers=float_metadata_buffers,
        )
