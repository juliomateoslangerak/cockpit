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
import queue
import time
import traceback

import numpy as np
import wx.glcanvas
import wx.lib.newevent
from OpenGL.GL import *

import cockpit.util.datadoc
import cockpit.util.threads
from cockpit import events
from cockpit.gui.mosaic.tile import MegaTile, Tile


_logger = logging.getLogger(__name__)


ProgressStartEvent, EVT_PROGRESS_START = wx.lib.newevent.NewEvent()
ProgressUpdateEvent, EVT_PROGRESS_UPDATE = wx.lib.newevent.NewEvent()
ProgressEndEvent, EVT_PROGRESS_END = wx.lib.newevent.NewEvent()

## Zoom level at which we switch from rendering megatiles to rendering tiles.
ZOOM_SWITCHOVER = 1
BUFFER_LENGTH = 32


class MosaicCanvas(wx.glcanvas.GLCanvas):
    """Canvas where the mosaic is drawn.

    Mosaics consist of collections of images from the cameras.  This
    class sets up the OpenGL canvas that image data is drawn to.  Has
    a basic level-of-detail system so that the computer doesn't bog
    down horribly when trying to draw thousands of tiles at the same
    time.

    """

    ## Tiles and context are shared amongst all instances, since all
    # offer views of the same data.
    # The first instance creates the context.
    ## List of MegaTiles. These will be created in self.initGL.
    mega_tiles = []
    ## List of Tiles. These are created as we receive new images from
    # our parent.
    tiles = []
    ## Set of tiles that need to be rerendered in the next onPaint call.
    tiles_to_refresh = set()
    ## WX rendering context
    context = None

    def __init__(
        self,
        parent,
        stage_hard_limits,
        overlay_callback,
        mouse_callback,
        *args,
        **kwargs,
    ):
        """Set up the mosaic canvas.

        Args:
            stage_hard_limits (tuple): An ((xMin, xMax), (yMin, yMax)) tuple
                describing the limits of motion, in microns, of the stage.
            overlay_callback (callable): Function to call, during rendering, to
                draw the overlay on top of the mosaic.
            mouse_callback (callable): Function to propagate mouse events to.
        """
        super().__init__(parent, *args, **kwargs)

        self.stage_hard_limits = stage_hard_limits
        self.overlay_callback = overlay_callback

        ## X and Y translation when rendering.
        self.dx, self.dy = 0.0, 0.0
        ## Scaling factor.
        self.scale = 1.0
        ## Set to True once we've done some initialization.
        self.have_inited_gl = False
        ## WX rendering context
        if MosaicCanvas.context is None:
            # This is the first (and master) instance.
            MosaicCanvas.context = wx.glcanvas.GLContext(self)
            # Hook up onIdle - only one instance needs to process new tiles.
            self.Bind(wx.EVT_IDLE, self.on_idle)

        ## Error that occurred when rendering. If this happens, we prevent
        # further rendering to avoid error spew.
        self.render_error = None

        ## A buffer of images waiting to be added to the mosaic.
        self.pending_images = queue.Queue(BUFFER_LENGTH)

        self.Bind(wx.EVT_PAINT, self.on_paint)
        self.Bind(wx.EVT_MOUSE_EVENTS, mouse_callback)
        # Do nothing on this event, to avoid flickering.
        self.Bind(wx.EVT_ERASE_BACKGROUND, lambda event: event)
        # event on DPI change on high DPI screens, needed for Mac retina
        # displays.
        self.Bind(wx.EVT_DPI_CHANGED, self.on_dpi_change)
        self.Bind(EVT_PROGRESS_START, self.create_progress_dialog)
        self.Bind(EVT_PROGRESS_UPDATE, self.update_progress_dialog)
        self.Bind(EVT_PROGRESS_END, self.destroy_progress_dialog)
        self.old_scale_factor = self.GetContentScaleFactor()

    def init_gl(self):
        """Now that OpenGL's ready to go, perform any necessary initialization.

        We can now create textures, for example, so it's time to create our
        MegaTiles.
        """
        glClearColor(1, 1, 1, 0)

        # Non-zero objective offsets require expansion of area covered
        # by megatiles.
        offsets = [h.offset for h in wx.GetApp().objectives.get_handlers()]
        min_max = lambda l: (min(l), max(l))
        x_off_lim = min_max([-offset[0] for offset in offsets])
        y_off_lim = min_max([offset[1] for offset in offsets])
        (x_min, x_max), (y_min, y_max) = self.stage_hard_limits
        # Bounds checks ensure we only increase the megatile area, not
        # decrease it.
        #
        # We need to increase upper limit further so we don't miss tiles at
        # the edges. We should just need to add megaTileMicronSize to each
        # axis to make sure we add tiles up to and including our bounds. This
        # works for the x-axis, but not y. For some reason, we need to add
        # *four* times this value to the y-axis. The megaTile's y-origin is at
        # its centre, whereas the x-origin is at an edge; as far as I can see,
        # but this should require only up to three times the tilesize added to
        # the upper limit, not four.
        # Four works, though.
        # Arrays run [0,0] .. [ncols, nrows]; GL runs (-1,-1) .. (1,1). Since
        # making adjustments to render [0,0] at (-1,1), we now add two megatiles
        # at each y limit, rather than 4 at one edge.
        texture_size = glGetInteger(GL_MAX_TEXTURE_SIZE)
        # If we use the full texture size, it seems it's too large for manipul-
        # ation in a framebuffer:
        #   * on Macs with Intel chipsets;
        #   * on some mobile nVidia chipsets.
        # Check vendor with glGetString(GL_VENDOR)
        # GL_MAX_FRAMEBUFFER_WIDTH and _HEIGHT are not universally available,
        # so we just use a quarter of the max texture size or a reasonable
        # upper bound which has been found to work in tests on 2017-ish
        # Macbook Pro.
        texture_size = min(texture_size // 4, 16384)
        MegaTile.set_pixel_size(texture_size)
        x_min += min(0, x_off_lim[0]) - MegaTile.micron_size
        x_max += max(0, x_off_lim[1]) + MegaTile.micron_size
        y_min += min(0, y_off_lim[0]) - 2 * MegaTile.micron_size
        y_max += max(0, y_off_lim[1]) + 2 * MegaTile.micron_size
        for x in np.arange(x_min, x_max, MegaTile.micron_size):
            for y in np.arange(y_min, y_max, MegaTile.micron_size):
                self.mega_tiles.append(MegaTile((-x, y)))
        self.have_inited_gl = True

    def rerender_megatiles(self, tiles=None):
        """Because tiles have been changed, we must now rerender all of our
        megatiles.

        Don't do this often, and definitely not when other threads need
        attention.

        Args:
            tiles (list): Which tiles to rerender. Defaults to rerendering all.
        """
        self.SetCurrent(self.context)
        if tiles is None:
            tiles = self.mega_tiles
        for tile in tiles:
            tile.recreate_texture()
            tile.prerender_tiles(self.tiles)

    def delete_all(self):
        """Delete all tiles and textures, including the megatiles."""
        self.delete_tiles_list(list(self.tiles))
        events.publish(events.MOSAIC_UPDATE)

    def get_tiles_intersecting(self, start, end, allowed_tiles=None):
        """Get all tiles that intersect the specified box, pulling from the
        provided list, or from all tiles if no list is provided."""
        if allowed_tiles is None:
            allowed_tiles = self.tiles
        x1 = min(start[0], end[0])
        x2 = max(start[0], end[0])
        y1 = min(start[1], end[1])
        y2 = max(start[1], end[1])
        start = (x1, y1)
        end = (x2, y2)
        tiles = []
        for tile in allowed_tiles:
            if tile.intersects_box((start, end)):
                tiles.append(tile)
        return tiles

    def get_composite_tile_data(self, tile, allowed_tiles=None):
        """Generate a composite array of tile data surrounding the provided
        tile, pulling only from the provided list of allowed tiles (or all
        tiles, if no list is provided)."""
        if allowed_tiles is None:
            allowed_tiles = self.tiles
        tile_shape = tile.texture_data.shape
        # Start with a neutral background based on the tile's mean value.
        result = (
            np.ones(
                (tile_shape[0] * 3, tile_shape[1] * 3),
                dtype=tile.texture_data.dtype,
            )
            * tile.texture_data.mean()
        )

        # Get the bounding box 3x bigger than the tile with the tile at the
        # center.
        width, height = tile.size
        tile_x, tile_y, tile_z = tile.position
        start = (tile_x - width, tile_y - height)
        end = (tile_x + width * 2, tile_y + height * 2)
        pixel_size = tile.get_pixel_size()
        for alt_tile in self.get_tiles_intersecting(start, end, allowed_tiles):
            if alt_tile.get_pixel_size() != pixel_size:
                # Don't try to deal with tiles with differing pixel sizes.
                continue
            # Figure out the portion of alt_tile that intersects our region.
            x_min = max(start[0], alt_tile.position[0])
            x_max = min(end[0], alt_tile.position[0] + alt_tile.size[0])
            y_min = max(start[1], alt_tile.position[1])
            y_max = min(end[1], alt_tile.position[1] + alt_tile.size[1])
            x_pixels = (x_max - x_min) // pixel_size[0]
            y_pixels = (y_max - y_min) // pixel_size[1]
            # Get the offset into alt_tile, and thus the relevant pixel data.
            alt_x = (x_min - alt_tile.position[0]) / pixel_size[0]
            alt_y = (y_min - alt_tile.position[1]) / pixel_size[1]
            sub_region = alt_tile.texture_data[
                alt_x : alt_x + x_pixels, alt_y : alt_y + y_pixels
            ]
            if 0 in sub_region.shape:
                # The intersection is tangential; unlikely but can happen.
                # Skip this tile as it doesn't provide useful intersection.
                continue
            # Get the offset into result.
            r_x = max(0, (alt_tile.position[0] - start[0]) / pixel_size[0])
            r_y = max(0, (alt_tile.position[1] - start[1]) / pixel_size[1])
            # HACK: for some reason that I don't understand, the above gives
            # me swapped-and-offset X and Y axes. I need to swap them back
            # while accounting for the difference in aspect ratio.
            t_x = r_y * (float(tile_shape[0]) / tile_shape[1])
            t_y = r_x * (float(tile_shape[1]) / tile_shape[0])
            r_x, r_y = t_x, t_y
            target = result[r_x : r_x + x_pixels, r_y : r_y + y_pixels]
            target[:] = sub_region

        return result

    def delete_tiles_intersecting(self, start, end):
        """Delete all tiles that intersect the specified box."""
        self.delete_tiles_list(self.get_tiles_intersecting(start, end))
        events.publish(events.MOSAIC_UPDATE)

    @cockpit.util.threads.call_in_main_thread
    def delete_tiles_list(self, tiles_to_delete):
        """Delete a list of tiles."""
        for tile in tiles_to_delete:
            tile.wipe()
            del self.tiles[self.tiles.index(tile)]
        self.SetCurrent(self.context)

        # Rerender all megatiles that are now invalid.
        dirtied = []
        for mega_tile in self.mega_tiles:
            for tile in tiles_to_delete:
                if mega_tile.intersects_box(tile.box):
                    dirtied.append(mega_tile)
                    break
        self.rerender_megatiles(dirtied)
        self.Refresh()
        events.publish(events.MOSAIC_UPDATE)

    def on_idle(self, event):
        if self.pending_images.empty():  # or not self.IsShownOnScreen():
            return
        # Draw as many images as possible in 50ms.
        t = time.time()
        new_tiles = []
        self.SetCurrent(self.context)
        while not self.pending_images.empty() and (time.time() - t < 0.05):
            (
                data,
                position,
                size,
                scalings,
                layer,
                metadata,
            ) = self.pending_images.get()
            new_tiles.append(Tile(data, position, size, scalings, layer, metadata))
        self.tiles.extend(new_tiles)
        for mega_tile in self.mega_tiles:
            mega_tile.prerender_tiles(new_tiles)

        self.tiles_to_refresh.update(new_tiles)

        self.Refresh()
        events.publish(events.MOSAIC_UPDATE)
        if not self.pending_images.empty():
            event.RequestMore()

    def add_image(
        self, data, position, size, scalings=(None, None), layer=0, metadata=None
    ):
        """Add a new image to the mosaic.
        @cockpit.util.threads.call_in_main_thread."""
        self.pending_images.put((data, position, size, scalings, layer, metadata))

    @cockpit.util.threads.call_in_main_thread
    def rescale(self, min_max=None):
        """Rescale the tiles.

        Args:
            min_max (tuple): A (blackpoint, whitepoint) tuple, or None to
                rescale each tile individually.
        """
        if min_max is None:
            # Tiles will treat this as "use our own data".
            min_max = (None, None)
        for tile in self.tiles:
            tile.scale_histogram(*min_max)
        self.tiles_to_refresh.update(self.tiles)
        self.rerender_megatiles()
        self.Refresh()

    def on_paint(self, event):
        """Paint the canvas -- in other words, paint all tiles, plus whatever
        overlays we need."""
        if self.render_error is not None:
            return

        try:
            dc = wx.PaintDC(self)
            self.SetCurrent(self.context)

            if not self.have_inited_gl:
                self.init_gl()

            width, height = self.GetClientSize() * self.GetContentScaleFactor()

            glViewport(0, 0, width, height)
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            glOrtho(-0.375, width - 0.375, -0.375, height - 0.375, 1, -1)
            glMatrixMode(GL_MODELVIEW)

            for tile in self.tiles_to_refresh:
                tile.refresh()
            self.tiles_to_refresh = set()

            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
            glTranslated(self.dx, self.dy, 0)
            glScaled(self.scale, self.scale, 1)

            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            ## Paint the megatiles if we're zoomed out, or the
            # normal tiles if we're zoomed in.
            glEnable(GL_TEXTURE_2D)
            view_box = self.get_view_box()
            if self.scale < ZOOM_SWITCHOVER:
                for mega_tile in self.mega_tiles:
                    mega_tile.render(view_box)
            else:
                for tile in self.tiles:
                    tile.render(view_box)
            glDisable(GL_TEXTURE_2D)

            if self.overlay_callback is not None:
                self.overlay_callback()

            glFlush()
            self.SwapBuffers()
        except Exception as e:
            print("Error rendering the canvas:", e)
            traceback.print_exc()
            self.render_error = e

    def zoom_to(self, x, y, scale):
        """Change our view transform."""
        # Paranoia
        if not scale:
            return
        width, height = self.GetClientSize() * self.GetContentScaleFactor()
        self.dx = -x * scale + width / 2
        self.dy = -y * scale + height / 2
        self.scale = scale
        self.Refresh()

    def zoom_at_point(self, multiplier: float, wx_point: wx.RealPoint) -> None:
        """Zoom in/out while keeping point in place.

        When using the mouse scroll to zoom in/out we want to keep the
        point under the mouse fixed in place.  See ``multiplyZoom`` to
        zoom while keeping the viewport centre point fixed.
        """
        wx_size = self.GetClientSize()
        scale_factor = self.GetContentScaleFactor()

        gl_size = wx.RealPoint(wx_size.x, wx_size.y) * scale_factor
        gl_xpoint = wx_point.x * scale_factor
        gl_ypoint = (wx_point.y * scale_factor - gl_size.y) * -1

        self.dx = gl_xpoint - (gl_xpoint - self.dx) * multiplier
        self.dy = gl_ypoint - (gl_ypoint - self.dy) * multiplier
        self.scale *= multiplier
        self.Refresh()

    def multiply_zoom(self, multiplier: float) -> None:
        """Change zoom by the specified multiplier.

        This will change the translational offset too to keep the view
        centred.  See ``ZoomAtPoint`` to zoom while keeping another
        point fixed (typically the mouse location).

        """
        width, height = self.GetClientSize()
        viewport_centre_point = wx.RealPoint(width, height) / 2
        self.zoom_at_point(multiplier, viewport_centre_point)

    def on_dpi_change(self, event):
        # get new scale and work out change.
        content_scale = self.GetContentScaleFactor()
        multiplier = content_scale / self.old_scale_factor
        # work out old canvas size to shift back to origin
        old_width, old_height = self.GetClientSize() * self.old_scale_factor
        old_half_width = old_width / 2
        old_half_height = old_height / 2
        # apply shift to give new position of center of image
        x = (-self.dx + old_half_width) / (self.scale)
        y = (-self.dy + old_half_height) / (self.scale)
        self.scale *= multiplier
        # move to this new view, which is same as old but at different DPI
        self.zoom_to(x, y, self.scale)
        # store scale factor so we can use for next rescale
        self.old_scale_factor = content_scale

    def drag_view(self, offset):
        """Change our translation by the specified number of pixels."""
        self.dx += offset[0]
        self.dy -= offset[1]
        self.Refresh()

    def map_screen_to_canvas(self, position):
        """Remap an (X, Y) tuple of screen coordinates to a location on the
        stage."""
        scale_factor = self.GetContentScaleFactor()
        position = (position[0] * scale_factor, position[1] * scale_factor)

        height = self.GetClientSize()[1] * scale_factor
        return (
            (self.dx - position[0]) / self.scale,
            -(self.dy - height + position[1]) / self.scale,
        )

    def get_view_box(self):
        """Return a (bottom left, top right) tuple showing what part of the
        stage is currently visible."""
        width, height = self.GetClientSize() * self.GetContentScaleFactor()
        bottom_left = (-self.dx / self.scale, -self.dy / self.scale)
        top_right = (
            -(self.dx - width) / self.scale,
            -(self.dy - height) / self.scale,
        )
        return (bottom_left, top_right)

    @cockpit.util.threads.call_in_new_thread
    def save_tiles(self, save_path):
        """Given a path to a file, save the mosaic to that file and an adjacent
        file.

        The first is a text file that describes the layout of the tiles; the
        second is an MRC file that holds the actual image data.
        """
        wx.PostEvent(
            self.GetEventHandler(),
            ProgressStartEvent(
                title="Saving...",
                message="Saving mosaic image data...",
                maximum=len(self.tiles),
            ),
        )
        handle = open(save_path, "w")
        mrcPath = save_path + ".mrc"
        if ".txt" in save_path:
            mrcPath = save_path.replace(".txt", ".dv")
        handle.write("%s\n" % mrcPath)
        width = 0
        height = 0
        for tile in self.tiles:
            width = max(width, tile.texture_data.shape[0])
            height = max(height, tile.texture_data.shape[1])
            # We do this by a series of extensions since some of these lists
            # may be Numpy arrays, which don't do array extension when you
            # "add" them.
            values = []
            values.extend(tile.position)
            values.extend(tile.size)
            values.extend(tile.texture_data.shape)
            values.extend(tile.histogram_scale)
            values.append(tile.layer)
            values = map(str, values)
            handle.write(",".join(values) + "\n")
        handle.close()

        # Now we have the max image extent in X and Y, we can pile everything
        # into a single array for saving as an MRC file. Images smaller than
        # the max will be padded with zeros.
        image_data = np.zeros(
            (1, 1, len(self.tiles), width, height), dtype=np.uint16
        )
        for i, tile in enumerate(self.tiles):
            image_data[
                0,
                0,
                i,
                : tile.texture_data.shape[0],
                : tile.texture_data.shape[1],
            ] = tile.texture_data
        header = cockpit.util.datadoc.make_header_for(image_data)
        # meta data for mosaic image header
        #
        nb_integers = 8
        nb_floats = 32
        int_metadata_buffer = np.array([0] * nb_integers, dtype=np.int32)
        float_metadata_buffer = np.array(
            [0.0] * nb_floats, dtype=np.float32
        )
        float_metadata_buffer[12] = 1.0  # intensity scaling

        header.NumIntegers = nb_integers
        header.NumFloats = nb_floats
        # we can only have one lens ID so just grab the current one.
        header.LensNum = wx.GetApp().objectives.get_current().lens_id
        extended_bytes = 4 * (nb_integers + nb_floats) * len(self.tiles)
        header.next = extended_bytes

        metadata_offset = 1024
        data_offset = 1024 + extended_bytes

        handle = open(mrcPath, "wb")
        cockpit.util.datadoc.write_mrc_header(header, handle)
        handle.seek(metadata_offset)
        for i in range(len(self.tiles)):
            metadata = self.tiles[i].metadata
            float_metadata_buffer[1] = metadata["timestamp"]
            float_metadata_buffer[2:5] = metadata["image_position"]
            float_metadata_buffer[5] = self.tiles[i].texture_data.min()
            float_metadata_buffer[6] = self.tiles[i].texture_data.min()
            float_metadata_buffer[8] = metadata["exposure_time"]
            float_metadata_buffer[10] = metadata["ex_wavelength"]
            float_metadata_buffer[11] = metadata["wavelength"]
            handle.write(int_metadata_buffer)
            handle.write(float_metadata_buffer)

        # we should be here already but just to be sure
        handle.seek(data_offset)
        for i, image in enumerate(image_data[:, :]):
            handle.write(image)
            wx.PostEvent(self.GetEventHandler(), ProgressUpdateEvent(value=i))
        handle.close()
        wx.PostEvent(self.GetEventHandler(), ProgressEndEvent())

    @cockpit.util.threads.call_in_new_thread
    def load_tiles(self, file_path):
        """Load a text file describing a set of tiles, as well as the tile
        image data.

        This is made a bit trickier by the fact that we want to display a
        progress dialog that updates as new images are added, but addImage()
        must run in the main thread while we run in a different one ...
        although maybe not since MAP added a queue and a thread to process new
        tiles.
        """
        with open(file_path, "r") as handle:
            mrc_path = handle.readline().strip()
            tile_stats = []
            for line in handle:
                # X position, Y position, Z position,
                # X micron size, Y micron size,
                # X pixel size, Y pixel size, blackpoint, whitepoint, layer.
                # We'll have to convert the pixel sizes and layer to
                # ints later.
                tile_stats.append(list(map(float, line.strip().split(","))))
        try:
            doc = cockpit.util.datadoc.DataDoc(mrc_path)
        except Exception as e:
            wx.MessageBox(
                (
                    "I was unable to load the MRC file at\n"
                    "%s\n"
                    "holding the tile data. The error message was:\n"
                    "\n"
                    "%s\n"
                    "\n"
                    "Please verify that the file path is correct and the file is valid."
                    % (mrc_path, e)
                ),
                style=(wx.ICON_INFORMATION | wx.OK),
                parent=self.GetParent(),
            )
            return
        # NOTE: this dialog is not safe to Update, since the update calls must
        # be referred to the main thread (via wx.CallAfter) and may arrive
        # in an unpredictable order. Due to the unpredictable order, the call
        # to Destroy (which must also happen in the main thread via CallAfter)
        # may arrive before all of the Update calls are processed, resulting
        # in a segfault.
        wx.PostEvent(
            self.GetEventHandler(),
            ProgressStartEvent(
                title="Loading",
                message="Loading mosaic image data...",
                maximum=100,
            ),
        )
        if doc.image_array.shape[2] > len(tile_stats):
            # More images in the file than we have stats for.
            _logger.warning(
                "Loading mosaic with %d images; only have positioning information for %d.",
                doc.image_array.shape[2],
                len(tile_stats),
            )
        max_images = min(doc.image_array.shape[2], len(tile_stats))
        for i in range(max_images):
            image = doc.image_array[0, 0, i]
            stats = tile_stats[i]
            try:
                data = image[: int(stats[5]), : int(stats[6])]
                self.add_image(
                    data, stats[:3], stats[3:5], stats[7:9], int(stats[9])
                )
            except Exception as e:
                wx.MessageBox(
                    (
                        "Failed to load line %d of file %s: %s.\n"
                        "\n"
                        "Please see the logs for more details."
                        % (i, file_path, e)
                    ),
                    style=(wx.ICON_INFORMATION | wx.OK),
                    parent=self.GetParent(),
                )
                _logger.error(traceback.format_exc())
                wx.PostEvent(self.GetEventHandler(), ProgressEndEvent())
                return
        # Wait until we've loaded all tiles or we go a full second without
        # any new tiles arriving.
        nb_expected_tiles = len(self.tiles) + len(tile_stats)
        last_updated_time = time.time()
        cur_count = len(self.tiles)
        while len(self.tiles) != nb_expected_tiles:
            count = len(self.tiles)
            if count != cur_count:
                last_updated_time = time.time()
                cur_count = count
            if time.time() - last_updated_time > 1:
                break
            time.sleep(0.1)
        wx.PostEvent(self.GetEventHandler(), ProgressEndEvent())
        events.publish(events.MOSAIC_UPDATE)

    def create_progress_dialog(self, event):
        if hasattr(self, "progress_dialog"):
            return
        self.progress_dialog = wx.ProgressDialog(
            parent=self.GetParent(),
            title=event.title,
            message=event.message,
            maximum=event.maximum,
        )
        self.progress_dialog.Show()

    def update_progress_dialog(self, event):
        if hasattr(self, "progress_dialog"):
            self.progress_dialog.Update(event.value)
            self.progress_dialog.Show()

    def destroy_progress_dialog(self, event):
        if hasattr(self, "progress_dialog"):
            self.progress_dialog.Destroy()
            del self.progress_dialog
