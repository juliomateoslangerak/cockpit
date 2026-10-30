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

"""
Handles display of a single tile in the canvas.  A tile is either a
single image from one camera, or a larger array of low-resolution
images from that camera; the latter is used when zoomed out, as a
performance measure.

"""

import numpy as np
from OpenGL.GL import (
    GL_BLUE_BIAS,
    GL_BLUE_SCALE,
    GL_CLAMP,
    GL_COLOR_ATTACHMENT0,
    GL_DRAW_FRAMEBUFFER,
    GL_FLOAT,
    GL_GREEN_BIAS,
    GL_GREEN_SCALE,
    GL_LINEAR,
    GL_LUMINANCE,
    GL_MAP_COLOR,
    GL_MODELVIEW,
    GL_NEAREST,
    GL_PROJECTION,
    GL_QUADS,
    GL_RED_BIAS,
    GL_RED_SCALE,
    GL_RGB,
    GL_SHORT,
    GL_TEXTURE_2D,
    GL_TEXTURE_MAG_FILTER,
    GL_TEXTURE_MIN_FILTER,
    GL_TEXTURE_WRAP_S,
    GL_TEXTURE_WRAP_T,
    GL_UNPACK_ALIGNMENT,
    GL_UNPACK_SWAP_BYTES,
    GL_UNSIGNED_BYTE,
    GL_UNSIGNED_SHORT,
    glBegin,
    glBindFramebuffer,
    glBindTexture,
    glColor3f,
    glDeleteTextures,
    glEnable,
    glEnd,
    glFramebufferTexture2D,
    glGenFramebuffers,
    glGenTextures,
    glLoadIdentity,
    glMatrixMode,
    glOrtho,
    glPixelStorei,
    glPixelTransferf,
    glPixelTransferi,
    glPopMatrix,
    glPushMatrix,
    glTexCoord2f,
    glTexImage2D,
    glTexParameteri,
    glTexSubImage2D,
    glTranslatef,
    glVertex2f,
    glViewport,
)


## This module contains the Tile and MegaTile classes, along with some
# supporting functions and constants.


def get_tex_size(width, height):
    """Finds the smallest powers of two that will contain a texture with the
    specified dimensions."""
    result = [2, 2]
    for i, val in enumerate([width, height]):
        while result[i] < val:
            result[i] *= 2
    return tuple(result)


## Maps numpy datatypes to OpenGL datatypes
dtype_to_gl_type_map = {
    np.uint8: GL_UNSIGNED_BYTE,
    np.uint16: GL_UNSIGNED_SHORT,
    np.int16: GL_SHORT,
    np.float32: GL_FLOAT,
    np.float64: GL_FLOAT,
    np.int32: GL_FLOAT,
    np.uint32: GL_FLOAT,
    np.complex64: GL_FLOAT,
    np.complex128: GL_FLOAT,
}


class Tile:
    """This class handles a single tile in the mosaic."""

    def __init__(
        self,
        texture_data,
        position,
        size,
        histogram_scale,
        layer,
        metadata,
        should_delay_allocation=False,
    ):
        """Array of pixel brightnesses."""
        self.texture_data = texture_data
        ## XYZ position tuple, in microns. NB the Z portion is ignored
        # for rendering purposes and is mostly just kept around so we know
        # the Z altitude at which the tile was collected, for later use.
        self.position = position
        ## width/height tuple, in microns
        self.size = size
        ## Box describing space we occupy: (upper left corner, lower right corner)
        self.box = (
            self.position[:2],
            (self.position[0] + self.size[0], self.position[1] + self.size[1]),
        )

        ## Grouping this tile belongs to, used to toggle display
        self.layer = layer
        self.metadata = metadata
        # collect metadata

        ## OpenGL texture ID
        self.texture = glGenTextures(1)
        self.scale_histogram(histogram_scale[0], histogram_scale[1])
        # Indicate refresh required after scaling histogram.
        self.should_refresh = False
        if not should_delay_allocation:
            self.bind_texture()
            self.refresh()

    def bind_texture(self):
        glBindTexture(GL_TEXTURE_2D, self.texture)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
        # These two are only really needed for megatiles; normal
        # tiles don't have to deal with texture wrapping.
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP)

        img = self.texture_data

        pic_ny, pic_nx = img.shape
        tex_nx, tex_ny = get_tex_size(pic_nx, pic_ny)

        img_type = img.dtype.type
        if img_type not in dtype_to_gl_type_map:
            raise ValueError("Unsupported data mode %s" % str(img_type))
        glTexImage2D(
            GL_TEXTURE_2D,
            0,
            GL_RGB,
            tex_nx,
            tex_ny,
            0,
            GL_LUMINANCE,
            dtype_to_gl_type_map[img_type],
            None,
        )

    def refresh(self):
        img = self.texture_data
        hist_min, hist_max = self.histogram_scale
        pic_ny, pic_nx = img.shape
        if img.dtype.type in (np.float64, np.int32, np.uint32):
            data = img.astype(np.float32)
            img_string = data.tobytes()
            img_type = np.float32
        else:
            img_string = img.tobytes()
            img_type = img.dtype.type

        # max_u_short: value that represents "maximum color" - i.e. white
        if img.dtype.type == np.uint16:
            max_u_short = (1 << 16) - 1
        elif img.dtype.type == np.int16:
            max_u_short = (1 << 15) - 1
        elif img.dtype.type == np.uint8:
            max_u_short = (1 << 8) - 1
        else:
            max_u_short = 1

        max_range = float(hist_max) - float(hist_min)
        f_bias = -float(hist_min) / max_range
        f = max_u_short / max_range

        glBindTexture(GL_TEXTURE_2D, self.texture)
        glPixelTransferf(GL_RED_SCALE, f)
        glPixelTransferf(GL_GREEN_SCALE, f)
        glPixelTransferf(GL_BLUE_SCALE, f)

        glPixelTransferf(GL_RED_BIAS, f_bias)
        glPixelTransferf(GL_GREEN_BIAS, f_bias)
        glPixelTransferf(GL_BLUE_BIAS, f_bias)

        glPixelTransferi(GL_MAP_COLOR, False)

        if img.dtype.type in (
            np.float64,
            np.int32,
            np.uint32,
            np.complex64,
            np.complex128,
        ):
            item_size = 4
            glPixelStorei(
                GL_UNPACK_SWAP_BYTES, False
            )  # create native float32 copy - see below
        else:
            item_size = img.itemsize
            glPixelStorei(GL_UNPACK_SWAP_BYTES, not img.dtype.isnative)

        glPixelStorei(GL_UNPACK_ALIGNMENT, item_size)

        if img_type not in dtype_to_gl_type_map:
            raise ValueError("Unsupported data mode %s" % str(img_type))
        glTexSubImage2D(
            GL_TEXTURE_2D,
            0,
            0,
            0,
            pic_nx,
            pic_ny,
            GL_LUMINANCE,
            dtype_to_gl_type_map[img_type],
            img_string,
        )

    def wipe(self):
        """Free up memory we were using."""
        glDeleteTextures([self.texture])

    def recreate_texture(self):
        """Wipe our texture and recreate it, presumably because it has changed
        somehow."""
        self.wipe()
        self.texture = glGenTextures(1)
        self.bind_texture()

    def intersects_box(self, view_box):
        """Return true iff our area intersects the given (bottom_left, top_right)
        tuple."""
        bottom_left, top_right = view_box
        tile_bottom_left, tile_top_right = self.box

        if (
            tile_bottom_left[0] > top_right[0]
            or tile_top_right[0] < bottom_left[0]
            or tile_top_right[1] < bottom_left[1]
            or tile_bottom_left[1] > top_right[1]
        ):
            return False
        return True

    def render(self, view_box):
        """Draw the tile, if it intersects the given view box."""
        if not self.intersects_box(view_box):
            return
        if self.should_refresh:
            self.refresh()
            self.should_refresh = False

        glColor3f(1, 1, 1)

        img = self.texture_data
        pic_ny, pic_nx = img.shape
        tex_nx, tex_ny = get_tex_size(pic_nx, pic_ny)
        pic_tex_ratio_x = float(pic_nx) / tex_nx
        pic_tex_ratio_y = float(pic_ny) / tex_ny

        x, y = self.position[:2]

        glBindTexture(GL_TEXTURE_2D, self.texture)
        glBegin(GL_QUADS)
        glTexCoord2f(0, pic_tex_ratio_y)
        glVertex2f(x, y)
        glTexCoord2f(pic_tex_ratio_x, pic_tex_ratio_y)
        glVertex2f(x + self.size[0], y)
        glTexCoord2f(pic_tex_ratio_x, 0)
        glVertex2f(x + self.size[0], y + self.size[1])
        glTexCoord2f(0, 0)
        glVertex2f(x, y + self.size[1])
        glEnd()

    def scale_histogram(self, min_val=None, max_val=None):
        """Set our histogram_scale tuple to (min, max), or base those off of
        self.texture_data if the provided values are None."""
        if min_val is None:
            min_val = self.texture_data.min()
        if max_val is None:
            max_val = self.texture_data.max()
        if min_val == max_val:
            # Prevent dividing by zero when we have to scale by these
            # values for display.
            max_val = min_val + 1
        ## Used to scale the brightness of the overall tile, like the
        # histogram controls used for the camera views.
        self.histogram_scale = (min_val, max_val)
        self.should_refresh = True

    def get_pixel_size(self):
        """Return the (xSize, ySize) tuple of a single pixel of texture data
        in GL units."""
        return (
            self.size[0] / self.texture_data.shape[0],
            self.size[1] / self.texture_data.shape[1],
        )


## Framebuffer to use when prerendering. Set to None initially since
# we have to wait for OpenGL to get set up in our window before we can
# use it.
mega_tile_framebuffer = None


class MegaTile(Tile):
    """This class handles pre-rendering of normal-sized Tile instances at a
    reduced level of detail, which allows us to keep the program responsive
    even when thousands of tiles are in view."""

    ## Length in pixels of one edge of a MegaTile's texture.
    pixel_size = None
    ## Length in microns of one edge of a MegaTile's texture.
    micron_size = None
    ## An array of ones, used to initialize the MegaTile textures.
    _empty_tile_data = None

    def __init__(self, position):
        """Instantiate the megatile.

        The main difference here is that megatiles don't allocated any video
        memory until they have something to display; since the majority of the
        mosaic is usually blank, this saves significantly om memory.

        At this time, if mega_tile_framebuffer has not been created
        yet, create it.
        """
        super().__init__(
            self._empty_tile_data,
            position,
            (self.micron_size, self.micron_size),
            (0, 1),
            "megatiles",
            metadata=None,
            should_delay_allocation=True,
        )
        ## Counts the number of tiles we've rendered to ourselves.
        self.nb_rendered_tiles = 0
        ## Whether or not we've allocated memory for our texture yet.
        self.have_allocated_memory = False

        global mega_tile_framebuffer
        if mega_tile_framebuffer is None:
            mega_tile_framebuffer = glGenFramebuffers(1)

    @classmethod
    def set_pixel_size(cls, edge):
        if cls.pixel_size is not None:
            # Class is already initialized - nothing to do.
            # Used to raise an exception here, but both MacroStage and
            # TouchScreen have MacroStageZ instances that each call this
            # method, and raising an exception will prevent correct
            # initialisation of any but the first instance.
            return
        cls.pixel_size = edge
        cls.micron_size = edge * 1
        cls._empty_tile_data = np.ones((edge, edge), dtype=np.float32)

    def prerender_tiles(self, tiles):
        """Go through the provided list of Tiles, find the ones that overlap
        our area, and prerender them to our texture."""
        if not tiles:
            return
        min_x = self.position[0]
        min_y = self.position[1]
        max_x = self.position[0] + self.micron_size
        max_y = self.position[1] + self.micron_size
        view_box = ((min_x, min_y), (max_x, max_y))
        new_tiles = []
        for tile in tiles:
            if tile.intersects_box(view_box):
                new_tiles.append(tile)
        if new_tiles:
            # Allocate memory for our texture, if needed.
            if not self.have_allocated_memory:
                self.bind_texture()
                self.refresh()
                self.have_allocated_memory = True
            self.nb_rendered_tiles += len(new_tiles)
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, mega_tile_framebuffer)
            glFramebufferTexture2D(
                GL_DRAW_FRAMEBUFFER,
                GL_COLOR_ATTACHMENT0,
                GL_TEXTURE_2D,
                self.texture,
                0,
            )

            glPushMatrix()
            glLoadIdentity()
            glViewport(0, 0, self.pixel_size, self.pixel_size)
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            glOrtho(0, self.micron_size, self.micron_size, 0, 1, 0)
            glTranslatef(-self.position[0], -self.position[1], 0)
            glMatrixMode(GL_MODELVIEW)

            glEnable(GL_TEXTURE_2D)
            for tile in new_tiles:
                tile.render(view_box)

            glPopMatrix()
            glBindFramebuffer(GL_DRAW_FRAMEBUFFER, 0)

    def wipe(self):
        """Prevent trying to delete our texture if we haven't made it yet."""
        if self.have_allocated_memory:
            super().wipe()
            self.have_allocated_memory = False
            self.nb_rendered_tiles = 0

    def recreate_texture(self):
        """Prevent allocating a new texture if we haven't drawn anything yet."""
        if self.have_allocated_memory:
            super().recreate_texture()
            self.refresh()

    def render(self, view_box):
        if not self.nb_rendered_tiles:
            # We're empty, so no need to render.
            return
        super().render(view_box)
