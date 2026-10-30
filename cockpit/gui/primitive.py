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

import re
from ctypes import c_float

import numpy as np
from OpenGL.GL import *


CIRCLE_SEGMENTS = 32
PI = 3.141592654


class Primitive:
    """A class for rendering primitives from devices.

    Stages can use primitives to show reference positions, such
    as well or sample grid locations. This class puts much of the
    code for this in one place.

    Note that canvases in separate contexts will each need their
    own Primitives - Primitives can not be shared between GL contexts.
    """

    @staticmethod
    def factory(spec):
        """
        Returns an appropriate primitive given a specification.
        Primitives are specified by a lines in a config entry of the form:
        primitives:  c 1000 1000 100
                     r 1000 1000 100 100
        where:
        'c x0 y0 radius' defines a circle centred on x0, y0
        'r x0 y0 width height' defines a rectangle centred on x0, y0
        The primitive identifier may be in quotes, and values may be separated
        by any combination of spaces, commas and semicolons.
        """
        p = re.split(r"[ |,|;]+", re.sub(r"['|\"]", "", spec))
        p_type = p[0]
        p_data = tuple(map(float, p[1:]))
        # Spec is a type and some data
        if p_type in ["c", "C"]:
            return Circle(*p_data, n=CIRCLE_SEGMENTS)
        if p_type in ["r", "R"]:
            return Rectangle(*p_data)

    def __init__(self, *args, **kwargs):
        self._vbo = None
        self._vertices = []
        self._num_vertices = 0

    def make_vbo(self):
        vertices = self._vertices
        if self._vbo is None:
            self._vbo = glGenBuffers(1)
        glBindBuffer(GL_ARRAY_BUFFER, self._vbo)
        glBufferData(
            GL_ARRAY_BUFFER,
            len(vertices) * 4,
            (c_float * len(vertices))(*vertices),
            GL_STATIC_DRAW,
        )
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        self._num_vertices = len(vertices) // 2

    def render(self):
        if self._vbo is None:
            self.make_vbo()
        glEnableClientState(GL_VERTEX_ARRAY)
        glBindBuffer(GL_ARRAY_BUFFER, self._vbo)
        glVertexPointer(2, GL_FLOAT, 0, None)
        glDrawArrays(GL_LINE_LOOP, 0, self._num_vertices)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glDisableClientState(GL_VERTEX_ARRAY)


class Circle(Primitive):
    def __init__(self, x0, y0, r, n=CIRCLE_SEGMENTS):
        super().__init__()
        d_theta = 2.0 * PI / n
        cos_theta = np.cos(d_theta)
        sin_theta = np.sin(d_theta)
        x = r
        y = 0.0

        vs = []
        for i in range(n):
            vs.extend([(x0 + x), y0 + y])
            x_old = x
            x = cos_theta * x - sin_theta * y
            y = sin_theta * x_old + cos_theta * y
        self._vertices = vs


class Rectangle(Primitive):
    def __init__(self, x0, y0, w, h):
        super().__init__()
        dw = w / 2.0
        dh = h / 2.0

        vs = [
            (x0 - dw),
            y0 - dh,
            (x0 + dw),
            y0 - dh,
            (x0 + dw),
            y0 + dh,
            (x0 - dw),
            y0 + dh,
        ]

        self._vertices = vs
