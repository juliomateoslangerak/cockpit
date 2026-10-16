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

## Copyright (c) 2005 The Regents of the University of California
##
## All rights reserved.
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions are met:
##
##   a. Redistributions of source code must retain the above copyright notice,
##      this list of conditions and the following disclaimer.
##   b. Redistributions in binary form must reproduce the above copyright
##      notice, this list of conditions and the following disclaimer in the
##      documentation and/or other materials provided with the distribution.
##   c. Neither the name of the University of California, San Francisco nor
##      the names of its contributors may be used to endorse or promote products
##      derived from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
## AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
## IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
## ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
## BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY,
## OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
## SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
## INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
## CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
## ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
## THE POSSIBILITY OF SUCH DAMAGE.

"""MRC file format: refer to
https://web.archive.org/web/20190129061250/http://msg.ucsf.edu/IVE/IVE4_HTML/IM_ref2.html

Mrc class uses memory mapping (file size limit about 1GB (more or less)
Mrc2 class section wise file/array I/O
"""

__author__ = "Sebastian Haase <haase@msg.ucsf.edu>"


import numpy as np


def bind_file(fn, writable=0):
    """open existing Mrc file

    returns memmaped array
    array has special 'Mrc' attribute
    """

    mode = "r"
    if writable:
        mode = "r+"
    a = Mrc(fn, mode)

    return a.data_with_mrc(fn)


class NdarrayInMrcFile(np.ndarray):
    def __array_finalize__(self, obj):
        self.Mrc = getattr(obj, "Mrc", None)


class Mrc:
    def __init__(
        self, path, mode="r", ext_hdr_size=0, ext_hdr_nints=0, ext_hdr_nfloats=0
    ):
        """mode can be 'r' or 'r+'"""
        import os

        self.path = os.path.abspath(path)
        self.filename = os.path.basename(path)

        if ext_hdr_size and ext_hdr_size % 1024:
            raise ValueError(
                "extended header size needs to be integer multiple of 1024"
            )

        self.m = np.memmap(path, mode=mode)
        self.h = self.m[:1024]

        self.hdr = make_hdr_array(self.h)

        nz_before_byte_order = self.hdr.Num[0]
        if nz_before_byte_order < 0 or nz_before_byte_order > 10000:
            self.hdr._array.dtype = self.hdr._array.dtype.newbyteorder()
            self.is_byte_swapped = True
        else:
            self.is_byte_swapped = False

        self.data_offset = 1024 + self.hdr.next
        self.d = self.m[self.data_offset :]

        self.e = self.m[1024 : self.data_offset]

        self.do_data_map()

        self.num_ints = self.hdr.NumIntegers
        self.num_floats = self.hdr.NumFloats

        if self.num_ints > 0 or self.num_floats > 0:
            self.do_ext_hdr_map()

        else:
            self.ext_hdr_array = None

    def insert_ext_hdr(self, num_ints, num_floats, nz=-1):
        if num_ints == num_floats == 0:
            raise ValueError("insertExtHdr with no integers or floats")
        assert self.data_offset == 1024
        assert self.hdr.next == 0
        if nz <= 0:
            nz = self.hdr.Num[-1]

        bytes = 4 * (num_ints + num_floats) * nz
        next = 1024 * ((bytes % 1024 != 0) + bytes // 1024)

        self.hdr.next = next
        self.hdr.NumIntegers = num_ints
        self.hdr.NumFloats = num_floats

        self.num_ints = num_ints
        self.num_floats = num_floats

        self.data_offset = 1024 + next
        self.e = self.m.insert(1024, next)

        self.do_ext_hdr_map()

    def do_ext_hdr_map(self, nz=0):
        """maps the extended header space to a recarray
        if nz==0: then it maps 'NumSecs' tuples
        if nz==-1 then it maps the _maximal_ available space
             then self.ext_hdr_array will likely have more entries then the 'NumSecs'
        """
        if nz == 0:
            nz = self.hdr.Num[-1]

        maxnz = len(self.e) // (self.num_ints + self.num_floats) * 4
        if nz < 0 or nz > maxnz:
            nz = maxnz

        byteorder = "="
        type_descr = [
            ("int", "%s%di4" % (byteorder, self.num_ints)),
            ("float", "%s%df4" % (byteorder, self.num_floats)),
        ]

        self.ext_hdr_array = np.recarray(shape=nz, dtype=type_descr, buf=self.e)
        if self.is_byte_swapped:
            self.ext_hdr_array = self.ext_hdr_array.newbyteorder()

        self.ext_ints = self.ext_hdr_array.field("int")
        self.ext_floats = self.ext_hdr_array.field("float")

    def do_data_map(self):
        dtype = mrc_mode_to_dtype(self.hdr.PixelType)
        header_shape = shape_from_hdr(self.hdr)

        self.data = self.d.view()
        self.data.dtype = dtype

        ## Maybe adjust data shape? (file may be truncated)
        adjusted_shape = adjusted_data_shape(self.data.size, header_shape)
        if header_shape != adjusted_shape:
            print(
                (
                    "** WARNING **: file truncated - shape from header: %s."
                    " Expected to get %i pixels but got %i pixels"
                )
                % (header_shape, np.prod(header_shape), self.data.size)
            )

            ## In some cases, this may require the introduction of
            ## blank/padding data (see cockpit bug #289).  In such
            ## cases, we need to expand the data first which will lead
            ## to a N.array being returned instead of N.memmap.
            if self.data.size != np.prod(adjusted_shape):
                blanks = np.full(
                    np.prod(adjusted_shape) - self.data.size,
                    np.nan,
                    dtype=self.data.dtype,
                )
                self.data = np.concatenate((self.data, blanks))

        self.data.shape = adjusted_shape

        if self.is_byte_swapped:
            self.data = self.data.newbyteorder()

    def set_title(self, s, i=-1):
        """set title i (i==-1 means "append") to s"""
        set_title(self.hdr, s, i)

    def axis_order_str(self, only_letters=True):
        """return string indicating meaning of shape dimensions
        ##
        ## ZTW   <- non-interleaved
        ## WZT   <- OM1 ( easy on stage)
        ## ZWT   <- added by API (used at all ??)
        ## ^
        ## |
        ## +--- first letter 'fastest'

        fixme: possibly wrong when do_data_map found bad file-size
        """
        return axis_order_str(self.hdr, only_letters)

    def looks_ok(self, verbose=1):
        """do some basic checks like filesize, ..."""
        shape = self.data.shape
        b = self.data.dtype.itemsize
        eb = np.prod(shape) * b
        ab = len(self.d)
        secb = np.prod(shape[-2:]) * b

        an_secs = ab / float(secb)
        en_secs = eb / float(secb)

        if verbose >= 3:
            print("expected total data bytes:", eb)
            print("data bytes in file       :", ab)
            print("expected total secs:", en_secs)
            print("file has total secs:", an_secs)

        if eb == ab:
            if verbose >= 2:
                print("OK")
            return 1
        elif eb < ab:
            if verbose >= 1:
                print(
                    "* we have %.2f more (hidden) section in file"
                    % (an_secs - en_secs)
                )
            return 0
        else:
            if verbose >= 1:
                print("* file MISSES %.2f sections " % (en_secs - an_secs))
                print("PLEASE SET shape to ", an_secs, "sections !!! ")
            return 0

    def info(self):
        """print useful information from header"""

        hdr_info(self.hdr)

    def data_with_mrc(self, fn):
        """use this to get 'spiffed up' array"""

        import weakref

        data = self.data
        data = data.view(NdarrayInMrcFile)
        ddd = weakref.proxy(data)
        self.data = ddd
        data.Mrc = self

        return data

    def close(self):
        # As of numpy 1.9, memmap no longer has a close method.  Instead
        # use del for all versions.
        if hasattr(self, "m"):
            del self.m


###########################################################################
###########################################################################
###########################################################################
###########################################################################


def open(path, mode="r"):
    return Mrc2(path, mode)


def load(fn):
    """return 3D array filled with the data
    (non memmap)
    """
    m = open(fn)
    a = m.read_stack(m.hdr.Num[2])
    return a


def save(
    a,
    fn,
    if_exists="ask",
    z_axis_order=None,
    hdr=None,
    hdr_eval="",
    calc_mmm=True,
    ext_ints=None,
    ext_floats=None,
):
    """
    if_exists shoud be one of
       ask
       raise
       overwrite

    (only first letter is checked)

     use z_axis_order if arr.ndim > 3:
       z_axis_order is given in order conform to python(last is fastest)
          (spaces,commas,dots,minuses  are ignored)
       examples:
          4D: time,z,y,x          -->  z_axis_order= 't z'
          5D: time, wave, z,y,x   -->  z_axis_order= 't,z,w'
       refer to Mrc spec 'ImgSequence' (interleaved or not)
       z_axis_order None means:
          3D: 'z'
          4D: 'tz'
          5D: 'tzw'

    if hdr is not None:  copy all fields(except 'Num',...)
    if calc_mmm:  calculate min,max,mean of data set and set hdr field
    if hdr_eval:  exec this string ("hdr" refers to the 'new' header)

    TODO: not implemented yet, ext_ints=None, ext_floats=None
    """
    import os

    if os.path.exists(fn):
        if if_exists[0] == "o":
            pass
        elif if_exists[0] == "a":
            yes = input("overwrite?").lower() == "y"
            if not yes:
                raise RuntimeError("not overwriting existing file '%s'" % fn)
        else:
            raise RuntimeError("not overwriting existing file '%s'" % fn)

    m = Mrc2(fn, mode="w")

    m.init_hdr_for_arr(a, z_axis_order)
    if hdr is not None:
        init_hdr_array_from(m.hdr, hdr)

    if calc_mmm:

        def min_max_median(array):
            return (np.min(array), np.max(array), np.median(array))

        def min_max(array):
            return (np.min(array), np.max(array))

        w_axis = axis_order_str(m.hdr).find("w")
        if w_axis < 0:
            m.hdr.mmm1 = min_max_median(a)
        else:
            nw = m.hdr.NumWaves
            m.hdr.mmm1 = min_max_median(a.take((0,), w_axis))
            if nw >= 2:
                m.hdr.mm2 = min_max(a.take((1,), w_axis))
            if nw >= 3:
                m.hdr.mm3 = min_max(a.take((2,), w_axis))
            if nw >= 4:
                m.hdr.mm4 = min_max(a.take((3,), w_axis))
            if nw >= 5:
                m.hdr.mm5 = min_max(a.take((4,), w_axis))

    if ext_ints is not None or ext_floats is not None:
        raise NotImplementedError("todo: implement ext hdr")

    if hdr_eval:
        import sys

        fr = sys._getframe(1)
        loc = {"hdr": m.hdr}
        loc.update(fr.f_locals)
        glo = fr.f_globals
        exec(hdr_eval, loc, glo)
    m.write_header()
    m.write_stack(a)
    m.close()


###########################################################################
###########################################################################
###########################################################################
###########################################################################


class Mrc2:
    """
    this class is for NON-memmapped access of Mrc files
    sections can be read and written on a by-need basis
    the Mrc2 object itself only handles
       the file-object and
       the header and
       extended header data
       BUT NOT ANY image data


    mode indicates how the file is to be opened:
        'r' for reading,
        'w' for writing (truncating an existing file),
        ['a' does not really make sense here]
        Modes 'r+', 'w+' [and 'a+'] open the file for updating (note that 'w+' truncates the file).
     ('b' for binary mode, is implicitly appended)
    """

    def __init__(self, path, mode="r"):
        """
        path is filename
        mode: same as for Python's open function
            ('b' is implicitly appended !)
            'r'   read-only
            'r+'  read-write
            'w'   write - erases old file !!
        """
        import builtins
        import os

        self._f = builtins.open(path, mode + "b")
        self._path = path
        self._name = os.path.basename(path)
        self._mode = mode

        self._hdr_size = 1024
        self._data_offset = self._hdr_size

        self._file_is_byte_swapped = False

        if mode in ("r", "r+"):
            self._init_from_existing_file()

            self.seek_sec(0)
        else:
            self.hdr = make_hdr_array()
            self.hdr.Num = (0, 0, 0)
            self.hdr.PixelType = 1
            self.hdr.mst = (0, 0, 0)
            self.hdr.m = (1, 1, 1)
            self.hdr.d = (1.0, 1.0, 1.0)
            self.hdr.angle = (90.0, 90.0, 90.0)
            self.hdr.axis = (1, 2, 3)
            self.hdr.mmm1 = (0.0, 0.0, 0.0)
            self.hdr.type = 0
            self.hdr.nspg = 0
            self.hdr.next = 0
            self.hdr.dvid = 0xC0A0
            self.hdr.blank = 0
            self.hdr.NumIntegers = 0
            self.hdr.NumFloats = 0
            self.hdr.sub = 0
            self.hdr.zfac = 2
            self.hdr.mm2 = (0.0, 0.0)
            self.hdr.mm3 = (0.0, 0.0)
            self.hdr.mm4 = (0.0, 0.0)
            self.hdr.ImageType = 0
            self.hdr.LensNum = 0
            self.hdr.n1 = 0
            self.hdr.n2 = 0
            self.hdr.v1 = 0
            self.hdr.v2 = 0
            self.hdr.mm5 = (0.0, 0.0)
            self.hdr.NumTimes = 1
            self.hdr.ImgSequence = 0
            self.hdr.tilt = (0.0, 0.0, 0.0)
            self.hdr.NumWaves = 1
            self.hdr.wave = (0, 0, 0, 0, 0)
            self.hdr.zxy0 = (0.0, 0.0, 0.0)
            self.hdr.NumTitles = 0
            self.hdr.title = " " * 800

            self._shape = None
            self._shape2d = None
            self._dtype = None  # scalar data type of pixels
            self._sec_byte_size = 0

    def init_hdr_for_arr(self, arr, z_axis_order=None):
        """
        use z_axis_order if arr.ndim > 3:
          z_axis_order is given in order conform to python(last is fastest)
             (spaces,commas,dots,minuses  are ignored)
          examples:
             4D: time,z,y,x          -->  z_axis_order= 't z'
             5D: time, wave, z,y,x   -->  z_axis_order= 't,z,w'
          refer to Mrc spec 'ImgSequence' (interleaved or not)
          z_axis_order None means:
             3D: 'z'
             4D: 'tz'
             5D: 'tzw'
        """
        if z_axis_order is None:
            if arr.ndim == 3:
                z_axis_order = "z"
            elif arr.ndim == 4:
                z_axis_order = "tz"
            else:
                z_axis_order = "tzw"
        else:
            import string

            # remove delimiter characters '-., '
            z_axis_order = z_axis_order.translate(
                string.join([chr(i) for i in range(256)], ""), "-., "
            ).lower()

        mrcmode = dtype_to_mrc_mode(arr.dtype.type)
        init_simple(self.hdr, mrcmode, arr.shape)
        if arr.ndim == 2:
            pass
        elif arr.ndim == 3:
            if z_axis_order[-1] == "z":
                self.hdr.ImgSequence = 0
            elif z_axis_order[-1] == "w":
                self.hdr.ImgSequence = 1
                self.hdr.NumWaves = arr.shape[-3]
            elif z_axis_order[-1] == "t":
                self.hdr.ImgSequence = 2
                self.hdr.NumTimes = arr.shape[-3]
            else:
                raise ValueError("unsupported axis order")
        elif arr.ndim == 4:
            if z_axis_order[-2:] == "zt":
                self.hdr.ImgSequence = 2
                self.hdr.NumTimes = arr.shape[-3]
            elif z_axis_order[-2:] == "tz":
                self.hdr.ImgSequence = 0
                self.hdr.NumTimes = arr.shape[-4]
            elif z_axis_order[-2:] == "wz":
                self.hdr.ImgSequence = 0
                self.hdr.NumWaves = arr.shape[-4]
            elif z_axis_order[-2:] == "zw":
                self.hdr.ImgSequence = 1
                self.hdr.NumWaves = arr.shape[-3]
            else:
                raise ValueError("unsupported axis order")
        elif arr.ndim == 5:
            raise ValueError("FIXME TODO: 5D")
        else:
            raise ValueError("unsupported array ndim")

        self._init_when_hdr_array_set()

    def _init_from_existing_file(self):
        self.seek_header()
        hdr_array = np.rec.fromfile(self._f, dtype=mrc_hdr_dtype, shape=1)

        self.hdr = implement_hdr(hdr_array)

        self._nz_before_byte_order = self.hdr.Num[0]
        if self._nz_before_byte_order < 0 or self._nz_before_byte_order > 10000:
            self.hdr._array.dtype = self.hdr._array.dtype.newbyteorder()
            self._file_is_byte_swapped = True

        self._ext_hdr_size = self.hdr.next
        self._ext_hdr_num_ints = self.hdr.NumIntegers
        self._ext_hdr_num_floats = self.hdr.NumFloats
        self._ext_hdr_bytes_per_sec = (
                                          self._ext_hdr_num_ints + self._ext_hdr_num_floats
        ) * 4
        self._data_offset = self._hdr_size + self._ext_hdr_size

        if self._ext_hdr_size > 0 and (
                self._ext_hdr_num_ints > 0 or self._ext_hdr_num_floats > 0
        ):
            n_secs = self._ext_hdr_size // self._ext_hdr_bytes_per_sec
            byteorder = "="
            type_descr = [
                ("int", "%s%di4" % (byteorder, self._ext_hdr_num_ints)),
                ("float", "%s%df4" % (byteorder, self._ext_hdr_num_floats)),
            ]
            self._ext_hdr_array = np.rec.fromfile(
                self._f, dtype=type_descr, shape=n_secs
            )
            if self._file_is_byte_swapped:
                self._ext_hdr_array.newbyteorder()

            self.ext_ints = self._ext_hdr_array.field("int")
            self.ext_floats = self._ext_hdr_array.field("float")

        self._init_when_hdr_array_set()

    def _init_when_hdr_array_set(self):
        nx, ny, nsecs = self.hdr.Num
        self._shape = (nsecs, ny, nx)  # todo: wavelengths , times
        self._shape2d = self._shape[-2:]
        self._dtype = mrc_mode_to_dtype(self.hdr.PixelType)
        self._sec_byte_size = self._dtype.itemsize * np.prod(self._shape2d)

    def set_hdr_for_shape_type(self, shape, type):
        mrcmode = dtype_to_mrc_mode(type)
        self.hdr.PixelType = mrcmode
        self.hdr.Num = shape[-1], shape[-2], np.prod(shape[:-2])
        self._init_when_hdr_array_set()

    def make_extended_hdr(self, num_ints, num_floats, n_secs=None):
        self._ext_hdr_num_ints = self.hdr.NumIntegers = num_ints
        self._ext_hdr_num_floats = self.hdr.NumFloats = num_floats
        self._ext_hdr_bytes_per_sec = (
                                          self._ext_hdr_num_ints + self._ext_hdr_num_floats
        ) * 4

        if n_secs is None:
            n_secs = self._shape[0]

        self._ext_hdr_size = self.hdr.next = min_ext_hdr_size(
            n_secs, self._ext_hdr_bytes_per_sec
        )
        self._data_offset = self._hdr_size + self._ext_hdr_size

        if self._ext_hdr_size > 0 and (
                self._ext_hdr_num_ints > 0 or self._ext_hdr_num_floats > 0
        ):
            n_secs = self._ext_hdr_size // self._ext_hdr_bytes_per_sec
            byteorder = "="
            type_descr = [
                ("int", "%s%di4" % (byteorder, self._ext_hdr_num_ints)),
                ("float", "%s%df4" % (byteorder, self._ext_hdr_num_floats)),
            ]
            self._ext_hdr_array = np.recarray(n_secs, dtype=type_descr)

            self.ext_ints = self._ext_hdr_array.field("int")
            self.ext_floats = self._ext_hdr_array.field("float")

    def info(self):
        """print useful information from header"""

        hdr_info(self.hdr)

    def close(self):
        self._f.close()

    def flush(self):
        self._f.flush()

    def seek_sec(self, i):
        if self._sec_byte_size == 0:
            raise ValueError("not inited yet - unknown shape, type")
        self._f.seek(self._data_offset + i * self._sec_byte_size)

    def seek_header(self):
        self._f.seek(0)

    def seek_ext_header(self):
        self._f.seek(self._hdr_size)

    def read_sec(self, i=None):
        """if i is None read "next" section at current position"""
        if i is not None:
            self.seek_sec(i)

        a = np.fromfile(self._f, self._dtype, np.prod(self._shape2d))
        a.shape = self._shape2d
        return a

    def write_sec(self, a, i=None):
        """if i is None write "next" section at current position"""
        if i is not None:
            self.seek_sec(i)

        return a.tofile(self._f)

    def read_stack(self, nz, i=None):
        """if i is None read "next" section at current position"""
        if i is not None:
            self.seek_sec(i)

        a = np.fromfile(self._f, self._dtype, nz * np.prod(self._shape2d))
        a.shape = (nz,) + self._shape2d
        return a

    def write_stack(self, a, i=None):
        """if i is None write "next" section at current position"""
        if i is not None:
            self.seek_sec(i)

        return a.tofile(self._f)

    def write_header(self, seek_to_0=False):
        self.seek_header()
        self.hdr._array.tofile(self._f)
        if seek_to_0:
            self.seek_sec(0)

    def write_ext_header(self, seek_to_0=False):
        self.seek_ext_header()
        self._ext_hdr_array.tofile(self._f)
        if seek_to_0:
            self.seek_sec(0)


###########################################################################
###########################################################################
###########################################################################
###########################################################################


def min_ext_hdr_size(n_secs, bytes_per_sec):
    """return smallest multiple of 1024 to fit ext_hdr data"""
    import math

    return int(math.ceil(n_secs * bytes_per_sec / 1024.0) * 1024)


def mrc_mode_to_dtype(mode):
    pixel_types = (
        np.uint8,
        np.int16,
        np.float32,
        np.float32,
        np.complex64,
        np.int16,
        np.uint16,
        np.int32,
    )

    if mode < 0 or mode > 7:
        raise RuntimeError(
            "Priism file supports pixeltype 0 to 7 - %d given" % mode
        )

    return pixel_types[int(mode)]


def dtype_to_mrc_mode(dtype):
    if dtype == np.uint8:
        return 0
    if dtype == np.int16:
        return 1
    if dtype == np.float32:
        return 2
    if dtype == np.complex64:
        return 4
    if dtype == np.uint16:
        return 6
    if dtype == np.int32:
        return 7
    raise TypeError("MRC does not support %s (%s)" % (dtype.name, dtype))


def shape_from_hdr(hdr, verbose=0):
    """
    return "smart" shape
    considering num_times, num_wavelengths and hdr.ImgSequence

    if verbose:
        print somthing like: w,t,z,y,x  ot z,y,x
    """
    z_order = hdr.ImgSequence  # , 'Image sequence. 0=ZTW, 1=WZT, 2=ZWT. '),
    nt, nw = hdr.NumTimes, hdr.NumWaves
    nx, ny, nsecs = hdr.Num
    if nt == 0:
        nt = 1
    if nw == 0:
        nw = 1
    nz = nsecs / nt / nw

    if nt == nw == 1:
        shape = (nz, ny, nx)
        order_letters = "zyx"
    elif nz == 1 == nw:
        shape = (nt, ny, nx)
        order_letters = "tyx"
    elif nt == 1 or nw == 1:
        if z_order == 0 or z_order == 2:
            nn = nt
            if nt == 1:
                nn = nw
                order_letters = "wyx"
            else:
                order_letters = "tyx"
            shape = (nn, nz, ny, nx)
        else:  # if z_order == 1:
            if nt == 1:
                shape = (nz, nw, ny, nx)
                order_letters = "zwyx"
            else:
                shape = (nt, nz, ny, nx)
                order_letters = "tzyx"

    else:  # both nt and nw > 1
        if z_order == 0:
            shape = (nw, nt, nz, ny, nx)
            order_letters = "wtzyx"
        elif z_order == 1:
            shape = (nt, nz, nw, ny, nx)
            order_letters = "tzwyx"
        else:  # z_order == 2:
            shape = (nt, nw, nz, ny, nx)
            order_letters = "twzyx"

    if verbose:
        print(",".join(order_letters))
    return shape


def implement_hdr(hdr_array):
    """my hack to allow thinks like a.Mrc.hdr.d = (1,2,3)."""

    class hdr:
        __slots__ = list(mrc_hdr_dtype.names) + ["_array"]

        def __init__(s):
            pass

        def __setattr__(s, n, v):
            hdr_array[n][0] = v

        def __getattr__(s, n):
            if n == "_array":
                return hdr_array
            return hdr_array[n][0]

    return hdr()


def make_hdr_array(buffer=None):
    """class function."""
    if buffer is not None:
        h = buffer
        h.dtype = mrc_hdr_dtype
        import weakref

        h = weakref.proxy(h)
    else:
        h = np.recarray(1, mrc_hdr_dtype)
    return implement_hdr(h)


def hdr_info(hdr):
    """class function."""
    shape = hdr.Num[::-1]
    nz = shape[0]
    num_ints = hdr.NumIntegers
    num_floats = hdr.NumFloats

    print("width:                      ", shape[2])
    print("height:                     ", shape[1])
    print("# total slices:             ", shape[0])

    nt, nw = hdr.NumTimes, hdr.NumWaves

    if nt == 0 or nw == 0:
        print(" ** ERROR ** : NumTimes or NumWaves is zero")
        print("NumTimes:", nt)
        print("NumWaves:", nw)
    else:
        if nt == 1 and nw == 1:
            print
        elif nw == 1:
            print("  (%d times for %d zsecs)") % (nt, nz / nt)
        elif nt == 1:
            print("  (%d waves in %d zsecs)") % (nw, nz / nw)
        else:
            print("  (%d times for %d waves in %d zsecs)") % (
                nt,
                nw,
                nz / nw / nt,
            )

    if nt != 1 or nw != 1:
        print(
            "# slice order:        %d (0,1,2 = (ZTW or WZT or ZWT)"
        ) % hdr.ImgSequence

    print("pixel width x    (um):      "), hdr.d[0]
    print("pixel width y    (um):      "), hdr.d[1]
    print("pixel height     (um):      "), hdr.d[2]

    print("# wavelengths:              "), nw
    print("   wavelength 1  (nm):      "), hdr.wave[0]
    print("    intensity min/max/mean: "), hdr.mmm1[0], hdr.mmm1[1], hdr.mmm1[
        2
    ]
    if nw > 1:
        print("   wavelength 2  (nm):      "), hdr.wave[1]
        print("    intensity min/max:      "), hdr.mm2[0], hdr.mm2[1]
    if nw > 2:
        print("   wavelength 3  (nm):      "), hdr.wave[2]
        print("    intensity min/max:      "), hdr.mm3[0], hdr.mm3[1]
    if nw > 3:
        print("   wavelength 4  (nm):      "), hdr.wave[3]
        print("    intensity min/max:      "), hdr.mm4[0], hdr.mm4[1]
    if nw > 4:
        print("   wavelength 5  (nm):      "), hdr.wave[4]
        print("    intensity min/max:      "), hdr.mm5[0], hdr.mm5[1]

    print("lens type:                  "), hdr.LensNum,
    if hdr.LensNum == 12:
        print(" (60x)")
    elif hdr.LensNum == 13:
        print(" (100x)")
    else:
        print("(??)")

    print("origin   (um) x/y/z:        "), hdr.zxy0[1], hdr.zxy0[2], hdr.zxy0[
        0
    ]

    print("# pixel data type:            "),
    if hdr.PixelType == 0:
        print("8 bit (unsigned)")
    elif hdr.PixelType == 1:
        print("16 bit (signed)")
    elif hdr.PixelType == 2:
        print("32 bit (signed real)")
    elif hdr.PixelType == 3:
        print("16 bit (signed complex integer)")
    elif hdr.PixelType == 4:
        print("32 bit (signed complex real)")
    elif hdr.PixelType == 5:
        print("16 bit (signed) IW_EMTOM")
    elif hdr.PixelType == 6:
        print("16 bit (unsigned short)")
    elif hdr.PixelType == 7:
        print("32 bit (signed long)")
    else:
        print(" ** undefined ** ")

    print("# extended header size:       "), hdr.next,
    if hdr.next > 0:
        n = num_ints + num_floats
        if n > 0:
            print(" (%d secs)") % (hdr.next / (4.0 * n),)
        else:
            print(" (??? secs)")
        print("  (%d ints + %d reals per section)") % (num_ints, num_floats)
    else:
        print
    if hdr.NumTitles < 0:
        print(
            " ** ERROR ** : NumTitles less than zero (NumTitles =",
            hdr.NumTitles,
            ")",
        )
    elif hdr.NumTitles > 0:
        n = hdr.NumTitles
        if n > 10:
            print(
                " ** ERROR ** : NumTitles larger than 10 (NumTitles =",
                hdr.NumTitles,
                ")",
            )
            n = 10
        for i in range(n):
            print("title %d: %s") % (i, hdr.title[i])


def axis_order_str(hdr, only_letters=True):
    """return string indicating meaning of shape dimensions
    ##
    ## ZTW   <- non-interleaved
    ## WZT   <- OM1 ( easy on stage)
    ## ZWT   <- added by API (used at all ??)
    ## ^
    ## |
    ## +--- first letter 'fastest'

    fixme: possibly wrong when do_data_map found bad file-size
    """
    z_order = int(
        hdr.ImgSequence
    )  # , 'Image sequence. 0=ZTW, 1=WZT, 2=ZWT. '),
    nt, nw = hdr.NumTimes, hdr.NumWaves
    if nt == nw == 1:
        order_letters = "zyx"
    elif nt == 1:
        order_letters = ("wzyx", "zwyx", "wzyx")[z_order]
    elif nw == 1:
        order_letters = ("tzyx", "tzyx", "tzyx")[z_order]
    else:
        order_letters = ("wtzyx", "tzwyx", "twzyx")[z_order]

    if only_letters:
        return order_letters
    else:
        return "[" + ",".join(order_letters) + "]"


def init_simple(hdr, mode, nx_or_shape, ny=None, nz=None):
    """note: if  nx_or_shape is tuple it is nz,ny,nx (note the order!!)"""
    if ny is nz is None:
        if len(nx_or_shape) == 2:
            nz, (ny, nx) = 1, nx_or_shape
        elif len(nx_or_shape) == 1:
            nz, ny, nx = 1, 1, nx_or_shape
        elif len(nx_or_shape) == 3:
            nz, ny, nx = nx_or_shape
        else:
            ny, nx = nx_or_shape[-2:]
            nz = np.prod(nx_or_shape[:-2])

    else:
        nx = nx_or_shape

    hdr.Num = (nx, ny, nz)
    hdr.PixelType = mode
    hdr.mst = (0, 0, 0)
    hdr.m = (1, 1, 1)
    hdr.d = (1, 1, 1)
    hdr.angle = (90, 90, 90)
    hdr.axis = (1, 2, 3)
    hdr.mmm1 = (0, 100000, 5000)
    hdr.type = 0
    hdr.nspg = 0
    hdr.next = 0
    hdr.dvid = -16224
    hdr.blank = 0
    hdr.NumIntegers = 0
    hdr.NumFloats = 0
    hdr.sub = 0
    hdr.zfac = 0
    hdr.mm2 = (0, 10000)
    hdr.mm3 = (0, 10000)
    hdr.mm4 = (0, 10000)
    hdr.ImageType = 0
    hdr.LensNum = 0
    hdr.n1 = 0
    hdr.n2 = 0
    hdr.v1 = 0
    hdr.v2 = 0
    hdr.mm5 = (0, 10000)
    hdr.NumTimes = 1
    hdr.ImgSequence = 0  # Zero => not interleaved. That means z changes
    # //                           // fastest, then time, then waves;
    # //                           // 1 => interleaved. That means wave changes fastest,
    # //                           // then z, then time.
    hdr.tilt = (0, 0, 0)
    hdr.NumWaves = 1
    hdr.wave = (999, 0, 0, 0, 0)
    hdr.zxy0 = (0, 0, 0)
    hdr.NumTitles = 0
    hdr.title = "\0" * 80


def init_hdr_array_from(hdr_dest, hdr_src):
    """copy all field of the header
    EXCEPT  shape AND PixelType AND all fields related to extended hdr
    """

    hdr_dest.mst = hdr_src.mst
    hdr_dest.m = hdr_src.m
    hdr_dest.d = hdr_src.d
    hdr_dest.angle = hdr_src.angle
    hdr_dest.axis = hdr_src.axis
    hdr_dest.mmm1 = hdr_src.mmm1
    hdr_dest.type = hdr_src.type
    hdr_dest.nspg = hdr_src.nspg
    hdr_dest.next = 0
    hdr_dest.dvid = hdr_src.dvid
    hdr_dest.blank = hdr_src.blank
    hdr_dest.NumIntegers = 0
    hdr_dest.NumFloats = 0
    hdr_dest.sub = hdr_src.sub
    hdr_dest.zfac = hdr_src.zfac
    hdr_dest.mm2 = hdr_src.mm2
    hdr_dest.mm3 = hdr_src.mm3
    hdr_dest.mm4 = hdr_src.mm4
    hdr_dest.ImageType = hdr_src.ImageType
    hdr_dest.LensNum = hdr_src.LensNum
    hdr_dest.n1 = hdr_src.n1
    hdr_dest.n2 = hdr_src.n2
    hdr_dest.v1 = hdr_src.v1
    hdr_dest.v2 = hdr_src.v2
    hdr_dest.mm5 = hdr_src.mm5
    hdr_dest.NumTimes = hdr_src.NumTimes
    hdr_dest.ImgSequence = hdr_src.ImgSequence
    hdr_dest.tilt = hdr_src.tilt
    hdr_dest.NumWaves = hdr_src.NumWaves
    hdr_dest.wave = hdr_src.wave
    hdr_dest.zxy0 = hdr_src.zxy0
    hdr_dest.NumTitles = hdr_src.NumTitles
    hdr_dest.title = hdr_src.title


def set_title(hdr, s, i=-1):
    """set title i (i==-1 means "append") to s"""

    n = hdr.NumTitles

    if i < 0:
        i = n
    if i > 9:
        raise ValueError("Mrc only support up to 10 titles (0<=i<10)")
    if len(s) > 80:
        raise ValueError("Mrc only support title up to 80 characters")
    if i >= n:
        hdr.NumTitles = i + 1

    if len(s) == 80:
        hdr.title[i] = s
    else:
        hdr.title[i] = s + "\0"


def adjusted_data_shape(numel, shape):
    """Return shape to use for a certain number of elements.

    There are truncated mrc/dv files.  May be that the file got
    truncated during a copy, or may be that comes from an experiment
    that was not completed.  See cockpit bug #289.  This function
    returns the smallest shape that data with numel elements can be
    resized to, while including all available data and keeping the
    length of the dimensions.

    Examples:

    * numel = 20; expected_shape = (10,10).  Returns (2,10).
    * numel = 15; expected_shape = (10,10).  Returns (2,10).

    """
    numel = int(numel)
    shape = tuple([int(s) for s in shape])
    if numel == np.prod(shape):
        return shape  # data is complete, nothing to do
    elif numel == 0:
        return tuple([0] * len(shape))  # special case with empty data
    elif numel > np.prod(shape):
        raise ValueError(
            (
                "data too large (%i elements) for proposed shape %s"
                % (numel, str(shape))
            )
        )

    shape = list(shape)
    for i in range(len(shape)):
        stride = int(np.prod(shape[i + 1 :]))
        if numel >= stride:
            shape[i] = numel // stride
            if numel % stride:
                ## The truncated data may not fit in an array with
                ## truncated shape.  For example, 15 elements to fill
                ## a 10x2 array.  In such cases, expand the truncated
                ## shape to include all data, even if that means an
                ## array larger than the data.  See cockpit bug #289.
                shape[i] += 1
            break
        else:
            shape[i] = 1

    return tuple(shape)


mrc_hdr_dtype = np.dtype(
    [
        ("Num", "i4", (3,)),
        ("PixelType", "i4"),
        ("mst", "i4", (3,)),
        ("m", "i4", (3,)),
        ("d", "f4", (3,)),
        ("angle", "f4", (3,)),
        ("axis", "i4", (3,)),
        ("mmm1", "f4", (3,)),
        ("type", "i2"),
        ("nspg", "i2"),
        ("next", "i4"),
        ("dvid", "i2"),
        ("blank", "i1", (30,)),
        (
            "NumIntegers",
            "i2",
        ),  # Number of 4 byte integers stored in the extended header per section.
        (
            "NumFloats",
            "i2",
        ),  # Number of 4 byte floating-point numbers stored in the extended header per section.
        (
            "sub",
            "i2",
        ),  # Number of sub-resolution data sets stored within the image. Typically, this equals 1.
        (
            "zfac",
            "i2",
        ),  # Reduction quotient for the z axis of the sub-resolution images.
        ("mm2", "f4", (2,)),  # Minimum intensity of the 2nd wavelength image.
        ("mm3", "f4", (2,)),  # Minimum intensity of the 2nd wavelength image.
        ("mm4", "f4", (2,)),  # Minimum intensity of the 2nd wavelength image.
        ("ImageType", "i2"),  # Image type. See Image Type table below.
        ("LensNum", "i2"),  # Lens identification number.
        ("n1", "i2"),  # Depends on the image type.
        ("n2", "i2"),  # Depends on the image type.
        ("v1", "i2"),  # Depends on the image type.
        ("v2", "i2"),  # Depends on the image type.
        ("mm5", "f4", (2,)),  # Minimum intensity of the 2nd wavelength image.
        ("NumTimes", "i2"),  # Number of time points.
        ("ImgSequence", "i2"),  # Image sequence. 0=ZTW, 1=WZT, 2=ZWT.
        ("tilt", "f4", (3,)),  # X axis tilt angle (degrees).
        ("NumWaves", "i2"),  # Number of wavelengths.
        ("wave", "i2", (5,)),  # Wavelength 1, in nm.
        ("zxy0", "f4", (3,)),  # X origin, in µm.
        (
            "NumTitles",
            "i4",
        ),  # Number of titles. Valid numbers are between 0 and 10.
        ("title", "S80", (10,)),  # Title 1. 80 characters long.
    ]
)
