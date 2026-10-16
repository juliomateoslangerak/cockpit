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


## This module exists to make interacting with MRC files more pleasant from a
# programming perspective. I (Chris Weisiger) originally wrote it for the OMX
# Editor program, since I needed to be able to construct multiple views and
# transformations of data on the fly. It's since expanded to provide multiple
# utility functions for reading and writing MRC files and headers.

import numpy
import scipy.ndimage
import wx

from cockpit.util import mrc


## Maps dimensional axes to their labels.
DIMENSION_LABELS = ["Wavelength", "Time", "Z", "Y", "X"]


class DataDoc:
    """The DataDoc class is, broadly, a wrapper around the mrc module.

    When it loads a file, it loads all of the data in that file, and then makes
    it available as an array in WTZYX order (regardless of the order in which
    the data is stored in the MRC file). It additionally exposes some
    attributes of the MRC metadata, and provides functions for transforming and
    projecting the data array.
    """

    def __init__(self, filename=""):
        """Instantiate the object.

        Args:
            filename (str): The filename of the MRC file you want to load.
        """
        ## Loaded MRC object. Note this is not just an array of pixels.
        self.image = mrc.bind_file(filename)
        ## Header for the image data, which tells us e.g. what the ordering
        # of X/Y/Z/time/wavelength is in the MRC file.
        self.image_header = mrc.implement_hdr(self.image.Mrc.hdr._array.copy())
        ## Location the file is saved on disk.
        self.file_path = self.image.Mrc.path

        ## Number of wavelengths in the array.
        self.num_wavelengths = self.image_header.NumWaves
        num_timepoints = self.image_header.NumTimes
        num_x = self.image_header.Num[0]
        num_y = self.image_header.Num[1]
        num_z = self.image_header.Num[2] // (self.num_wavelengths * num_timepoints)
        ## Size in pixels of the data, since having it as a Numpy array
        # instead of a tuple (from self.image_array.shape) is occasionally
        # handy.
        self.size = numpy.array(
            [self.num_wavelengths, num_timepoints, num_z, num_y, num_x], dtype=int
        )
        ## 5D array of pixel data, indexed as
        # self.image_array[wavelength][time][z][y][x]
        # In other words, in WTZYX order. In general we try to treat
        # Z and time as "just another axis", but wavelength is dealt with
        # specially.
        self.image_array = self.get_image_array()
        ## Two arrays, one for ints, one for floats, for the extended header.
        # Indexed as
        # self.extended_header_ints[wavelength, time, z, i]
        # Either may be empty, depending on how big the extended header is.
        self.extended_header_ints, self.extended_header_floats = get_extended_header(
            self.image.Mrc.e, self.image_header
        )
        ## Datatype of our array.
        self.dtype = self.image_array.dtype.type

        ## Averages for each wavelength, used to provide fill values when
        # taking slices.
        self.averages = []
        for wavelength in range(self.num_wavelengths):
            self.averages.append(self.image_array[wavelength].mean())

        ## Lower boundary of the cropped data.
        self.crop_min = numpy.array([0, 0, 0, 0, 0], numpy.int32)
        ## Upper boundary of the cropped data.
        self.crop_max = numpy.array(self.size, numpy.int32)

        ## Index of the single pixel that is visible in all different data
        # views.
        self.cur_view_index = numpy.array(self.size / 2, int)
        # Initial time view is at 0
        self.cur_view_index[1] = 0

        ## Parameters for transforming different wavelengths so they align
        # with each other. Order is dx, dy, dz, angle, zoom
        self.align_params = numpy.zeros((self.size[0], 5), numpy.float32)
        # Default zoom to 1.0
        self.align_params[:, 4] = 1.0

        ## List of functions to call whenever the alignment parameters change.
        # Each will be passed self.align_params so it can take whatever
        # action is necessary.
        self.align_callbacks = []

    def get_n_planes(self):
        return numpy.prod(self.size[0:3])

    def get_image_array(self):
        """Convert the loaded MRC image data into a 5D array of pixel data in
        WTZYX order."""
        # This is a string describing the dimension ordering as stored in
        # the file.
        return reorder_array(
            self.image, self.size, self.image.Mrc.axis_order_str()
        )

    def take_slice(self, axes, should_transform=True, order=1):
        """Passthrough to take_slice_from_data, using our normal array."""
        return self.take_slice_from_data(
            self.image_array, axes, should_transform, order
        )

    def take_projected_slice(
        self, axes, projection_axis, should_transform, order=1
    ):
        r"""As take_slice, but do a max-intensity projection across one axis.

        This becomes impossible to do efficiently if we have rotation or
        scaling in a given wavelength, so we just have to transform the entire
        volume. It gets *really* expensive if we want to do projections across
        time with this... \todo We could be a bit more efficient here since
        only the wavelengths with nonzero rotation/scaling need to be
        transformed as volumes.
        """
        if projection_axis == 2 or (
            numpy.all(self.align_params[:, 3] == 0)
            and numpy.all(self.align_params[:, 4] == 1)
        ):
            # Scaling/rotation doesn't affect the projection; lucky us!
            data = self.image_array.max(axis=projection_axis)
            # Augment data with an extra dimension to replace the one we
            # flattened out.
            data = numpy.expand_dims(data, projection_axis)
            # Since we flattened out this axis, change its index to be the only
            # possible valid index.
            axes[projection_axis] = 0
            return self.take_slice_from_data(data, axes, should_transform, order)
        elif projection_axis in [3, 4]:
            # Projecting through Y or X; just transform the local volume.
            dialog = wx.ProgressDialog(
                title="Constructing projection",
                message="Please wait...",
                maximum=self.size[0],
                style=wx.PD_AUTO_HIDE | wx.PD_REMAINING_TIME,
            )
            cur_timepoint = self.cur_view_index[1]
            data = []
            for wavelength in range(self.size[0]):
                data.append(
                    self.transform_array(
                        self.image_array[wavelength, cur_timepoint],
                        *self.align_params[wavelength],
                        order=1,
                    )
                )
                dialog.Update(wavelength)
            data = numpy.array(data, dtype=self.dtype)
            dialog.Destroy()
            return data.max(axis=projection_axis - 1)
        else:
            # Projecting through time; transform EVERY volume. Ouch.
            dialog = wx.ProgressDialog(
                title="Constructing projection",
                message="Please wait...",
                maximum=self.size[0] * self.size[1],
                style=wx.PD_AUTO_HIDE | wx.PD_REMAINING_TIME,
            )
            data = []
            for timepoint in range(self.size[1]):
                time_data = []
                for wavelength in range(self.size[0]):
                    volume = self.transform_array(
                        self.image_array[wavelength, timepoint],
                        *self.align_params[wavelength],
                        order=1,
                    )
                    time_data.append(volume)
                    dialog.Update(timepoint * self.size[0] + wavelength)
                time_data = numpy.array(time_data, dtype=self.dtype)
                data.append(time_data)

            data = numpy.array(data, dtype=self.dtype)
            data = data.max(axis=0)
            dialog.Destroy()
            # Slice through data per our axes parameter.
            slice = [Ellipsis] * 4
            for axis, position in axes.items():
                if axis != 1:
                    slice[axis - 1] = position
                    return data[slice]
            raise RuntimeError("Couldn't find a valid slice axis.")

    def take_slice_from_data(self, data, axes, should_transform=True, order=1):
        """Generate a 2D slice of the given data in each wavelength.

        Since the data is 5D (wavelength/time/Z/Y/X), there are three axes to
        be perpendicular to, one of which is always wavelength. The "axes"
        argument maps the other two axis indices to the coordinate the slice
        should pass through. E.g. passing in {1: 10, 2: 32} means to take a WXY
        slice at timepoint 10 through Z index 32. This was fairly complicated
        for me to figure out, since I'm not a scientific programmer, so I'm
        including my general process here: - Figure out which axes the slice
        cuts across, and generate an array of the appropriate shape to hold the
        results. - Create an array of similar size augmented with a length-4
        dimension. This array holds XYZ coordinates for each pixel in the
        slice; the 4th index holds a 1 (so that we can use a 4x4 affine
        transformation matrix to do rotation and offsets in the same pass). For
        example, an XY slice at Z = 5 would look something like this: [  [0, 0,
        5]  [0, 1, 5]  [0, 2, 5] ... [  [1, 0, 5]  ... [  [2, 0, 5] [  ... [ -
        Subtract the XYZ center off of the coordinates so that when we apply
        the rotation transformation, it's done about the center of the dataset
        instead of the corner. - Multiply the inverse transformation matrix by
        the coordinates. - Add the center back on. - Chop off the dummy 1
        coordinate, reorder to ZYX, and prepend the time dimension. - Pass the
        list of coordinates off to numpy.map_coordinates so it can look up
        actual pixel values. - Reshape the resulting array to match the slice
        shape.
        """
        if should_transform:
            target_shape = []
            target_axes = []
            presets = [-1] * 5
            # Generate an array to hold the slice. Note this includes all
            # wavelengths.
            for i, size in enumerate(data.shape):
                if i not in axes:
                    target_shape.append(size)
                    target_axes.append(i)
                else:
                    presets[i] = axes[i]

            # Create a matrix of size (NxMx3) where N and M are the width
            # and height of the desired slice, and the remaining dimension
            # holds the desired XYZ coordinates for each pixel in the slice,
            # pre-transform. Note this is wavelength-agnostic.
            target_coords = numpy.empty(target_shape[1:] + [3])
            have_already_resized = False
            # Axes here are in WTZYX order, so we need to reorder them to XYZ.
            for axis in [2, 3, 4]:
                if axis in target_axes:
                    basis = numpy.arange(data.shape[axis])
                    if (
                        data.shape[axis] == target_coords.shape[0]
                        and not have_already_resized
                    ):
                        # Reshape into a column vector. We only want to do this
                        # once, but unfortunately can't tell solely with the
                        # length of the array in the given axis since it's not
                        # uncommon for e.g. X and Y to have the same size.
                        basis.shape = data.shape[axis], 1
                        have_already_resized = True
                    target_coords[:, :, 4 - axis] = basis
                else:
                    target_coords[:, :, 4 - axis] = axes[axis]
            return self.map_coords(data, target_coords, target_shape, axes, order)
        else:
            # Simply take an ordinary slice.
            # Ellipsis is a builtin keyword for the full-array slice. Who knew?
            slices = [Ellipsis]
            for axis in range(1, 5):
                if axis in axes:
                    slices.append(axes[axis])
                else:
                    slices.append(Ellipsis)
            return data[slices]

    def map_coords(self, data, target_coords, target_shape, axes, order):
        """Inverse-transform the provided coordinates and use them to look up
        into the given array, to generate a transformed slice of the specified
        shape along the specified axes.

        Lower order is faster but less accurate.

        Args:
            data (numpy.ndarray): A 5D array of pixel data (WTZYX).
            target_coords (numpy.ndarray): 4D array of WXYZ coordinates.
            target_shape (tuple): Shape of the resulting slice.
            axes (dict): Axes the slice cuts along.
            order (int): Spline order to use when mapping.
        """
        # Reshape into a 2D list of the desired coordinates
        target_coords.shape = numpy.product(target_shape[1:]), 3
        # Insert a dummy 4th dimension so we can use translation in an
        # affine transformation.
        tmp = numpy.empty((target_coords.shape[0], 4))
        tmp[:, :3] = target_coords
        tmp[:, 3] = 1
        target_coords = tmp

        transforms = self.get_transformation_matrices()
        inverse_transforms = [numpy.linalg.inv(matrix) for matrix in transforms]
        transposed_coords = target_coords.T
        # XYZ center, which needs to be added and subtracted from the
        # coordinates before/after transforming so that rotation is done
        # about the center of the image.
        center = numpy.array(data.shape[2:][::-1]).reshape(3, 1) / 2.0
        transposed_coords[:3, :] -= center
        result = numpy.zeros(target_shape, dtype=self.dtype)
        for wavelength in range(data.shape[0]):
            # Transform the coordinates according to the alignment
            # parameters for the specific wavelength.
            transformed_coords = numpy.dot(
                inverse_transforms[wavelength], transposed_coords
            )
            transformed_coords[:3, :] += center

            # Chop off the trailing 1, reorder to ZYX, and insert the time
            # coordinate.
            tmp = numpy.zeros(
                (4, transformed_coords.shape[1]), dtype=numpy.float
            )
            for i in range(3):
                tmp[i + 1] = transformed_coords[2 - i]

            transformed_coords = tmp
            if 1 not in axes:
                # User wants a cut across time.
                transformed_coords[0, :] = numpy.arange(data.shape[1]).repeat(
                    transformed_coords.shape[1] / data.shape[1]
                )
            else:
                transformed_coords[0, :] = axes[1]

            result_vals = scipy.ndimage.map_coordinates(
                data[wavelength],
                transformed_coords,
                order=order,
                cval=self.averages[wavelength],
            )
            result_vals.shape = target_shape[1:]
            result[wavelength] = result_vals

        return result

    def get_values_at(self, coord):
        """Return the value for each wavelength at the specified TZYX
        coordinate, taking transforms into account.

        Also return the transformed coordinates.

        \\todo This copies a fair amount of logic from self.map_coords.
        """
        transforms = self.get_transformation_matrices()
        inverse_transforms = [numpy.linalg.inv(matrix) for matrix in transforms]
        # Reorder to XYZ and add a dummy 4th dimension.
        transposed_coord = numpy.array(
            [[coord[3]], [coord[2]], [coord[1]], [1]]
        )
        # XYZ center, which needs to be added and subtracted from the
        # coordinates before/after transforming so that rotation is done
        # about the center of the image.
        center = (self.size[2:][::-1] / 2.0).reshape(3, 1)
        transposed_coord[:3] -= center
        result_vals = numpy.zeros(self.num_wavelengths, dtype=self.dtype)
        result_coords = numpy.zeros((self.num_wavelengths, 4))
        for wavelength in range(self.num_wavelengths):
            # Transform the coordinates according to the alignment
            # parameters for the specific wavelength.
            transformed_coord = numpy.dot(
                inverse_transforms[wavelength], transposed_coord
            )
            transformed_coord[:3, :] += center
            # Reorder to ZYX and insert the time dimension.
            transformed_coord = numpy.array(
                [
                    coord[0],
                    transformed_coord[2],
                    transformed_coord[1],
                    transformed_coord[0],
                ],
                dtype=int,
            )
            result_coords[wavelength, :] = transformed_coord
            transformed_coord.shape = 4, 1

            result_vals[wavelength] = scipy.ndimage.map_coordinates(
                self.image_array[wavelength],
                transformed_coord,
                order=1,
                cval=self.averages[wavelength],
            )[0]
        return result_vals, result_coords

    def take_default_slice(self, perpendicular_axes, should_transform=True):
        """Take a default slice through our view indices perpendicular to the
        given axes."""
        target_coords = self.get_slice_coords(perpendicular_axes)
        return self.take_slice(target_coords, should_transform)

    def get_transformation_matrices(self):
        """Generate a 4D transformation matrix based on self.align_params for
        each wavelength."""
        result = []
        for wavelength in range(self.num_wavelengths):
            dx, dy, dz, angle, zoom = self.align_params[wavelength]
            angle = angle * numpy.pi / 180.0
            cos_theta = numpy.cos(angle)
            sin_theta = numpy.sin(angle)
            transform = zoom * numpy.array(
                [
                    [cos_theta, sin_theta, 0, dx],
                    [-sin_theta, cos_theta, 0, dy],
                    [0, 0, 1, dz],
                    [0, 0, 0, 1],
                ]
            )
            result.append(transform)
        return result

    def has_z_motion(self):
        """Return true if there is any Z motion in any wavelength's alignment
        parameters."""
        return numpy.any(self.align_params[:, 2] != 0)

    def has_transformation(self):
        """Return true if there is any non-default transformation."""
        for i, null_value in enumerate([0, 0, 0, 0, 1]):
            if not numpy.all(self.align_params[:, i] == null_value):
                # At least one wavelength has a transformation here.
                return True
        return False

    def register_alignment_callback(self, callback):
        """Register a callback to be invoked when the alignment parameters
        change."""
        self.align_callbacks.append(callback)

    def set_align_params(self, wavelength, params):
        """Update the alignment parameters, then invoke our callbacks."""
        self.align_params[wavelength] = params
        for callback in self.align_callbacks:
            callback(self.align_params)

    def get_align_params(self, wavelength):
        """Get the current alignment parameters for the specified wavelength."""
        return self.align_params[wavelength]

    def align_and_crop(self, wavelengths=[], timepoints=[], save_path=None):
        r"""Apply our alignment parameters to the data, then crop them, and
        either return the result for the specified wavelength(s), or save the
        result to the specified file path.

        If no wavelengths are specified, use them all.
        \todo All of the logic dealing with the MRC file writing is basically
        \copied from the old imdoc module, and I don't claim to understand why
        \it does what it does.
        \todo The extended header is not preserved. On the flip side, according
        \to Eric we don't currently use the extended header anyway, so it was
        \just wasting space.
        """
        if not wavelengths:
            wavelengths = range(self.size[0])
        if not timepoints:
            timepoints = range(self.crop_min[1], self.crop_max[1])

        # Generate the cropped shape of the file.
        cropped_shape = [len(wavelengths)]
        for min, max in zip(self.crop_min[1:], self.crop_max[1:]):
            cropped_shape.append(max - min)
        # Reorder to time/wavelength/z/y/x for saving.
        cropped_shape[0], cropped_shape[1] = cropped_shape[1], cropped_shape[0]
        cropped_shape = tuple(cropped_shape)

        new_header = mrc.make_hdr_array()
        mrc.init_hdr_array_from(new_header, self.image_header)
        new_header.Num = (
            cropped_shape[4],
            cropped_shape[3],
            cropped_shape[2] * len(timepoints) * len(wavelengths),
        )
        new_header.NumTimes = len(timepoints)
        new_header.NumWaves = len(wavelengths)
        # Size of the extended header -- forced to zero for now.
        new_header.next = 0
        # Ordering of data in the file; 2 means z/w/t
        new_header.ImgSequence = 2
        new_header.PixelType = mrc.dtype_to_mrc_mode(numpy.float32)

        if not save_path:
            output_array = numpy.empty(cropped_shape, numpy.float32)
        else:
            if self.file_path == save_path:
                # \todo Why do we do this?
                del self.image.Mrc

            # Write out the header.
            output_file = open(save_path, "wb")
            output_file.write(new_header._array.tobytes())

        # Slices to use to crop out the 3D volume we want to use for each
        # wave-timepoint pair.
        volume_slices = []
        for min, max in zip(self.crop_min[2:], self.crop_max[2:]):
            volume_slices.append(slice(min, max))

        for timepoint in timepoints:
            for wave_index, wavelength in enumerate(wavelengths):
                volume = self.image_array[wavelength][timepoint]

                dx, dy, dz, angle, zoom = self.align_params[wavelength]
                if dz and self.size[2] == 1:
                    # HACK: no Z translate in 2D files. Even
                    # infinitesimal translates will zero out the entire slice,
                    # otherwise.
                    dz = 0
                if dx or dy or dz or angle or zoom != 1:
                    # Transform the volume.
                    volume = self.transform_array(
                        volume, dx, dy, dz, angle, zoom
                    )
                # Crop to the desired shape.
                volume = volume[volume_slices].astype(numpy.float32)

                if not save_path:
                    output_array[timepoint, wave_index] = volume
                else:
                    # Write to the file.
                    for i, z_slice in enumerate(volume):
                        output_file.write(z_slice)

        if not save_path:
            # Reorder to WTZYX since that's what the user expects.
            return output_array.transpose([1, 0, 2, 3, 4])
        else:
            output_file.close()

    def save_to(self, save_path):
        """Just save our array to the specified file."""
        filehandle = open(save_path, "wb")
        write_mrc_header(self.image_header, filehandle)
        filehandle.write(self.image_array)
        filehandle.close()

    def get_slice_size(self, axis1, axis2):
        """Get the size of a slice in the specified dimensions.

        Dimensions are as ordered in self.size
        """
        return numpy.array([self.size[axis1], self.size[axis2]])

    def get_slice_coords(self, axes=None):
        """Returns a mapping of axes to our view positions on those axes.

        If None, then operate on all axes.

        Args:
            axes (list): A list of axes, e.g. [0, 3] for (time, Y).
        """
        if axes is None:
            axes = range(5)
        return dict([(axis, self.cur_view_index[axis]) for axis in axes])

    def move_slice_lines(self, offset):
        """Move self.cur_view_index by the specified amount, ensuring that we
        stay in-bounds."""
        for i, delta in enumerate(offset):
            target_val = self.cur_view_index[i] + delta
            if target_val >= 0 and target_val < self.size[i]:
                self.cur_view_index[i] = target_val

    def move_crop_box(self, offset, is_min):
        """Move the crop box by the specified amount, ensuring that we stay
        in-bounds."""
        if is_min:
            self.crop_min += offset
            for i, val in enumerate(self.crop_min):
                self.crop_min[i] = max(0, min(self.size[i], val))
        else:
            self.crop_max += offset
            for i, val in enumerate(self.crop_max):
                self.crop_max[i] = max(0, min(self.size[i], val))

    def convert_to_microns(self, offsets):
        """Multiply the given XYZ offsets by our pixel sizes to get offsets in
        microns."""
        return numpy.multiply(offsets, self.image_header.d)

    def convert_from_microns(self, offsets):
        """As convert_to_microns, but in reverse."""
        return numpy.divide(offsets, self.image_header.d)

    def transform_array(self, data, dx, dy, dz, angle, zoom, order=3):
        """Apply a transformation to an input 3D array in ZYX order.

        Angle rotates each slice, zoom scales each slice (i.e. neither is 3D).
        """
        # Input angle is in degrees, but scipy's transformations expect angles
        # in radians.
        angle = angle * numpy.pi / 180
        cos_theta = numpy.cos(-angle)
        sin_theta = numpy.sin(-angle)
        affine_transform = zoom * numpy.array(
            [[cos_theta, sin_theta], [-sin_theta, cos_theta]]
        )

        inverted_transform = numpy.linalg.inv(affine_transform)
        yx_center = numpy.array(data.shape[1:]) / 2.0
        offset = -numpy.dot(inverted_transform, yx_center) + yx_center

        output = numpy.zeros(data.shape)
        for i, slice in enumerate(data):
            output[i] = scipy.ndimage.affine_transform(
                slice,
                inverted_transform,
                offset,
                output=numpy.float32,
                cval=slice.min(),
                order=order,
            )
        output = scipy.ndimage.shift(output, [dz, dy, dx], order=order)
        return output


def make_header_for(data, should_set_min_max=True, **kwargs):
    """Generate an MRC header object based on the provided Numpy array.

    The input array must be five-dimensional, in WTZYX order. Just a
    passthrough to make_header_for_shape, really.

    Args:
        data (numpy.ndarray): Array of pixels in (W, T, Z, Y, X) order.
        should_set_min_max (bool): If True (the default), calculate the min and
            max values for each wavelength and put them in the header.
            Pointless if this will be overridden later, and may be costly
            depending on the size of the data array.
    """
    header = make_header_for_shape(data.shape, data.dtype.type, **kwargs)
    if should_set_min_max:
        # Set the min/max values. This is a bit ugly because they're in
        # differently-named fields on a per-wavelength basis...and the first
        # wavelength gets extra data stored, to boot.
        for i in range(data.shape[0]):
            min_val = data[i].min()
            max_val = data[i].max()
            if i == 0:
                setattr(
                    header, "mmm1", (min_val, max_val, numpy.median(data[i]))
                )
            else:
                setattr(header, "mm%d" % (i + 1), (min_val, max_val))
    return header


def make_header_for_shape(
    shape, dtype, xy_size=None, z_size=None, wavelengths=[], zxy0=None
):
    """Generate an MRC header, filling in some values.

    Args:
        shape (tuple): 5D tuple describing the shape of the data. Must be in
            (W, T, Z, Y, X) order.
        dtype (type): Datatype of the pixel data.
        xy_size (float): Size of the (assumed square) pixels. Defaults to our
            current camera pixel size.
        z_size (float): Size of the Z-step between slices. Defaults to 0.
        wavelengths (list): List of wavelengths (e.g. [488, 560]). Defaults to
            the currently active wavelengths.
    """
    header = mrc.make_hdr_array()
    mrc.init_simple(header, mrc.dtype_to_mrc_mode(dtype), shape)
    header.NumTimes = shape[1]
    header.NumWaves = shape[0]
    header.Num = (shape[4], shape[3], shape[0] * shape[1] * shape[2])
    for i, wavelength in enumerate(wavelengths):
        if wavelength:
            header.wave[i] = wavelength
        else:
            header.wave[i] = 0
    header.ImgSequence = 2
    header.d = [xy_size, xy_size, z_size]
    if zxy0 is not None:
        header.zxy0 = zxy0
    return header


def write_mrc_header(header, filehandle):
    """Write just a header to the provided filehandle."""
    filehandle.seek(0)
    filehandle.write(header._array.tobytes())


def write_data_as_mrc(
    data, filename, xy_size=None, z_size=None, wavelengths=[], zxy0=None
):
    """Write out the provided data array as if it were an MRC file.

    Note that the input array must be in WTZYX order; if the array has
    insufficient dimensions it will be augmented with dimensions of size 1
    starting from the left (e.g. a 512x512 array becomes a 1x1x1x512x512
    array).
    """
    shape = (5 - len(data.shape)) * [1] + list(data.shape)
    data_out = data.reshape(shape)
    header = make_header_for(
        data_out,
        xy_size=xy_size,
        z_size=z_size,
        wavelengths=wavelengths,
        zxy0=zxy0,
    )
    handle = open(filename, "wb")
    write_mrc_header(header, handle)
    handle.seek(1024)  # Seek to end of header
    data_out.tofile(handle)
    handle.close()


def write_data_as_mrc_with_exthdr(
    data,
    filename,
    xy_size=None,
    z_size=None,
    wavelengths=[],
    zxy0=None,
    lens_id=None,
    int_metadata_buffers=[],
    float_metadata_buffers=[],
):
    """Write out the provided data array as if it were an MRC file.

    Note that the input array must be in WTZYX order; if the array has
    insufficient dimensions it will be augmented with dimensions of size 1
    starting from the left (e.g. a 512x512 array becomes a 1x1x1x512x512
    array).
    """
    shape = (5 - len(data.shape)) * [1] + list(data.shape)
    data_out = data.reshape(shape)
    header = make_header_for(
        data_out,
        xy_size=xy_size,
        z_size=z_size,
        wavelengths=wavelengths,
        zxy0=zxy0,
    )
    # deal with extended header meta data
    num_integers = len(int_metadata_buffers)
    num_floats = len(float_metadata_buffers)
    header.NumIntegers = num_integers
    header.NumFloats = num_floats
    header.LensNum = lens_id
    extended_bytes = 4 * (num_integers + num_floats)
    header.next = extended_bytes

    handle = open(filename, "wb")
    write_mrc_header(header, handle)

    metadata_offset = 1024
    data_offset = 1024 + extended_bytes
    try:
        handle.seek(metadata_offset)
        handle.write(int_metadata_buffers)
        handle.write(float_metadata_buffers)
        handle.seek(data_offset)  # Seek to end of header
        data_out.tofile(handle)
    except Exception as e:
        print("Error writing image:", e)
        raise e
    handle.close()


def get_extended_header(data, header):
    """Given a buffer of memory that contains the extended header, and the
    standard header, return the extended header as two arrays: one of the ints,
    the other of the floats."""
    num_wavelengths = header.NumWaves
    # \todo Assuming the 'Num' array is in XYZ order.
    images_per_wavelength = header.Num[2] // num_wavelengths
    num_ints = header.NumIntegers
    num_floats = header.NumFloats
    # Start with blank bytes; we'll type-convert later.
    int_array = numpy.zeros(
        int(images_per_wavelength * num_wavelengths * num_ints * 4),
        dtype=numpy.uint8,
    )
    float_array = numpy.zeros(
        int(images_per_wavelength * num_wavelengths * num_floats * 4),
        dtype=numpy.uint8,
    )
    # Amount of memory per image allocated to ints.
    int_size = int(num_ints * 4)
    # Ditto, for floats.
    float_size = int(num_floats * 4)
    chunk_size = int_size + float_size
    # Load the extended header as a bytesequence.
    for i in range(images_per_wavelength * num_wavelengths):
        offset = int(i * chunk_size)
        int_array[i * int_size: (i + 1) * int_size] = data[
            offset : offset + int_size
        ]
        float_array[i * float_size: (i + 1) * float_size] = data[
            offset + int_size: offset + chunk_size
        ]
    # Cast to appropriate datatypes.
    int_array.dtype = numpy.int32
    float_array.dtype = numpy.float32

    # Set the array dimensions as if the arrays were of image data.
    # Use the X axis to store the ints/floats (the Y axis is unused).
    order_str = mrc.axis_order_str(header)
    shape = []
    num_timepoints = header.NumTimes
    num_z = header.Num[2] // (num_wavelengths * num_timepoints)
    key_to_size = {"w": num_wavelengths, "t": num_timepoints, "z": num_z, "y": 1}
    for key in order_str:
        if key in key_to_size:
            shape.append(key_to_size[key])
    int_array.shape = tuple(shape + [num_ints])
    float_array.shape = tuple(shape + [num_floats])
    # Reorder the arrays to WTZYX order.
    if num_ints:
        int_array = reorder_array(
            int_array,
            (num_wavelengths, num_timepoints, num_z, 1, num_ints),
            order_str,
        )
    if num_floats:
        float_array = reorder_array(
            float_array,
            (num_wavelengths, num_timepoints, num_z, 1, num_floats),
            order_str,
        )
    return int_array, float_array


def load_header(path):
    """Load just the header and extended header for the specified filepath."""
    data = numpy.memmap(path)
    header_buffer = data[:1024]
    header = mrc.make_hdr_array(header_buffer)
    num_bytes = header.next
    extended_data = data[1024: 1024 + num_bytes]
    ints, floats = get_extended_header(extended_data, header)
    return header, header_buffer, ints, floats


def reorder_array(data, size, sequence):
    """Given an input array, an expected final array shape, and a sequence
    string (e.g. as generated by mrc.axis_order_str()), reorder the array to be
    in WTZYX order, with any neccesary padding of the array elements.

    How we do this depends on the ordering of X/Y/Z/time/wavelength in the file
    -- the problem being that the shape of the array in the file is not padded
    out with dimensions that are length 1 (e.g. a file with 1 wavelength). So
    we pad out the array until it is five-dimensional, and then rearrange its
    axes until its ordering is WTZYX.
    """
    dim_order = ["w", "t", "z", "y", "x"]
    vals = list(zip(size, dim_order))
    data_copy = numpy.array(data)
    # Find missing axes and pad the array until it has 5 axes.
    for val, key in vals[:-2]:
        # The W/T/Z dimensions are left off if they have
        # length 1.
        if val == 1 and len(data_copy.shape) < 5:
            # The array is missing a dimension, so pad it out.
            data_copy = numpy.expand_dims(data_copy, -1)
            if key in sequence:
                # Remove the existing position for that key and add it to the
                # end, since its existing position is actually wrong.
                index = sequence.index(key)
                sequence = sequence[:index] + sequence[index + 1 :]
            sequence = sequence + key
    # Generate a list of how we need to reorder the axes.
    ordering = []
    for val, key in vals:
        ordering.append(sequence.index(key))

    return data_copy.transpose(ordering)
