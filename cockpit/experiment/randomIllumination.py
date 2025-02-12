#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2025 CNRS Julio Mateos Langerak <
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

"""Runs RIM experiments."""

from cockpit.experiment import actionTable
from cockpit import depot
from cockpit.experiment import experiment
from cockpit.gui import guiUtils
import cockpit.util.Mrc
import cockpit.util.datadoc
import cockpit.util.userConfig

import decimal
import math
import numpy
import os
import tempfile
import shutil
import wx

## Provided so the UI knows what to call this experiment.
EXPERIMENT_NAME = 'Random Illumination'


def postpad_data(data, shape):
    """Return padded data at end ofp each dimension and reshape.

    This is to handle truncated files when it is required to add blank
    values to obtain a specific shape.  The blank values are zero or
    NaN when supported by the datatype.  See cockpit bug #289.
    """
    postpad_length =  shape - numpy.array(data.shape)
    pad_width = list(zip([0] * len(shape), postpad_length))
    ## Let numpy figure out what to convert NaN into for blank values
    return numpy.pad(data, pad_width, mode='constant',
                     constant_values=[numpy.nan])


def reorder_z_dim(data, order_packed, z_lengths, z_order, z_wanted):
    """Reorder the Z dimension of a numpy array.

    To fix the order of the Z dimension, we reshape the numpy array
    into the real 7 dimensions array that it is, tranpose it as
    necessary, and then reshape it back into the fake 5 dimensions.

    Args:
        data - numpy.array
        order_packed - tuple of 1 character
        z_lengths - tuple of 3 elements with the length of each of the
            dimensions packed in z, same order as z_order
        z_order - a 3 element tuple of 1 character, the order of the z
            dimension.
        z_wanted - a 3 element tuple of 1 character, with the wanted
            order of the z dimension.
    """
    assert data.ndim == len(order_packed), \
        "DATA ndims different from lenght of ORDER_PACKED"
    assert sorted(z_order) == ['a', 'p', 'z'], \
        "Z_ORDER does not have only 'a, z, p'"
    assert sorted(z_order) == sorted(z_wanted), \
        "Z_ORDER not same elements as Z_WANTED"

    z_idx = order_packed.index("z")
    order_in = order_packed[0:z_idx] + z_order + order_packed[z_idx+1:]
    order_out = order_packed[0:z_idx] + z_wanted + order_packed[z_idx+1:]

    packed_shape = data.shape
    unpacked_shape = packed_shape[0:z_idx] + z_lengths + packed_shape[z_idx+1:]

    ## If we are dealing with truncated files we may need to add blank
    ## planes into the data.  See cockpit bug #289.
    if numpy.prod(z_lengths) != packed_shape[z_idx]:
        packed_shape = list(packed_shape)
        packed_shape[z_idx] = numpy.prod(z_lengths)
        packed_shape = tuple(packed_shape)
        data = postpad_data(data, packed_shape)

    ## The new order for the array axes
    dim_map = dict(zip(order_in, range(len(order_in))))
    axes_order = [dim_map[i] for i in order_out]

    data = data.reshape(unpacked_shape)
    data = numpy.transpose(data, axes_order)
    data = data.reshape(packed_shape)
    return data


## This class handles RIM experiments.
class RIExperiment(experiment.Experiment):
    # \param numRImages How many random images to perform per plane.
    # \param polarizerHandler The polarizer to use for the experiment.
    # \param slmHandler Optionally, random illumination is handled by an
    #        SLM or similar pattern-generating device.
    def __init__(self, numRImages, patternSize, polarizerHandler=None,
            slmHandler=None,
            *args, **kwargs):
        # Store the collection order in the MRC header.
        metadata = 'Nr of RImages: %s' % numRImages
        #Store the diffraction angle in MRC metadata
        self.slmdev = depot.getDeviceWithName('slm')
        if self.slmdev:
            self.diffangle = self.slmdev.connection.get_sim_diffraction_angle()
            metadata += ': SLM diff_angle %.3f' % self.diffangle
            self.slm_shape = self.slmdev.connection.get_shape()
            # TODO: what other metadata to get from SLM?
        if 'metadata' in kwargs:
            # Augment the existing string.
            kwargs['metadata'] += "; %s" % metadata
        else:
            kwargs['metadata'] = metadata
        super().__init__(*args, **kwargs)
        self.numZSlices = int(math.ceil(self.zHeight / self.sliceHeight))
        self.numRImages = numRImages
        self.pattern_size = patternSize
        if self.zHeight > 1e-6:
            # Non-2D experiment; tack on an extra image to hit the top of
            # the volume.
            self.numZSlices += 1
        self.polarizerHandler = polarizerHandler
        self.slmHandler = slmHandler

    def generate_rim_sequence(self, numRImages, shape, pattern_size: int = 1):
        """Generate a random illumination sequence.

        This function generates a sequence of length numRImages of random phase patterns
        with a numerical aperture of numerical_aperture.

        param numRImages: The number of images to generate.
        param shape: The shape of the images to generate.
        param numerical_aperture: The numerical aperture of the random patterns. That is the
        size of the structuring object in pixels
        """
        gen = numpy.random.Generator(numpy.random.PCG64())

        # TODO: fix this to get proper rescaling using scipy.ndimage.zoom
        #  a.repeat(2,axis=0).repeat(2,axis=1)
        corr_shape = (shape[0] // pattern_size, shape[1] // pattern_size)
        choices = numpy.array([0, 1], dtype=numpy.uint16)

        for i in range(numRImages):
            pattern = gen.choice(choices, size=corr_shape)
            yield pattern.repeat(pattern_size, axis=0).repeat(pattern_size, axis=1)

    def generateActions(self):
        table = actionTable.ActionTable()
        curTime = 0
        prevZ = None
        numZSlices = int(math.ceil(self.zHeight / self.sliceHeight))

        table.addAction(curTime, self.zPositioner, self.zStart)
        curTime += decimal.Decimal('1')

        # Add a first trigger of the SLM to get first new image.
        table.addAction(curTime, self.slmHandler, 0)
        # Wait a few ms for any necessary SLM triggers.
        curTime = self.slmHandler.getMovementTime() * 1000

        for zIndex in range(numZSlices):
            # Move to the next position, then wait for the stage to
            # stabilize.
            zTarget = self.zStart + self.sliceHeight * zIndex
            motionTime, stabilizationTime = 0, 0
            if prevZ is not None:
                motionTime, stabilizationTime = self.zPositioner.getMovementTime(prevZ, zTarget)
                motionTime *= 1000
                stabilizationTime *= 1000
            curTime += motionTime
            table.addAction(curTime, self.zPositioner, zTarget)
            curTime += stabilizationTime
            prevZ = zTarget

            for cameras, lightTimePairs in self.exposureSettings:
                curTime = self.expose(curTime, cameras, lightTimePairs, table)

            for pattern_index in range(self.numRImages):
                table.addAction(curTime, self.slmHandler, pattern_index)
                curTime += self.slmHandler.settlingtime
                # Image the sample.
                for cameras, lightTimePairs in self.exposureSettings:
                    curTime = self.expose(curTime, cameras, lightTimePairs, table)
                    # Advance the time very slightly so that all exposures
                    # are strictly ordered.
                    curTime += decimal.Decimal('1e-10')
                # Hold the Z motion flat during the exposure.
            table.addAction(curTime, self.zPositioner, zTarget)

        motionTime, stabilizationTime = self.zPositioner.getMovementTime(
                self.zHeight, self.zStart)
        motionTime *= 1000
        stabilizationTime *= 1000
        table.addAction(curTime + motionTime, self.zPositioner, self.zStart)
        finalWaitTime = motionTime + stabilizationTime

        # Ramp down Z
        table.addAction(curTime + finalWaitTime, self.zPositioner, self.zStart)

        # Set SLM back to 0th image ready for next measurement in timelapse or multi-site.
        if self.slmHandler is not None:
            # Toggle the slmHandler's digital line handler to advance one frame.
            table.addToggle(curTime, self.slmHandler)
            table.addAction(curTime, self.slmHandler, 0)

        return table

    ## Wrapper around Experiment.expose() that creates the RIM patterns for each exposure
    def expose(self, curTime, cameras, lightTimePairs, table):
        # We use the longest wavelength in a given exposure to determine the RIM pattern.
        wavelengths = []
        longestWavelength = max([ltp[0].wavelength for ltp in lightTimePairs])

        for pattern in self.generate_rim_sequence(self.numRImages, self.slm_shape, self.pattern_size):
            # SLM trigger
            table.addAction(curTime, self.slmHandler, (pattern, longestWavelength))
            curTime += self.slmHandler.getMovementTime()
            return super().expose(curTime, cameras, lightTimePairs, table)

    def cleanup(self, runThread = None, saveThread = None):
        super().cleanup(runThread, saveThread)
        return

    def lastMinuteActions(self):
        if self.sliceHeight != 0.125:
            warning = "Slice height must be 0.125 for softWoRx 3D " \
                      "reconstruction. Choose:" \
                      "\n    'OK' to run as is;" \
                      "\n    'Cancel' to go back and change parameters."
            if not guiUtils.getUserPermission(warning):
                return False
        return True


## A consistent name to use to refer to the class itself.
EXPERIMENT_CLASS = RIExperiment


## Generate the UI for special parameters used by this experiment.
class BaseRIMExperimentUI(wx.Panel):
    """Base Experiment UI for RIM experiments.

    Subclasses must implement class property `_CONFIG_KEY_SUFFIX`.
    """
    def __init__(self, parent, configKey):
        super().__init__(parent=parent)

        self.configKey = configKey + self._CONFIG_KEY_SUFFIX
        self.allLights = depot.getHandlersOfType(depot.LIGHT_TOGGLE)
        self.settings = self.loadSettings()

        sizer = wx.BoxSizer(wx.VERTICAL)

        self.SetSizerAndFit(sizer)


    ## Given a parameters dict (parameter name to value) to hand to the
    # experiment instance, augment them with our special parameters.
    def augmentParams(self, params):
        self.saveSettings()
        params['slmHandler'] = depot.getHandler('slm', depot.EXECUTOR)
        return params

    def _getDefaultSettings(self):
        allLights = depot.getHandlersOfType(depot.LIGHT_TOGGLE)
        default = {
        }
        return default

    ## Load the saved experiment settings, if any.
    def loadSettings(self):
        result = cockpit.util.userConfig.getValue(
                self.configKey,
                default = self._getDefaultSettings()
        )

        allLights = depot.getHandlersOfType(depot.LIGHT_TOGGLE)
        return result

    ## Generate a dict of our settings.
    def getSettingsDict(self):
        return {
        }

    ## Save the current experiment settings to config.
    def saveSettings(self, settings = None):
        if settings is None:
            settings = self.getSettingsDict()
        cockpit.util.userConfig.setValue(self.configKey, settings)


class ExperimentUI(BaseRIMExperimentUI):
    _CONFIG_KEY_SUFFIX = 'RIExperimentSettings'

    def __init__(self, parent, configKey):
        super().__init__(parent, configKey)

        self.Sizer.SetSizeHints(self)

    def getSettingsDict(self):
        all_settings = super().getSettingsDict()
        return all_settings
