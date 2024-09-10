#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2024 Centre National de la Recherche Scientifique
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

"""Spatial Light Modulators from python-micrsocope device server."""

import logging
import typing

import Pyro4
import numpy as np

from cockpit import depot
from cockpit import events
from cockpit.devices.microscopeDevice import MicroscopeBase, MicroscopeSwitchableDevice, MicroscopeGenericDevice

class MicroscopeSLM(MicroscopeSwitchableDevice):
    """Device class for a remote python-microscope Spatial Light Modulator.

    Sample configuration entry:

    [SLM]
    type: cockpit.devices.microscopeSLM.MicroscopeSLM
    uri: PYRO:SomeSLM@192.168.0.23:7001
    triggersource: trigsource
    triggerline: 2
    settlingtime: 10

    [trigsource]
    type: ExecutorDevice
    """
    def __init__(self, name: str, config: typing.Mapping[str, str]):
        """Initialise the SLM device."""
        super().__init__(name, config)
        self.name = name
    #     self.uri = config.get('uri')
    #     self.triggerSource = config.get('triggerSource')
    #     self.triggerLine = config.get('triggerLine')
    #     self.slm = Pyro4.Proxy(self.uri)
    #     self.slm.set_trigger_source(self.triggerSource)
    #     self.slm.set_trigger_line(self.triggerLine)
    #
    #     self.depot = depot.Depot()
    #     self.depot.register('slm', self.slm)
    #
    def getHandlers(self):
        """Return camera handlers."""
        trigsource = self.config.get('triggersource', None)
        trigline = self.config.get('triggerline', None)
        if trigsource:
            trighandler = depot.getHandler(trigsource, depot.EXECUTOR)
        else:
            trighandler = None

