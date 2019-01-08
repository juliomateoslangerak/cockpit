#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2018 David Miguel Susano Pinto <david.pinto@bioch.ox.ac.uk>
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

import pkg_resources

## The resource_name argument for resource_filename is not a
## filesystem filepath.  It is a /-separated filepath, even on
## windows, so do not use os.path.join.

FONT_PATH = pkg_resources.resource_filename(
    'cockpit',
    'resources/fonts/UniversalisADFStd-Regular.otf'
)

BITMAPS_PATH = pkg_resources.resource_filename(
    'cockpit',
    'resources/bitmaps/'
)
