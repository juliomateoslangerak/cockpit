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

import numpy as np
import wx

import cockpit.gui.gui_utils
import cockpit.interfaces.stage_mover
import cockpit.util.user_config


class GridSitesDialog(wx.Dialog):
    """This class shows a simple dialog to the user that allows them to lay
    down a grid of sites on the mosaic.

    They can then use this to image large areas in a regulated manner without
    relying on the mosaic's spiral system.
    """

    def __init__(self, parent):
        """Create the dialog, and lay out its UI widgets."""
        super().__init__(parent, -1, "Place a Grid of Sites")

        ## config-loaded settings for the form.
        self.settings = cockpit.util.user_config.get_value(
            "gridSitesDialog",
            default={
                "num_rows": "10",
                "num_columns": "10",
                "image_width": "512",
                "image_height": "512",
                "marker_size": "25",
            },
        )

        sizer = wx.BoxSizer(wx.VERTICAL)

        label = wx.StaticText(
            self,
            -1,
            "The upper-left corner of the grid will be at the current "
            + "stage position.",
        )
        sizer.Add(label, 0, wx.ALIGN_CENTRE | wx.ALL, 5)

        self.num_rows = cockpit.gui.gui_utils.add_labeled_input(
            self,
            sizer,
            label="Number of rows:",
            default_value=self.settings["num_rows"],
        )
        self.num_columns = cockpit.gui.gui_utils.add_labeled_input(
            self,
            sizer,
            label="Number of columns:",
            default_value=self.settings["num_columns"],
        )
        self.image_width = cockpit.gui.gui_utils.add_labeled_input(
            self,
            sizer,
            label="Horizontal spacing (pixels):",
            default_value=self.settings["image_width"],
        )
        self.image_height = cockpit.gui.gui_utils.add_labeled_input(
            self,
            sizer,
            label="Vertical spacing (pixels):",
            default_value=self.settings["image_height"],
        )
        self.marker_size = cockpit.gui.gui_utils.add_labeled_input(
            self,
            sizer,
            label="Marker size (default 25):",
            default_value=self.settings["marker_size"],
        )

        button_box = wx.BoxSizer(wx.HORIZONTAL)

        cancel_button = wx.Button(self, wx.ID_CANCEL, "Cancel")
        cancel_button.SetToolTip("Close this window")
        button_box.Add(cancel_button, 0, wx.ALIGN_CENTRE | wx.ALL, 5)

        start_button = wx.Button(self, wx.ID_OK, "Mark sites")
        button_box.Add(start_button, 0, wx.ALIGN_CENTRE | wx.ALL, 5)

        button_box.Add((20, -1), 1, wx.ALL, 5)
        sizer.Add(button_box, 0, wx.ALIGN_RIGHT | wx.ALL, 5)

        self.SetSizer(sizer)
        self.SetAutoLayout(True)
        sizer.Fit(self)

        self.Bind(wx.EVT_BUTTON, self.on_start, start_button)

    def on_start(self, evt):
        """Create the grid of sites."""
        self.save_settings()

        cur_location = cockpit.interfaces.stage_mover.get_position()
        image_width = float(self.image_width.GetValue())
        image_height = float(self.image_height.GetValue())
        marker_size = float(self.marker_size.GetValue())
        pixel_size = wx.GetApp().objectives.get_pixel_size()

        for x_offset in range(int(self.num_columns.GetValue())):
            x_location = cur_location[0] - x_offset * pixel_size * image_width
            for y_offset in range(int(self.num_rows.GetValue())):
                y_location = cur_location[1] - y_offset * pixel_size * image_height
                target = np.array([x_location, y_location, cur_location[2]])
                new_site = cockpit.interfaces.stage_mover.Site(
                    target, size=marker_size
                )
                cockpit.interfaces.stage_mover.save_site(new_site)
        self.Destroy()

    def save_settings(self):
        """Save the user's settings to the configuration file."""
        cockpit.util.user_config.set_value(
            "gridSitesDialog",
            {
                "num_rows": self.num_rows.GetValue(),
                "num_columns": self.num_columns.GetValue(),
                "image_width": self.image_width.GetValue(),
                "image_height": self.image_height.GetValue(),
                "marker_size": self.marker_size.GetValue(),
            },
        )


def show_dialog(parent):
    """Show the dialog."""
    dialog = GridSitesDialog(parent)
    dialog.Show()
    dialog.SetFocus()
    return dialog
