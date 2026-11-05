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


import wx

import cockpit.gui.gui_utils
import cockpit.interfaces.stage_mover


## @package safety_min_dialog.py
# This package contains the SafetyMin_Dialog class and associated constants and
# functions.

## Altitude for slides.
SLIDE_SAFETY = 7300
## Altitude for dishes.
DISH_SAFETY = 5725


class SafetyMinDialog(wx.Dialog):
    """This class provides a simple wrapper around the interfaces.stage_mover's
    safety functionality.

    Note that unlike most dialogs, this one does not save the user's settings;
    instead, it always shows the current safety min as the default setting.
    This is to keep users from blindly setting the safety min to what they
    always use; we want them to think about what they're doing.
    """

    def __init__(
        self,
        parent,
        size=wx.DefaultSize,
        position=wx.DefaultPosition,
        style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER | wx.TAB_TRAVERSAL,
    ):
        super().__init__(parent, -1, "Set Z motion minimum", position, size, style)

        self.main_sizer = wx.BoxSizer(wx.VERTICAL)

        self.main_sizer.Add(
            wx.StaticText(
                self,
                -1,
                "Set the minimum altitude the stage is allowed\n"
                + "to move to.",
            ),
            0,
            wx.ALL | wx.ALIGN_CENTER_HORIZONTAL,
            10,
        )

        self.min_stage_z = cockpit.gui.gui_utils.add_labeled_input(
            parent=self,
            sizer=self.main_sizer,
            label="stage Z minimum (µm):",
            default_value=str(
                cockpit.interfaces.stage_mover.get_soft_limits()[2][0]
            ),
            size=(70, -1),
            min_size=(150, -1),
            should_right_align_input=True,
            border=3,
            control_type=wx.TextCtrl,
        )
        row_sizer = wx.BoxSizer(wx.HORIZONTAL)
        slide_safety_button = wx.Button(self, -1, "Slide")
        slide_safety_button.SetToolTip(
            wx.ToolTip("Set the safety to a good value for slide experiments")
        )
        slide_safety_button.Bind(
            wx.EVT_BUTTON, lambda event: self.set_safety_text(SLIDE_SAFETY)
        )
        row_sizer.Add(slide_safety_button, 0, wx.ALL, 5)
        dish_safety_button = wx.Button(self, -1, "Dish")
        dish_safety_button.SetToolTip(
            wx.ToolTip("Set the safety to a good value for dish experiments")
        )
        dish_safety_button.Bind(
            wx.EVT_BUTTON, lambda event: self.set_safety_text(DISH_SAFETY)
        )
        row_sizer.Add(dish_safety_button, 0, wx.ALL, 5)

        self.main_sizer.Add(row_sizer, 0, wx.ALL | wx.ALIGN_CENTER_HORIZONTAL, 3)

        buttons_box = wx.BoxSizer(wx.HORIZONTAL)

        cancel_button = wx.Button(self, label="Cancel")
        cancel_button.SetToolTip(wx.ToolTip("Close this window"))
        buttons_box.Add(cancel_button, 0, wx.ALL, 5)

        start_button = wx.Button(self, label="Apply")
        start_button.SetToolTip(wx.ToolTip("Apply the chosen safety min"))
        buttons_box.Add(start_button, 0, wx.ALL, 5)

        self.main_sizer.Add(
            buttons_box, 0, wx.ALL | wx.ALIGN_CENTER_HORIZONTAL, 3
        )

        self.SetSizer(self.main_sizer)
        self.SetAutoLayout(True)
        self.main_sizer.Fit(self)

        start_button.Bind(wx.EVT_BUTTON, self.on_start)

    def set_safety_text(self, value):
        """Set the text for the stage safety min to a default value."""
        self.min_stage_z.SetValue("%.1f" % value)

    def on_start(self, event):
        """Save the user's selected Z min to the user config, and then set the
        new min."""
        self.Hide()
        cockpit.interfaces.stage_mover.set_soft_min(
            2, float(self.min_stage_z.GetValue())
        )


## Global dialog singleton.
dialog = None


def show_dialog(parent):
    """Generate the dialog for display.

    If it already exists, just bring it forwards.
    """
    global dialog
    if dialog:
        try:
            dialog.Show()
            dialog.SetFocus()
            return
        except:
            # dialog got destroyed, so just remake it.
            pass
    dialog = SafetyMinDialog(parent)
    dialog.Show()
