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


class GetNumberDialog(wx.Dialog):
    """This class allows for prompting the user for a number, similar to
    wx.GetNumberFromUser except that we allow for floating point values as
    well."""

    def __init__(self, parent, title, prompt, default, at_mouse=False):
        # Nothing checks how the window was closed, so the OK button should
        # be the only way to close it.
        style = wx.CAPTION
        if at_mouse:
            mouse_pos = wx.GetMousePosition()
            super().__init__(parent, -1, title, mouse_pos, style=style)
        else:
            super().__init__(parent, -1, title, style=style)

        main_sizer = wx.BoxSizer(wx.VERTICAL)

        self.value = cockpit.gui.gui_utils.add_labeled_input(
            parent=self,
            sizer=main_sizer,
            label=prompt,
            default_value=str(default),
            size=(70, -1),
            min_size=(150, -1),
            should_right_align_input=True,
            border=3,
            control_type=wx.TextCtrl,
        )

        buttons_box = wx.BoxSizer(wx.HORIZONTAL)

        # cancel_button = wx.Button(self, wx.ID_CANCEL, "Cancel")
        # cancel_button.SetToolTip(wx.ToolTip("Close this window"))
        # buttons_box.Add(cancelButton, 0, wx.ALL, 5)

        start_button = wx.Button(self, wx.ID_OK, "Okay")
        buttons_box.Add(start_button, 0, wx.ALL, 5)

        main_sizer.Add(buttons_box, 0, wx.ALL | wx.ALIGN_CENTER_HORIZONTAL, 3)

        self.SetSizer(main_sizer)
        self.SetAutoLayout(True)
        main_sizer.Fit(self)

    def get_value(self):
        return self.value.GetValue()


class GetManyNumbersDialog(wx.Dialog):
    """As above, but we can accept any number of prompts for multiple numbers."""

    def __init__(self, parent, title, prompts, default_values, at_mouse=False):
        # Nothing checks how the window was closed, so the OK button should
        # be the only way to close it.
        style = wx.CAPTION
        if at_mouse:
            mouse_pos = wx.GetMousePosition()
            super().__init__(parent, -1, title, mouse_pos, style=style)
        else:
            super().__init__(parent, -1, title, style=style)

        main_sizer = wx.BoxSizer(wx.VERTICAL)

        self.controls = []
        for i, prompt in enumerate(prompts):
            control = cockpit.gui.gui_utils.add_labeled_input(
                parent=self,
                sizer=main_sizer,
                label=prompt,
                default_value=str(default_values[i]),
                size=(70, -1),
                min_size=(150, -1),
                should_right_align_input=True,
                border=3,
                control_type=wx.TextCtrl,
            )
            self.controls.append(control)

        buttons_box = wx.BoxSizer(wx.HORIZONTAL)

        # cancel_button = wx.Button(self, wx.ID_CANCEL, "Cancel")
        # cancel_button.SetToolTip(wx.ToolTip("Close this window"))
        # buttons_box.Add(cancel_button, 0, wx.ALL, 5)

        start_button = wx.Button(self, wx.ID_OK, "Okay")
        buttons_box.Add(start_button, 0, wx.ALL, 5)

        main_sizer.Add(buttons_box, 0, wx.ALL | wx.ALIGN_CENTER_HORIZONTAL, 3)

        self.SetSizer(main_sizer)
        self.SetAutoLayout(True)
        main_sizer.Fit(self)

    def get_values(self):
        return [control.GetValue() for control in self.controls]


def get_number_from_user(parent, title, prompt, default, at_mouse=True):
    dialog = GetNumberDialog(parent, title, prompt, default, at_mouse)
    dialog.ShowModal()
    value = dialog.get_value()
    dialog.Destroy()
    return value


def get_many_numbers_from_user(
    parent, title, prompts, default_values, at_mouse=True
):
    dialog = GetManyNumbersDialog(
        parent, title, prompts, default_values, at_mouse
    )
    dialog.ShowModal()
    value = dialog.get_values()
    dialog.Destroy()
    return value
