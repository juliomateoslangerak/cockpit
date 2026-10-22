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

import cockpit.gui.camera.window
import cockpit.gui.mosaic.window
import cockpit.interfaces.stage_mover


def on_char(evt):
    if isinstance(evt.EventObject, wx.TextCtrl) or evt.KeyCode in [
        wx.WXK_TAB,
        wx.WXK_ESCAPE,
    ]:
        evt.Skip()
        return
    if isinstance(
        evt.EventObject, (wx.ToggleButton, wx.Button)
    ) and evt.KeyCode in [wx.WXK_RETURN, wx.WXK_SPACE, wx.WXK_TAB]:
        evt.Skip()
        return
    if evt.KeyCode == wx.WXK_NUMPAD_MULTIPLY:
        cockpit.gui.camera.window.rescale_views()
    elif evt.KeyCode == wx.WXK_NUMPAD_DIVIDE:
        cockpit.interfaces.stage_mover.change_mover()
    elif evt.KeyCode == wx.WXK_NUMPAD_DECIMAL:
        cockpit.gui.mosaic.window.transferCameraImage()
    elif evt.KeyCode == wx.WXK_NUMPAD_SUBTRACT:
        cockpit.interfaces.stage_mover.recenter_fine_motion()
    elif evt.KeyCode == wx.WXK_DOWN:
        # Z down
        cockpit.interfaces.stage_mover.step((0, 0, -1))
    elif evt.KeyCode == wx.WXK_NUMPAD2:
        # Y down
        cockpit.interfaces.stage_mover.step((0, -1, 0))
    elif evt.KeyCode == wx.WXK_NUMPAD3:
        # Decrease delta
        cockpit.interfaces.stage_mover.change_step_size(-1)
    elif evt.KeyCode == wx.WXK_NUMPAD4:
        # X up
        cockpit.interfaces.stage_mover.step((1, 0, 0))
    elif evt.KeyCode == wx.WXK_NUMPAD5:
        # Stop motion
        # TODO: was never handled.
        pass
    elif evt.KeyCode == wx.WXK_NUMPAD6:
        # X down
        cockpit.interfaces.stage_mover.step((-1, 0, 0))
    elif evt.KeyCode == wx.WXK_UP:
        # Z up
        cockpit.interfaces.stage_mover.step((0, 0, 1))
    elif evt.KeyCode == wx.WXK_NUMPAD8:
        # Y up
        cockpit.interfaces.stage_mover.step((0, 1, 0))
    elif evt.KeyCode == wx.WXK_NUMPAD9:
        # Increase delta
        cockpit.interfaces.stage_mover.change_step_size(1)
    elif evt.KeyCode in [wx.WXK_NUMPAD_ADD, wx.WXK_NUMPAD0]:
        # Take image
        wx.GetApp().imager.take_image()
    else:
        evt.Skip()


def set_keyboard_handlers(window):
    """Given a wx.Window instance, set up keyboard controls for that instance."""
    accel_table = wx.AcceleratorTable(
        [
            # Pop up a menu to help the user find hidden windows.
            (wx.ACCEL_CTRL, ord("M"), 6321),
            # toggle python shell state with ^P
            (wx.ACCEL_CTRL, ord("P"), 6322),
        ]
    )
    window.SetAcceleratorTable(accel_table)
    window.Bind(wx.EVT_MENU, lambda e: show_hide_shell(window), id=6322)
    window.Bind(wx.EVT_CHAR_HOOK, on_char)


def show_hide_shell(parent):
    """Function to show/hide the pythion shell window."""
    for window in wx.GetTopLevelWindows():
        if window.GetTitle() == "PyShell":
            window.Show(not window.IsShown())
            break
