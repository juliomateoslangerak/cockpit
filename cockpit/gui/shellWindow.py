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

import wx
import wx.py.shell


class ShellWindow(wx.py.shell.ShellFrame):
    SHOW_DEFAULT = False
    LIST_AS_COCKPIT_WINDOW = True

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # wx.py forces a white background but leaves the foreground to
        # the system default, which is white in dark mode (wx >= 3.3)
        # making the text invisible.  Set a black default foreground
        # and reapply the styles so that it propagates to all of them.
        self.shell.StyleSetForeground(wx.stc.STC_STYLE_DEFAULT, wx.BLACK)
        self.shell.setStyles(wx.py.editwindow.FACES)
        self.shell.SetCaretForeground(wx.BLACK)


def makeWindow(parent):
    window = ShellWindow(parent)
    window.shell.run("import wx")
    window.shell.run("depot = wx.GetApp().Depot")
    # Default icon for the ShellFrame is the PyCrust, so replace it.
    window.SetIcon(parent.GetIcon())
