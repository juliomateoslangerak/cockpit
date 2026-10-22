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

import logging
import sys
import threading

import wx
import wx.aui


_logger = logging.getLogger(__name__)


class LoggingWindow(wx.Frame):
    """This class provides a window that displays two text panels which capture
    output (stdout and stderr) from the rest of the program.

    This simplifies debugging in many ways.
    """

    SHOW_DEFAULT = False
    LIST_AS_COCKPIT_WINDOW = True

    def __init__(self, parent, title="Logging panels"):
        super().__init__(parent, title=title)

        self.aui_manager = wx.aui.AuiManager()
        self.aui_manager.SetManagedWindow(self)

        ## Text control that captures standard output.
        self.std_out = wx.TextCtrl(
            self, 6465, style=wx.TE_MULTILINE | wx.BORDER_SUNKEN
        )
        ## Text control that captures standard error.
        self.std_err = wx.TextCtrl(
            self, 6265, style=wx.TE_MULTILINE | wx.BORDER_SUNKEN
        )
        ## Cached text, awaiting addition to the logs. If we just log everything
        # as it comes in, then we get tons of newlines we don't want.
        self.text_cache = ""
        # Need a lock on the cache to prevent segfaults due to concurrent access.
        self.cache_lock = threading.Lock()

        # HACK: enforce that writing to these controls only happens in the
        # main thread.
        self.std_out.write = lambda *args: self.write(self.std_out, *args)
        self.std_err.write = lambda *args: self.write(self.std_err, *args)

        sys.stdout = self.std_out
        sys.stderr = self.std_err

        self.aui_manager.AddPane(
            self.std_err,
            wx.aui.AuiPaneInfo()
            .Caption("Standard error")
            .CloseButton(False)
            .Top()
            .MinSize((-1, 194)),
        )
        self.aui_manager.AddPane(
            self.std_out,
            wx.aui.AuiPaneInfo()
            .Caption("Standard out")
            .CloseButton(False)
            .Center()
            .MinSize((-1, 194)),
        )
        self.std_out.write(
            "Device configuration read from: %s\n"
            % wx.GetApp().config.depot_config.files
        )
        self.aui_manager.Update()

        self.Bind(wx.EVT_WINDOW_DESTROY, self.on_destroy)
        self.SetSize((600, 460))

    def write(self, target, *args):
        """Send text to one of our output boxes, and also log that text."""
        wx.CallAfter(target.AppendText, *args)
        # Text output reveals logging window.
        self.Show()

        with self.cache_lock:
            self.text_cache += " ".join(map(str, args))
            if "\n" in self.text_cache:
                # Ended a line; send the text to the logs, minus any trailing
                # whitespace (since the logs add their own trailing newline.
                # We strip any unicode with filter to prevent a cascade of
                # ---Logging Error--- messages.
                if target is self.std_out:
                    _logger.debug(
                        "".join(filter(lambda c: ord(c) < 128, self.text_cache))
                    )
                else:
                    _logger.error(
                        "".join(filter(lambda c: ord(c) < 128, self.text_cache))
                    )
                self.text_cache = ""

    def on_destroy(self, event: wx.WindowDestroyEvent) -> None:
        self.aui_manager.UnInit()
        sys.stdout = sys.__stdout__
        sys.stderr = sys.__stderr__
        event.Skip()


def make_window(parent):
    return LoggingWindow(parent)
