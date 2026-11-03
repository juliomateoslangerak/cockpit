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

import unittest
import wx

from cockpit.testsuite.test_gui import WxTestCase
from cockpit.gui import gui_utils


class TestAddLabeledInput(WxTestCase):
    def test_add_labeled_input_uses_wx_sizer_api(self):
        sizer = wx.BoxSizer(wx.VERTICAL)

        control = gui_utils.add_labeled_input(
            self.frame, sizer, label="Test", helper_string="Help text"
        )

        self.assertIsInstance(control, wx.TextCtrl)
        self.assertEqual(sizer.GetItemCount(), 1)


if __name__ == "__main__":
    unittest.main()
