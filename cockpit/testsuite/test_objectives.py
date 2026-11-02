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
from unittest import mock

import wx

import cockpit.events
from cockpit.gui import main_panels
from cockpit.handlers.objective import ObjectiveHandler
from cockpit.interfaces import Objectives
from cockpit.testsuite.test_gui import WxTestCase


def make_handler(name, pixel_size, offset, colour, lens_id):
    return ObjectiveHandler(
        name,
        "objectives",
        pixel_size,
        (0, 0, 0),
        offset,
        colour,
        lens_id,
    )


class TestObjectivesApi(WxTestCase):
    # Needs a wx.App because change_objective queues a wx event.
    def setUp(self):
        super().setUp()
        self.handler_20x = make_handler(
            "20x", 0.5, (1, 2, 3), (1.0, 0.0, 0.0), 20
        )
        self.handler_10x = make_handler(
            "10x", 1.0, (4, 5, 6), (0.0, 1.0, 0.0), 10
        )
        self.objectives = Objectives([self.handler_20x, self.handler_10x])

    def test_empty_handlers_raises(self):
        with self.assertRaises(ValueError):
            Objectives([])

    def test_get_handlers_returns_all_in_order(self):
        self.assertEqual(
            self.objectives.get_handlers(),
            [self.handler_20x, self.handler_10x],
        )

    def test_get_current_defaults_to_first(self):
        self.assertIs(self.objectives.get_current(), self.handler_20x)

    def test_get_names(self):
        self.assertEqual(self.objectives.get_names(), ["20x", "10x"])

    def test_get_names_sorted_by_magnification(self):
        # Single-digit magnifications: parse_magnification only reads the
        # last digit of a multi-digit prefix (pre-existing FIXME), so
        # keep this test to names it parses correctly.
        objectives = Objectives(
            [
                make_handler("5x", 1.0, (0, 0, 0), (0, 0, 0), 5),
                make_handler("2x", 1.0, (0, 0, 0), (0, 0, 0), 2),
            ]
        )
        self.assertEqual(objectives.get_names_sorted(), ["2x", "5x"])

    def test_current_properties_follow_current_handler(self):
        self.assertEqual(self.objectives.get_name(), "20x")
        self.assertEqual(self.objectives.get_pixel_size(), 0.5)
        self.assertEqual(self.objectives.get_offset(), (1, 2, 3))
        self.assertEqual(self.objectives.get_colour(), (1.0, 0.0, 0.0))

    def test_change_objective_updates_current(self):
        with mock.patch.object(cockpit.events, "publish") as publish:
            self.objectives.change_objective("10x")
        publish.assert_called_once_with(
            cockpit.events.OBJECTIVE_CHANGE, self.handler_10x
        )
        self.assertIs(self.objectives.get_current(), self.handler_10x)
        self.assertEqual(self.objectives.get_name(), "10x")
        self.assertEqual(self.objectives.get_pixel_size(), 1.0)
        self.assertEqual(self.objectives.get_offset(), (4, 5, 6))
        self.assertEqual(self.objectives.get_colour(), (0.0, 1.0, 0.0))

    def test_change_objective_unknown_name_raises(self):
        with self.assertRaises(ValueError):
            self.objectives.change_objective("40x")
        self.assertIs(self.objectives.get_current(), self.handler_20x)


class TestObjectiveControls(WxTestCase):
    def setUp(self):
        super().setUp()
        self.handler_20x = make_handler(
            "20x", 0.5, (1, 2, 3), (1.0, 0.0, 0.0), 20
        )
        self.handler_10x = make_handler(
            "10x", 1.0, (4, 5, 6), (0.0, 1.0, 0.0), 10
        )
        self.objectives = Objectives([self.handler_20x, self.handler_10x])
        patcher = mock.patch.object(cockpit.events, "publish")
        self.publish = patcher.start()
        self.addCleanup(patcher.stop)
        self.panel = main_panels.ObjectiveControls(
            self.frame, self.objectives
        )

    def test_initial_choice(self):
        choice = self.panel._choice
        self.assertEqual(choice.GetStrings(), self.objectives.get_names_sorted())
        self.assertEqual(choice.GetStringSelection(), "20x")

    def test_choice_event_changes_objective(self):
        choice = self.panel._choice
        choice.SetStringSelection("10x")
        event = wx.CommandEvent(wx.EVT_CHOICE.typeId, choice.GetId())
        event.SetEventObject(choice)
        event.SetString("10x")
        choice.GetEventHandler().ProcessEvent(event)
        self.assertIs(self.objectives.get_current(), self.handler_10x)
        self.publish.assert_called_once_with(
            cockpit.events.OBJECTIVE_CHANGE, self.handler_10x
        )

    def test_objective_change_updates_choice(self):
        self.objectives.change_objective("10x")
        wx.GetApp().ProcessPendingEvents()
        self.assertEqual(self.panel._choice.GetStringSelection(), "10x")


if __name__ == "__main__":
    unittest.main()
