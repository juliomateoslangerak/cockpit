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
from unittest.mock import Mock, patch

import numpy

from cockpit.interfaces import stage_mover


class TestStepSizeSense(unittest.TestCase):
    def assertNextStepSize(self, step: float, expected: float) -> None:
        self.assertEqual(
            stage_mover.sensible_next_step_size(step),
            expected,
            "expected step size after %f to be %f" % (step, expected),
        )

    def assertPreviousStepSize(self, step: float, expected: float) -> None:
        self.assertEqual(
            stage_mover.sensible_previous_step_size(step),
            expected,
            "expected step size before %f to be %f" % (step, expected),
        )

    def test(self):
        for step, up, down in [
            (0.00001, 0.00002, 0.000005),
            (0.00004, 0.00005, 0.00002),
            (0.00009, 0.0001, 0.00005),
            (0.9, 1, 0.5),
            (0.5, 1, 0.2),
            (0.3, 0.5, 0.2),
            (0.2, 0.5, 0.1),
            (0.1, 0.2, 0.05),
            (1, 2, 0.5),
            (2, 5, 1),
            (3, 5, 2),
            (5, 10, 2),
            (9, 10, 5),
            (10, 20, 5),
            (15, 20, 10),
            (20, 50, 10),
            (30, 50, 20),
            (50, 100, 20),
            (60, 100, 50),
            (99, 100, 50),
            (100, 200, 50),
            (130, 200, 100),
            (150, 200, 100),
            (2000.1, 5000, 2000),
        ]:
            self.assertNextStepSize(step, up)
            self.assertPreviousStepSize(step, down)


class TestStageMoverHandlerProtocol(unittest.TestCase):
    def setUp(self):
        saved_site_index = stage_mover.unique_site_index
        self.addCleanup(
            setattr, stage_mover, "unique_site_index", saved_site_index
        )

        self.handlers = []
        for axis in range(3):
            handler = Mock(spec=["get_position", "move_absolute", "set_soft_limit"])
            handler.name = "axis_%d" % axis
            handler.getPosition.return_value = axis + 1
            self.handlers.append(handler)

        self.mover = object.__new__(stage_mover.StageMover)
        self.mover.axis_to_handlers = {
            axis: [handler] for axis, handler in enumerate(self.handlers)
        }
        self.mover.cur_handler_index = 0
        self.mover.name_to_stopped_event = {}
        self.mover.id_to_site = {}
        mover_patch = patch.object(stage_mover, "mover", self.mover)
        mover_patch.start()
        self.addCleanup(mover_patch.stop)

    def test_get_position_uses_handler_protocol(self):
        self.assertEqual(stage_mover.get_position(), [1, 2, 3])
        for handler in self.handlers:
            handler.get_position.assert_called_once_with()

    def test_go_to_uses_handler_protocol(self):
        stage_mover.go_to([4, 5, 6])

        for axis, handler in enumerate(self.handlers):
            handler.get_position.assert_called_once_with()
            handler.move_absolute.assert_called_once_with(axis + 4)

    def test_move_relative_uses_handler_protocol(self):
        stage_mover.move_relative([4, 5, 6])

        for axis, handler in enumerate(self.handlers):
            handler.move_absolute.assert_called_once_with(2 * axis + 5)

    def test_save_site_uses_handler_protocol(self):
        with patch.object(stage_mover.events, "publish"):
            stage_mover.save_site()

        site = next(iter(self.mover.id_to_site.values()))
        self.assertEqual(site.position, [1, 2, 3])
        for handler in self.handlers:
            handler.get_position.assert_called_once_with()

    def test_set_soft_limit_uses_handler_protocol(self):
        handler = self.handlers[0]
        with patch.object(stage_mover.events, "publish") as publish:
            self.assertTrue(stage_mover.set_soft_limit(0, 10.0, True))
            handler.set_soft_limit.assert_called_once_with(10.0, True)
            publish.assert_any_call(stage_mover.events.SOFT_SAFETY_LIMIT, 0, 10.0, True)

            stage_mover.set_soft_min(0, 0.0)
            handler.set_soft_limit.assert_called_with(0.0, False)
            publish.assert_any_call(stage_mover.events.SOFT_SAFETY_LIMIT, 0, 0.0, False)

            stage_mover.set_soft_max(0, 20.0)
            handler.set_soft_limit.assert_called_with(20.0, True)
            publish.assert_any_call(stage_mover.events.SOFT_SAFETY_LIMIT, 0, 20.0, True)

    def test_set_soft_limit_returns_false_when_handler_fails(self):
        handler = self.handlers[0]
        handler.set_soft_limit.side_effect = RuntimeError("device error")
        with patch.object(stage_mover.events, "publish") as publish:
            self.assertFalse(stage_mover.set_soft_limit(0, 10.0, True))
            handler.set_soft_limit.assert_called_once_with(10.0, True)
            publish.assert_not_called()

    def test_site_serialization_round_trip(self):
        site = stage_mover.Site(
            numpy.array([1.0, 2.5, -3.0]),
            group="test",
            color=(1, 2, 3),
            size=4.5,
        )

        restored = stage_mover.deserialize_site(site.serialize())

        self.assertEqual(list(restored.position), [1.0, 2.5, -3.0])
        self.assertEqual(restored.group, "test")
        self.assertEqual(restored.color, (1.0, 2.0, 3.0))
        self.assertEqual(restored.size, 4.5)
        self.assertEqual(restored.unique_id, site.unique_id)


if __name__ == "__main__":
    unittest.main()
