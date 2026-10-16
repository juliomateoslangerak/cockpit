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

import contextlib
import unittest
import unittest.mock

import cockpit.depot
import cockpit.events
import cockpit.handlers.camera


def mock_events_at(where: str):
    # This function mocks the entire events module at a specific place
    # but it does while keeping the original event constants so that
    # it can be used to check if the events were subscribed to,
    # otherwise they would subscribe to mock constants.  No need to
    # fix all events, name only those that are needed.
    mocked_events = unittest.mock.MagicMock()
    for event_name in [
        "PREPARE_FOR_EXPERIMENT",
    ]:
        setattr(mocked_events, event_name, getattr(cockpit.events, event_name))

    @unittest.mock.patch(where + ".events", new=mocked_events)
    def with_mocked_events(func):
        return lambda func: func(mocked_events)

    return with_mocked_events


@contextlib.contextmanager
def mock_not_in_video_mode():
    """Mock CockpitApp that reports we are not in video mode.

    The camera handler uses `cockpit.interfaces.imager.pause_video`
    which looks into wx's app singleton.  This ruins the abstraction,
    the handlers should not require a wx app running.  While that is
    not fixed, this mocks access to CockpitApp.Imager so it reports we
    are not in video mode and pause_video then does nothing.

    """

    class MockImager:
        def __init__(self):
            self.am_in_video_mode = False

    class MockCockpitApp:
        @property
        def Imager(self):
            return MockImager()

    with unittest.mock.patch(
        "cockpit.interfaces.imager.wx.GetApp", new=MockCockpitApp
    ):
        yield


class CameraHandlerTestCase(unittest.TestCase):
    def setUp(self):
        """On each test we will be creating new handlers which may conflict
        with handlers created in previous tests.

        So just build a new depot and reinitialise the imager.
        """
        cockpit.depot.deviceDepot = cockpit.depot.DeviceDepot()

        self.callback_mocks = {
            "set_enabled": unittest.mock.Mock(side_effect=lambda n, s: s),
        }

        self.args = {
            "name": "mock",
            "group_name": "testsuite",
            "callbacks": self.callback_mocks,
            "exposure_mode": cockpit.handlers.camera.TRIGGER_BEFORE,
        }

    def test_update_filter(self):
        event_handler = unittest.mock.Mock()
        cockpit.events.subscribe(cockpit.events.FILTER_CHANGE, event_handler)

        try:
            camera = cockpit.handlers.camera.CameraHandler(**self.args)
            camera.update_filter("test-dye", 512.0)
        finally:
            cockpit.events.unsubscribe(
                cockpit.events.FILTER_CHANGE, event_handler
            )

        event_handler.assert_called_once()
        self.assertEqual(camera.dye, "test-dye")
        self.assertEqual(camera.wavelength, 512.0)

    def test_descriptive_name(self):
        camera = cockpit.handlers.camera.CameraHandler(**self.args)
        camera.update_filter("Test-Dye", 512.0)
        self.assertEqual(
            camera.descriptive_name, self.args["name"] + " (Test-Dye)"
        )

    def test_camera_enable_event(self):
        camera = cockpit.handlers.camera.CameraHandler(**self.args)

        event_handler = unittest.mock.Mock()
        cockpit.events.subscribe(cockpit.events.CAMERA_ENABLE, event_handler)
        try:
            with mock_not_in_video_mode():
                camera.set_enabled(True)
        finally:
            cockpit.events.unsubscribe(
                cockpit.events.CAMERA_ENABLE, event_handler
            )

        event_handler.assert_called_once()
        event_handler.assert_called_with(camera, True)
        self.callback_mocks["set_enabled"].assert_called_with("mock", True)

    @mock_events_at("cockpit.handlers.camera")
    def test_set_enabled_sends_experiment_event(self, mockEvents):
        camera = cockpit.handlers.camera.CameraHandler(**self.args)
        camera.set_enabled(True)

        mockEvents.subscribe.assert_called_with(
            cockpit.events.PREPARE_FOR_EXPERIMENT, camera.prepare_for_experiment
        )

    @mock_events_at("cockpit.handlers.camera")
    def testSetEnabledSendsCorrectEvent_disable(self, mockEvents):
        camera = cockpit.handlers.camera.CameraHandler(**self.args)
        camera.set_enabled(False)
        mockEvents.unsubscribe.assert_called_with(
            cockpit.events.PREPARE_FOR_EXPERIMENT, camera.prepare_for_experiment
        )

    def test_enabled_getter(self):
        camera = cockpit.handlers.camera.CameraHandler(**self.args)
        self.assertFalse(camera.get_is_enabled())
        with mock_not_in_video_mode():
            camera.set_enabled(True)
        self.assertTrue(camera.get_is_enabled())
        with mock_not_in_video_mode():
            camera.set_enabled(False)
        self.assertFalse(camera.get_is_enabled())

    def test_get_time_between_exposures(self):
        callback = unittest.mock.Mock(return_value=50)
        self.args["callbacks"] = {"get_time_between_exposures": callback}
        camera = cockpit.handlers.camera.CameraHandler(**self.args)

        self.assertEqual(camera.get_time_between_exposures(), 50)
        callback.assert_called_with("mock", False)

    def testGetMinExposureTime(self):
        callback = unittest.mock.Mock(return_value=50)
        self.args["callbacks"] = {"get_min_exposure_time": callback}
        camera = cockpit.handlers.camera.CameraHandler(**self.args)

        self.assertEqual(camera.get_min_exposure_time(), 50)
        callback.assert_called_with("mock")

    def testSetExposureTime(self):
        callback = unittest.mock.Mock()
        self.args["callbacks"] = {"set_exposure_time": callback}
        camera = cockpit.handlers.camera.CameraHandler(**self.args)

        camera.set_exposure_time(50)
        callback.assert_called_with("mock", 50)


if __name__ == "__main__":
    unittest.main()
