"""Tests for the device modules renamed to PEP 8 names.

These cover the renamed internals and the config and handler paths that
callers depend on, without touching real hardware.
"""

import unittest
from unittest import mock

from cockpit.devices import aerotech_axis, drawer, light


class TestAerotechLimitsConfig(unittest.TestCase):
    def test_softlimits_are_parsed_from_config(self):
        axis = aerotech_axis.AerotechZStage(
            "aerotech", {"soft_limits": "(-45000, 5000)"}
        )
        self.assertEqual(axis.soft_limits, (-45000, 5000))

    def test_missing_soft_limits_fall_back_to_defaults(self):
        axis = aerotech_axis.AerotechZStage("aerotech", {})
        self.assertEqual(axis.soft_limits, (-30000, 7000))

    def test_get_handlers_uses_soft_limits_for_handler_limits(self):
        axis = aerotech_axis.AerotechZStage(
            "aerotech", {"soft_limits": "(-45000, 5000)"}
        )
        axis.axis = 1
        handlers = axis.get_handlers()
        self.assertEqual(len(handlers), 1)
        # PositionerHandler owns the public soft_limits attribute.
        self.assertEqual(list(handlers[0].soft_limits), [-45000, 5000])
        self.assertEqual(
            set(handlers[0].callbacks),
            {
                "move_absolute",
                "move_relative",
                "get_position",
                "get_movement_time",
            },
        )


class TestAerotechMovementTime(unittest.TestCase):
    def setUp(self):
        self.axis = aerotech_axis.AerotechZStage("aerotech", {})
        self.axis.speed = 1000
        self.axis.acceleration = 1000

    def test_long_move_uses_cruise_time(self):
        dt, settle = self.axis.get_movement_time(1, 0, 1000000)
        # 1000 mm displacement at v=1000, a=1000: cruise dominates.
        self.assertAlmostEqual(
            dt, 1000 * ((1000.0 / 1000) + (1000 / 1000)) + 10
        )
        self.assertEqual(settle, 2)

    def test_short_move_uses_triangular_profile(self):
        dt, _ = self.axis.get_movement_time(1, 0, 100)
        # 0.1 mm: below the braking distance, so no cruise phase.
        self.assertAlmostEqual(dt, 1000 * 2 * (0.1 / 1000) ** 0.5 + 10)


class TestAerotechOpenConnection(unittest.TestCase):
    def test_open_connection_connects_to_ip_and_port(self):
        axis = aerotech_axis.AerotechZStage(
            "aerotech", {"ip_address": "10.0.0.7", "port": "8000"}
        )
        with mock.patch.object(aerotech_axis.socket, "socket") as socket_cls:
            axis.open_connection()
        sock = socket_cls.return_value
        sock.connect.assert_called_once_with(("10.0.0.7", 8000))
        self.assertIs(axis.socket, sock)

    def test_open_connection_is_the_renamed_method(self):
        self.assertTrue(callable(aerotech_axis.AerotechZStage.open_connection))
        self.assertFalse(
            hasattr(aerotech_axis.AerotechZStage, "openConnection")
        )


class TestDrawerParseConfig(unittest.TestCase):
    def setUp(self):
        self.config = {
            "type": "cockpit.devices.drawer.Drawer",
            "cameras": "cam1, cam2",
            "default": "GFP: 525, Cy5: 695",
            "TRITC": "FITC: 518, TRITC: 600",
        }
        self.drawer = drawer.Drawer("drawer", dict(self.config))

    def test_parse_config_builds_settings_for_each_key(self):
        settings = self.drawer.parse_config()
        self.assertEqual(
            sorted(s.name for s in settings), ["TRITC", "default"]
        )

    def test_parse_config_accepts_explicit_config(self):
        settings = self.drawer.parse_config(dict(self.config))
        self.assertEqual(len(settings), 2)

    def test_parse_config_rejects_mismatched_filters(self):
        bad = dict(self.config, default="GFP: 525")
        with self.assertRaises(Exception):
            self.drawer.parse_config(bad)

    def test_get_handlers_uses_parse_config(self):
        handlers = self.drawer.get_handlers()
        self.assertEqual(len(handlers), 1)
        self.assertIs(self.drawer.handler, handlers[0])
        self.assertFalse(hasattr(drawer.Drawer, "parseConfig"))


class TestSimpleLightExposureTime(unittest.TestCase):
    def _make_handler(self):
        dev = light.SimpleLight("led", {"wavelength": "488"})
        handlers = dev.get_handlers()
        return dev, handlers[0]

    def test_exposure_time_defaults_to_100(self):
        _, handler = self._make_handler()
        self.assertEqual(handler.get_exposure_time(), 100)

    def test_set_exposure_time_round_trips_through_callbacks(self):
        dev, handler = self._make_handler()
        handler.callbacks["set_exposure_time"]("led", 42)
        self.assertEqual(dev._exposure_time, 42)
        self.assertEqual(handler.callbacks["get_exposure_time"]("led"), 42)
        self.assertEqual(handler.get_exposure_time(), 42)

    def test_exposure_time_attribute_is_renamed(self):
        dev, _ = self._make_handler()
        self.assertFalse(hasattr(dev, "_exposureTime"))


class TestAmbientLightExposureTime(unittest.TestCase):
    def test_ambient_light_stores_exposure_time(self):
        dev = light.AmbientLight("ambient", {})
        handler = dev.get_handlers()[0]
        self.assertEqual(handler.get_exposure_time(), 100.0)
        handler.callbacks["set_exposure_time"]("Ambient", 7.5)
        self.assertEqual(
            handler.callbacks["get_exposure_time"]("Ambient"), 7.5
        )


if __name__ == "__main__":
    unittest.main()
