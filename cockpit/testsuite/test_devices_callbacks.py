"""Callback-mapping regression tests for aurox, boulder_slm and dummy_analog.

The handler protocol keys (e.g. "set_enabled", "getMovementTime") are
unchanged; these tests check that they map to the PEP 8 device methods.
"""

import unittest
from unittest import mock

from cockpit.devices import aurox
from cockpit.devices import boulder_slm
from cockpit.devices import dummy_analog


class TestClaritySetEnabled(unittest.TestCase):
    def setUp(self):
        self.dev = aurox.Clarity.__new__(aurox.Clarity)
        self.dev.name = "clarity"
        self.dev._proxy = mock.MagicMock()
        self.dev.handlers = []

    def test_set_enabled_calls_proxy_enable(self):
        self.dev.set_enabled(True)
        self.dev._proxy.enable.assert_called_once_with()
        self.dev._proxy.disable.assert_not_called()

    def test_set_enabled_false_calls_proxy_disable(self):
        self.dev.set_enabled(False)
        self.dev._proxy.disable.assert_called_once_with()
        self.dev._proxy.enable.assert_not_called()

    def test_set_enabled_key_maps_to_set_enabled_method(self):
        with mock.patch.object(aurox.microscope_device.MicroscopeFilter,
                               "get_handlers", return_value=[]), \
                mock.patch.object(aurox, "ClaritySlideHandler") as handler_cls:
            self.dev.get_handlers()
        callbacks = handler_cls.call_args.args[3]
        self.assertEqual(callbacks["set_enabled"], self.dev.set_enabled)
        self.assertTrue(callable(callbacks["set_enabled"]))


class TestBoulderSLMCallbacks(unittest.TestCase):
    def setUp(self):
        self.dev = boulder_slm.BoulderSLM.__new__(boulder_slm.BoulderSLM)
        self.dev.name = "slm"
        self.dev.config = {}
        self.dev.connection = mock.MagicMock()
        self.dev.was_powered = False

    def test_set_enabled_key_maps_to_set_enabled_method(self):
        with mock.patch(
            "cockpit.handlers.executor.DelegateTrigger"
        ) as delegate_cls:
            self.dev.get_handlers()
        callbacks = delegate_cls.call_args.args[3]
        self.assertEqual(callbacks["set_enabled"], self.dev.set_enabled)
        self.assertEqual(callbacks["get_is_enabled"], self.dev.get_is_enabled)
        self.assertEqual(callbacks["execute_table"], self.dev.execute_table)
        self.assertEqual(callbacks["examine_actions"], self.dev.examine_actions)

    def test_cleanup_after_experiment_disables_when_not_powered_before(self):
        self.dev.was_powered = False
        with mock.patch.object(self.dev, "set_enabled") as set_enabled:
            self.dev.cleanup_after_experiment()
        set_enabled.assert_called_once_with(False)

    def test_cleanup_after_experiment_keeps_power_if_it_was_on(self):
        self.dev.was_powered = True
        with mock.patch.object(self.dev, "set_enabled") as set_enabled:
            self.dev.cleanup_after_experiment()
        set_enabled.assert_not_called()

    def test_cleanup_subscription_uses_renamed_method(self):
        with mock.patch.object(boulder_slm.events, "subscribe") as subscribe:
            self.dev.perform_subscriptions()
        subscribed = {c.args[0]: c.args[1] for c in subscribe.call_args_list}
        self.assertEqual(
            subscribed[boulder_slm.events.CLEANUP_AFTER_EXPERIMENT],
            self.dev.cleanup_after_experiment,
        )
        self.assertEqual(
            subscribed[boulder_slm.events.PREPARE_FOR_EXPERIMENT],
            self.dev.on_prepare_for_experiment,
        )


class TestDummyAnalogMovementTime(unittest.TestCase):
    def test_get_movement_time_returns_distance_scaled_time(self):
        dev = dummy_analog.DummyAnalogDevice.__new__(
            dummy_analog.DummyAnalogDevice
        )
        self.assertEqual(dev.get_movement_time(0, 1000), (1000 * 1e-6, 1e-6))
        self.assertEqual(dev.get_movement_time(5, 5), (0.0, 1e-6))

    def test_register_analog_receives_renamed_movement_time(self):
        dev = dummy_analog.DummyAnalogDevice.__new__(
            dummy_analog.DummyAnalogDevice
        )
        dev.name = "dummy"
        dev.config = {"analog_source": "exe", "analog_line": 2, "gain": 1}
        executor = mock.MagicMock()
        with mock.patch.object(dummy_analog.depot, "get_handler",
                               return_value=executor):
            dev.get_handlers()
        args = executor.register_analog.call_args.args
        self.assertEqual(args[4], dev.get_movement_time)


if __name__ == "__main__":
    unittest.main()
