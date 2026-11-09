"""Tests for the second group of device modules renamed to PEP 8 names.

Hardware connections are mocked.  The tests cover the renamed internals
together with the handler callbacks, subscriptions, config keys and wire
strings which must not change.
"""

import unittest
from unittest import mock

from cockpit import events
from cockpit.devices import (
    light,
    linkam_stage,
    nanomover,
    picomotor,
    polarization_rotator,
    shutter,
    sr470,
    stage,
)


class TestShutterSubscriptions(unittest.TestCase):
    def test_perform_subscriptions_uses_renamed_callbacks(self):
        dev = shutter.ShutterDevice("shutter", {})
        with mock.patch.object(shutter.events, "subscribe") as subscribe:
            dev.perform_subscriptions()
        subscribed = {c.args[0]: c.args[1] for c in subscribe.call_args_list}
        self.assertEqual(
            subscribed[events.EXPERIMENT_COMPLETE],
            dev.on_cleanup_after_experiment,
        )
        self.assertEqual(
            subscribed[events.PREPARE_FOR_EXPERIMENT],
            dev.on_prepare_for_experiment,
        )
        self.assertEqual(
            subscribed[events.LIGHT_SOURCE_ENABLE], dev.on_light_source_enable
        )

    def test_light_source_enable_toggles_trigger(self):
        dev = shutter.ShutterDevice("shutter", {})
        light_handler = mock.Mock()
        light_handler.get_is_enabled.return_value = False
        dev.lights = [light_handler]
        with mock.patch.object(dev, "enable_trigger") as enable_trigger:
            dev.on_light_source_enable(light_handler, True)
            enable_trigger.assert_called_once_with()
            enable_trigger.reset_mock()
            dev.on_light_source_enable(light_handler, False)
            enable_trigger.assert_called_once_with(False)

    def test_old_names_are_gone(self):
        dev = shutter.ShutterDevice("shutter", {})
        self.assertFalse(hasattr(dev, "enableTrigger"))
        self.assertFalse(hasattr(dev, "onCleanupAfterExperiment"))


class TestStanfordShutter(unittest.TestCase):
    def setUp(self):
        self.dev = sr470.StanfordShutter("sr470", {})
        self.sent = []
        self.dev.send = self.sent.append

    def test_enable_trigger_sends_enable_sequence(self):
        self.dev.enable_trigger()
        self.assertEqual(self.sent, ["POLR 0", "SRCE 2", "ENAB 1"])

    def test_enable_trigger_false_sends_disable(self):
        self.dev.enable_trigger(False)
        self.assertEqual(self.sent, ["POLR 0", "SRCE 2", "ENAB 0"])

    def test_cleanup_after_experiment_asserts_closed_first(self):
        self.dev.on_cleanup_after_experiment()
        self.assertEqual(self.sent, ["ASRT 0", "POLR 0", "SRCE 2", "ENAB 1"])

    def test_prepare_for_experiment_enables_trigger(self):
        self.dev.on_prepare_for_experiment(None)
        self.assertEqual(self.sent, ["POLR 0", "SRCE 2", "ENAB 1"])

    def test_send_writes_newline_terminated_command(self):
        dev = sr470.StanfordShutter("sr470", {})
        dev.connection = mock.Mock()
        dev.send("ENAB 1")
        dev.connection.write.assert_called_once_with(b"ENAB 1\n")


class TestAmbientLightCallbacks(unittest.TestCase):
    def test_private_callbacks_are_wired_in_handler_dict(self):
        dev = light.AmbientLight("ambient", {})
        self.assertTrue(callable(dev._get_exposure_time))
        self.assertTrue(callable(dev._set_exposure_time))
        self.assertTrue(callable(dev._set_enabled))

    def test_exposure_time_round_trip(self):
        dev = light.AmbientLight("ambient", {})
        dev._set_exposure_time("ambient", 12.5)
        self.assertEqual(dev._get_exposure_time("ambient"), 12.5)


class TestNanomover(unittest.TestCase):
    def make(self, config=None):
        with mock.patch.object(nanomover.events, "subscribe") as subscribe:
            dev = nanomover.Nanomover("nano", config or {})
        return dev, subscribe

    def test_init_subscribes_renamed_prompt(self):
        dev, subscribe = self.make()
        subscribed = {c.args[0]: c.args[1] for c in subscribe.call_args_list}
        self.assertEqual(
            subscribed[events.COCKPIT_INIT_COMPLETE], dev.prompt_exercise_stage
        )
        self.assertFalse(hasattr(dev, "prompt_rxercise_stage"))

    def test_get_position_with_mappers_and_cache(self):
        dev, _ = self.make()
        dev.axis_mapper = {0: 2, 1: 0, 2: 1}
        dev.axis_sign_mapper = {0: -1, 1: 1, 2: 1}
        dev.connection = mock.Mock()
        dev.connection.connection.posXYZ_OMX.return_value = [10, 20, 30]
        self.assertEqual(
            dev.get_position(should_use_cache=False), (-30.0, 10.0, 20.0)
        )
        # Cached afterwards: no further hardware access.
        dev.connection.connection.posXYZ_OMX.reset_mock()
        self.assertEqual(dev.get_position(0), -30.0)
        self.assertEqual(dev.get_position(), (-30.0, 10.0, 20.0))
        dev.connection.connection.posXYZ_OMX.assert_not_called()

    def test_get_handlers_uses_soft_limits_and_protocol_callbacks(self):
        dev, _ = self.make({"soft_limits": "((0, 100), (0, 200), (10, 300))"})
        handlers = dev.get_handlers()
        self.assertEqual(len(handlers), 3)
        self.assertEqual(list(handlers[1].soft_limits), [0, 200])
        self.assertEqual(
            set(handlers[0].callbacks),
            {"move_absolute", "move_relative", "get_position"},
        )

    def test_default_limits_when_config_missing(self):
        dev, _ = self.make()
        self.assertEqual(
            dev.soft_limits, [[0, 25000], [0, 25000], [7300, 25000]]
        )

    def test_prompt_exercise_stage_asks_when_timestamp_is_old(self):
        dev, _ = self.make()
        with (
            mock.patch.object(
                nanomover.cockpit.util.user_config, "get_value", return_value=0
            ) as get_value,
            mock.patch.object(
                nanomover.cockpit.util.user_config, "set_value"
            ) as set_value,
            mock.patch.object(
                nanomover.cockpit.gui.gui_utils,
                "get_user_permission",
                return_value=True,
            ) as ask,
            mock.patch.object(
                nanomover.cockpit.interfaces.stage_mover, "go_to_xy"
            ) as go_to_xy,
        ):
            dev.position_cache = (1, 2, 3)
            dev.prompt_exercise_stage()
        get_value.assert_called_once_with(
            "NanomoverLastExerciseTimestamp", default=0
        )
        ask.assert_called_once()
        self.assertIn("days since", ask.call_args.args[0])
        self.assertTrue(go_to_xy.called)
        self.assertEqual(
            set_value.call_args.args[0], "NanomoverLastExerciseTimestamp"
        )

    def test_prompt_exercise_stage_skips_when_recent(self):
        dev, _ = self.make()
        with (
            mock.patch.object(
                nanomover.cockpit.util.user_config,
                "get_value",
                return_value=nanomover.time.time(),
            ),
            mock.patch.object(
                nanomover.cockpit.gui.gui_utils, "get_user_permission"
            ) as ask,
        ):
            dev.prompt_exercise_stage()
        ask.assert_not_called()

    def test_receive_data_publishes_stopped_when_all_stopped(self):
        dev, _ = self.make()
        with (
            mock.patch.object(nanomover.events, "publish") as publish,
            mock.patch.object(dev, "publish_position"),
        ):
            dev.receiveData("nanoMotionStatus", [1, 2, 3], "allStopped")
        self.assertEqual(dev.cur_position, [1, 2, 3])
        stopped = [
            c.args
            for c in publish.call_args_list
            if c.args[0] == events.STAGE_STOPPED
        ]
        self.assertEqual(
            stopped,
            [(events.STAGE_STOPPED, "%d nanomover" % i) for i in range(3)],
        )


class TestPicomotor(unittest.TestCase):
    CONFIG = {"cal": "10", "ip_address": "1.2.3.4", "port": "23"}

    def make(self):
        with mock.patch.object(picomotor.events, "subscribe"):
            dev = picomotor.PicoMotorDevice("pico", dict(self.CONFIG))
        dev.xy_connection = mock.Mock()
        return dev

    def test_config_keys_are_unchanged(self):
        dev = self.make()
        self.assertEqual(dev.STAGE_CAL, 10.0)
        self.assertEqual(dev.PICO_CONTROLLER, "1.2.3.4")
        self.assertEqual(dev.PICO_PORT, 23)

    def test_get_xy_position_queries_controller(self):
        dev = self.make()
        dev.xy_connection.recv.side_effect = ["1>10", "1>20", "2>30"]
        position = dev.get_xy_position(should_use_cache=False)
        self.assertEqual(list(position), [1.0, 2.0, 3.0])
        sent = [c.args[0] for c in dev.xy_connection.sendall.call_args_list]
        self.assertEqual(sent, ["1>1 TP?\n", "1>2 TP?\n", "2>1 TP?\n"])

    def test_get_xy_position_single_axis_and_cache(self):
        dev = self.make()
        dev.xy_connection.recv.return_value = "1>50"
        self.assertEqual(dev.get_xy_position(1, should_use_cache=False), 5.0)
        dev.xy_connection.sendall.reset_mock()
        self.assertEqual(dev.get_xy_position(1), 5.0)
        dev.xy_connection.sendall.assert_not_called()

    def test_handler_get_position_lambda_uses_cache(self):
        dev = self.make()
        dev.xy_position_cache = [1.0, 2.0, 3.0]
        handlers = dev.get_handlers()
        self.assertEqual(len(handlers), 3)
        self.assertEqual(
            set(handlers[2].callbacks),
            {
                "move_absolute",
                "move_relative",
                "get_position",
                "get_movement_time",
            },
        )
        self.assertEqual(handlers[2].callbacks["get_position"](), 3.0)

    def test_move_xy_absolute_sends_wire_command(self):
        dev = self.make()
        with mock.patch.object(dev, "send_xy_position_updates") as updates:
            dev.move_xy_absolute(0, 2.0)
            dev.move_xy_absolute(2, 2.0)
        sent = [c.args[0] for c in dev.xy_connection.sendall.call_args_list]
        # X is flipped; Z is not.
        self.assertEqual(sent, ["1>1 PA -20\n", "2>1 PA 20\n"])
        self.assertEqual(updates.call_count, 2)

    def test_move_xy_relative_ignores_zero_delta(self):
        dev = self.make()
        with mock.patch.object(dev, "send_xy_position_updates"):
            dev.move_xy_relative(0, 0)
            dev.xy_connection.sendall.assert_not_called()
            dev.move_xy_relative(1, 3.0)
        dev.xy_connection.sendall.assert_called_once_with("1>2 PR -30\n")

    def test_get_xy_movement_time(self):
        dev = self.make()
        self.assertAlmostEqual(
            dev.get_xy_movement_time(0, 0, 300), (300 * 10.0) / 300 + 0.25
        )

    def test_on_abort_sends_abort(self):
        dev = self.make()
        dev.on_abort()
        dev.xy_connection.sendall.assert_called_once_with("AB\n")

    def test_old_names_are_gone(self):
        dev = self.make()
        for old in (
            "getXYPosition",
            "moveXYAbsolute",
            "sendXYCommand",
            "xyConnection",
            "axisMapper",
        ):
            self.assertFalse(hasattr(dev, old), old)


class TestLinkamStage(unittest.TestCase):
    def make(self, position=None):
        proxy = mock.Mock()
        proxy.get_value_limits.side_effect = [(-10, 10), (-20, 20)]
        if position is not None:
            proxy.get_position.return_value = position
        with (
            mock.patch.object(linkam_stage.Pyro4, "Proxy", return_value=proxy),
            mock.patch.object(linkam_stage.value_logger, "ValueLogger"),
            mock.patch.object(linkam_stage.events, "subscribe"),
        ):
            dev = linkam_stage.LinkamStage("linkam", {"uri": "PYRO:x@h:1"})
        return dev, proxy

    def test_limits_come_from_proxy(self):
        dev, _ = self.make()
        self.assertEqual(dev.hard_limits, ((-10, -20), (10, 20)))

    def test_get_position_queries_proxy_when_not_cached(self):
        dev, proxy = self.make({"X": 5, "Y": 6})
        self.assertEqual(dev.get_position(should_use_cache=False), (5, 6))
        proxy.get_position.reset_mock()
        self.assertEqual(dev.get_position(1), 6)
        proxy.get_position.assert_not_called()

    def test_get_position_keyword_is_renamed(self):
        dev, _ = self.make({"X": 1, "Y": 2})
        with self.assertRaises(TypeError):
            dev.get_position(shouldUseCache=False)

    def test_move_absolute_moves_one_axis(self):
        dev, proxy = self.make()
        with mock.patch.object(dev, "send_position_updates") as updates:
            dev.move_absolute(0, 12.7)
            proxy.move_to.assert_called_with(12, None)
            dev.move_absolute(1, 4)
            proxy.move_to.assert_called_with(None, 4)
            self.assertEqual(dev.motion_targets, [12, 4])
            proxy.move_to.reset_mock()
            dev.move_absolute(2, 1)
            proxy.move_to.assert_not_called()
        self.assertEqual(updates.call_count, 2)

    def test_move_relative_uses_position_cache(self):
        dev, proxy = self.make()
        dev.position_cache = (100, 200)
        with mock.patch.object(dev, "send_position_updates"):
            dev.move_relative(1, 5)
            proxy.move_to.assert_called_once_with(None, 205)
            proxy.move_to.reset_mock()
            dev.move_relative(0, 0)
            proxy.move_to.assert_not_called()


class TestStagePiezoLimits(unittest.TestCase):
    def make_handlers(self, config):
        executor = mock.Mock()
        with (
            mock.patch.object(
                stage.depot, "get_handler", return_value=executor
            ),
            mock.patch.object(
                stage.stage_positioner.PositionerHandler,
                "connect_to_analog_source",
            ),
        ):
            dev = stage.SimplePiezo("piezo", dict(config))
            return dev.get_handlers()

    def test_min_and_range(self):
        (handler,) = self.make_handlers({"min": "10", "range": "100"})
        self.assertEqual(list(handler.soft_limits), [10, 110])
        self.assertEqual(list(handler.hard_limits), [10, 110])

    def test_max_and_range(self):
        (handler,) = self.make_handlers({"max": "100", "range": "40"})
        self.assertEqual(list(handler.soft_limits), [60, 100])

    def test_only_range_starts_at_zero(self):
        (handler,) = self.make_handlers({"range": "50"})
        self.assertEqual(list(handler.soft_limits), [0, 50])

    def test_min_and_max(self):
        (handler,) = self.make_handlers({"min": "1", "max": "9"})
        self.assertEqual(list(handler.soft_limits), [1, 9])

    def test_missing_limits_raise(self):
        with self.assertRaises(Exception):
            self.make_handlers({})


class TestPolarizationRotator(unittest.TestCase):
    def test_get_handlers_registers_analog_with_executor(self):
        executor = mock.Mock()
        analog_handler = mock.Mock()
        executor.register_analog.return_value = analog_handler
        dev = polarization_rotator.PolarizationDevice(
            "SI polarizer",
            {
                "analog_source": "exec",
                "analog_line": "1",
                "gain": "2",
                "offset": "0.5",
                "idle_voltage": "1.5",
                "si_voltages": "488: 0.9, 1.0, 1.1",
            },
        )
        with mock.patch.object(
            polarization_rotator.depot, "get_handler", return_value=executor
        ) as get_handler:
            handlers = dev.get_handlers()
        get_handler.assert_called_once_with(
            "exec", polarization_rotator.depot.EXECUTOR
        )
        executor.register_analog.assert_called_once_with(dev, "1", 0.5, 2.0)
        self.assertEqual(handlers, [analog_handler])
        self.assertEqual(
            analog_handler.positions, {"488": (0.9, 1.0, 1.1), None: [1.5] * 3}
        )

    def test_missing_control_source_raises(self):
        dev = polarization_rotator.PolarizationDevice("SI polarizer", {})
        with mock.patch.object(
            polarization_rotator.depot, "get_handler", return_value=None
        ):
            with self.assertRaises(Exception):
                dev.get_handlers()


if __name__ == "__main__":
    unittest.main()
