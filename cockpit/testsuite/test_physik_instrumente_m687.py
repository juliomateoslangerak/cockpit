"""Tests for the PhysikInstrumenteM687 XY stage device after its PEP 8 rename.

The serial connection and the event bus are mocked; no hardware is needed.
"""

import configparser
import time
import unittest
from unittest import mock

import cockpit.config
from cockpit import events
from cockpit.devices import physik_instrumente_m687 as physik


def make_device(config=None):
    cfg = {"port": "com6", "baud": "115200", "timeout": "0.1"}
    cfg.update(config or {})
    with mock.patch.object(physik.events, "subscribe") as subscribe:
        dev = physik.PhysikInstrumenteM687("m687", cfg)
    return dev, subscribe


class TestConfigTypeConversion(unittest.TestCase):
    """Depot ``type:`` strings for this device, canonical and legacy."""

    def setUp(self):
        self.config = configparser.ConfigParser(
            converters=cockpit.config._type_converters
        )

    def test_canonical_module_path_resolves_class(self):
        self.config.read_dict(
            {
                "sect": {
                    "type": "cockpit.devices.physik_instrumente_m687"
                    ".PhysikInstrumenteM687"
                }
            }
        )
        self.assertIs(
            self.config.gettype("sect", "type"), physik.PhysikInstrumenteM687
        )

    def test_legacy_module_path_resolves_class(self):
        self.config.read_dict(
            {
                "sect": {
                    "type": "cockpit.devices.physikInstrumente_M687"
                    ".PhysikInstrumenteM687"
                }
            }
        )
        self.assertIs(
            self.config.gettype("sect", "type"), physik.PhysikInstrumenteM687
        )


class TestConstruction(unittest.TestCase):
    def test_canonical_class_is_named_pep8_module(self):
        self.assertEqual(
            physik.PhysikInstrumenteM687.__module__,
            "cockpit.devices.physik_instrumente_m687",
        )

    def test_renamed_attributes_exist(self):
        dev, _ = make_device()
        self.assertIsNone(dev.xy_connection)
        self.assertEqual(dev.xy_position_cache, (10**100, 10**100))
        self.assertEqual(dev.xy_motion_targets, [None, None])
        self.assertEqual(dev.axis_mapper, {0: 1, 1: 2})
        self.assertEqual(dev.axis_sign_mapper, {0: 1, 1: 1})

    def test_old_attributes_are_gone(self):
        dev, _ = make_device()
        for old in (
            "xyConnection",
            "xyLock",
            "xyPositionCache",
            "xyMotionTargets",
            "axisMapper",
            "axisSignMapper",
        ):
            self.assertFalse(hasattr(dev, old))

    def test_swapaxes_and_inversion_are_applied(self):
        dev, _ = make_device({"swapaxes": "1", "invert0": "1", "invert1": "0"})
        self.assertEqual(dev.axis_mapper, {0: 2, 1: 1})
        self.assertEqual(dev.axis_sign_mapper, {0: -1, 1: 1})

    def test_subscribes_to_abort_and_macro_stage_paint(self):
        _, subscribe = make_device()
        events_subscribed = [c.args[0] for c in subscribe.call_args_list]
        self.assertIn(events.USER_ABORT, events_subscribed)
        self.assertIn(events.MACRO_STAGE_XY_DRAW, events_subscribed)

    def test_soft_limits_parsed_from_config(self):
        dev, _ = make_device({"soft_limits": "((-100,-200),(300,400))"})
        self.assertEqual(dev.soft_limits, ((-100, -200), (300, 400)))

    def test_default_soft_limits_when_missing(self):
        dev, _ = make_device()
        self.assertEqual(dev.soft_limits, ((-67500, 67500), (-42500, 42500)))


class TestXYCommands(unittest.TestCase):
    def setUp(self):
        self.dev, _ = make_device()
        self.dev.xy_connection = mock.Mock()
        # Skip the error-check round trip; only the motion command matters here.
        self.dev.get_xy_response = mock.Mock(return_value=b"0\n")

    def test_send_xy_command_writes_newline_terminated_bytes(self):
        self.dev.send_xy_command(b"SVO 1 1", should_check_errors=False)
        self.dev.xy_connection.write.assert_called_once_with(b"SVO 1 1\n")

    def test_error_code_0_is_no_error(self):
        self.dev.get_xy_response = mock.Mock(side_effect=[b"ok\n", b"0\n"])
        self.dev.send_xy_command(b"MOV 1 1.0")

    def test_error_code_5_message_is_unchanged(self):
        self.assertEqual(
            physik.ERROR_CODES[5],
            "Can't move because servo is off or axis has no reference. Most "
            "likely the axis needs to be rehomed; remove the objective from "
            "its mount (or otherwise ensure no collision is possible with "
            "the sample holder, e.g. raise the Picomotors) and then do "
            "sendXYCommand('FRF'). Please know what you are doing before "
            "you try this.",
        )

    def test_unexpected_error_code_raises(self):
        self.dev.get_xy_response = mock.Mock(side_effect=[b"ok\n", b"3\n"])
        with self.assertRaises(RuntimeError):
            self.dev.send_xy_command(b"MOV 1 1.0")

    def test_move_xy_absolute_sends_mapped_axis_in_mm(self):
        with mock.patch.object(self.dev, "send_xy_position_updates"):
            self.dev.xy_position_cache = (0.0, 0.0)
            self.dev.move_xy_absolute(0, 5000)
        self.dev.xy_connection.write.assert_any_call(b"MOV 1 5.000000\n")
        self.assertEqual(self.dev.xy_motion_targets, [5000, None])

    def test_move_xy_absolute_inverted_axis(self):
        self.dev.axis_sign_mapper = {0: -1, 1: 1}
        with mock.patch.object(self.dev, "send_xy_position_updates"):
            self.dev.xy_position_cache = (0.0, 0.0)
            self.dev.move_xy_absolute(0, 2000)
        self.dev.xy_connection.write.assert_any_call(b"MOV 1 -2.000000\n")

    def test_move_xy_relative_zero_delta_is_ignored(self):
        with mock.patch.object(self.dev, "move_xy_absolute") as absolute:
            self.dev.move_xy_relative(0, 0)
            absolute.assert_not_called()

    def test_move_xy_relative_adds_to_cache(self):
        with mock.patch.object(self.dev, "move_xy_absolute") as absolute:
            self.dev.xy_position_cache = (100, 200)
            self.dev.move_xy_relative(1, 50)
            absolute.assert_called_once_with(1, 250)

    def test_move_into_banned_rectangle_is_refused(self):
        self.dev.xy_position_cache = (0, 0)
        banned = (((-10, -10), (10, 10)),)
        with mock.patch.object(physik, "BANNED_RECTANGLES", banned):
            with mock.patch.object(self.dev, "send_xy_position_updates"):
                with self.assertRaises(RuntimeError):
                    self.dev.move_xy_absolute(0, 5)
        self.dev.xy_connection.write.assert_not_called()
        self.assertEqual(self.dev.xy_motion_targets, [None, None])

    def test_on_abort_halts_motion(self):
        self.dev.get_xy_response = mock.Mock(side_effect=[b"ok\n", b"10\n"])
        self.dev.on_abort()
        self.dev.xy_connection.write.assert_any_call(b"HLT\n")


class TestHandlers(unittest.TestCase):
    def test_get_handlers_callback_keys(self):
        dev, _ = make_device()
        handlers = dev.get_handlers()
        self.assertEqual(len(handlers), 2)
        for handler in handlers:
            self.assertEqual(
                set(handler.callbacks),
                {"move_absolute", "move_relative", "get_position"},
            )
            self.assertEqual(
                handler.callbacks["move_absolute"], dev.move_xy_absolute
            )
            self.assertEqual(
                handler.callbacks["move_relative"], dev.move_xy_relative
            )
            self.assertEqual(
                handler.callbacks["get_position"], dev.get_xy_position
            )


class TestExerciseStage(unittest.TestCase):
    def test_subscription_name_is_renamed(self):
        dev, _ = make_device()
        self.assertTrue(callable(dev.prompt_exercise_stage))

    def test_recent_exercise_does_not_prompt(self):
        dev, _ = make_device()
        recent = time.time() - 60.0
        with (
            mock.patch.object(
                physik.cockpit.util.user_config,
                "get_value",
                return_value=recent,
            ),
            mock.patch.object(
                physik.cockpit.gui.gui_utils, "get_user_permission"
            ) as permission,
        ):
            dev.prompt_exercise_stage()
        permission.assert_not_called()

    def test_old_exercise_prompts_and_stores_timestamp(self):
        dev, _ = make_device()
        dev.soft_limits = ((0, 0), (1, 1))
        with (
            mock.patch.object(
                physik.cockpit.util.user_config,
                "get_value",
                return_value=0,
            ),
            mock.patch.object(
                physik.cockpit.util.user_config, "set_value"
            ) as set_value,
            mock.patch.object(
                physik.cockpit.gui.gui_utils,
                "get_user_permission",
                return_value=True,
            ) as permission,
            mock.patch.object(
                physik.cockpit.interfaces.stage_mover, "go_to_xy"
            ) as go_to_xy,
        ):
            dev.prompt_exercise_stage()
        permission.assert_called_once()
        self.assertTrue(go_to_xy.called)
        set_value.assert_called_once()
        self.assertEqual(
            set_value.call_args.args[0], "PIM687LastExerciseTimestamp"
        )


if __name__ == "__main__":
    unittest.main()
