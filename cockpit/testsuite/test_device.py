"""Tests for the Device base class and the transport boundaries of its subclasses."""

import decimal
import unittest
from unittest import mock

from microscope import ROI

from cockpit.devices import camera
from cockpit.devices import device
from cockpit.devices import microscope_camera
from cockpit.devices import server
from cockpit.devices import sr470


class TestDeviceProtocolBoundary(unittest.TestCase):
    def test_make_ui_name_is_unchanged(self):
        # main_window calls make_ui on raw Device objects, so the name must stay.
        dev = device.Device("dev")
        self.assertIsNone(dev.make_ui(None))
        self.assertTrue(hasattr(device.Device, "make_ui"))

    def test_ip_address_is_read_from_config_key(self):
        # The config key stays "ip_address"; the attribute is ip_address.
        dev = device.Device("dev", {"ip_address": "10.0.0.5"})
        self.assertEqual(dev.ip_address, "10.0.0.5")

    def test_camera_base_transform_is_renamed(self):
        cam = camera.CameraDevice("cam", {"transform": "(1,0,0)"})
        self.assertEqual(cam.base_transform, (True, False, False))
        self.assertFalse(hasattr(cam, "baseTransform"))


class TestLifecycleNames(unittest.TestCase):
    LIFECYCLE_NAMES = (
        "get_handlers",
        "make_ui",
        "perform_subscriptions",
        "make_initial_publications",
        "finalize_initialization",
        "on_exit",
    )

    def test_lifecycle_names_are_unchanged(self):
        for name in self.LIFECYCLE_NAMES:
            with self.subTest(name=name):
                self.assertTrue(callable(getattr(device.Device, name)))

    def test_lifecycle_methods_are_callable_on_raw_device(self):
        dev = device.Device("dev")
        self.assertEqual(dev.get_handlers(), [])
        self.assertIsNone(dev.make_ui(None))
        for name in self.LIFECYCLE_NAMES[2:]:
            with self.subTest(name=name):
                self.assertIsNone(getattr(dev, name)())


class TestCockpitServerRegister(unittest.TestCase):
    def test_register_uses_configured_ip_address(self):
        srv = server.CockpitServer("server", {"ip_address": "10.1.2.3"})
        func = mock.Mock()
        with mock.patch.object(server, "ServerDaemon") as daemon_cls:
            uri = srv.register(func)
        self.assertEqual(uri, "PYRO:mui@10.1.2.3:7701")
        daemon_cls.assert_called_once_with("mui", func, 7701, "10.1.2.3")
        daemon_cls.return_value.serve.assert_called_once_with()

    def test_register_local_ip_overrides_configured_address(self):
        srv = server.CockpitServer("server", {"ip_address": "10.1.2.3"})
        with mock.patch.object(server, "ServerDaemon") as daemon_cls:
            uri = srv.register(mock.Mock(), local_ip="1.1.1.1")
        self.assertEqual(uri, "PYRO:mui@1.1.1.1:7701")
        daemon_cls.assert_called_once_with("mui", mock.ANY, 7701, "1.1.1.1")

    def test_register_defaults_to_loopback(self):
        srv = server.CockpitServer("server", {})
        self.assertEqual(srv.ip_address, "127.0.0.1")
        with mock.patch.object(server, "ServerDaemon"):
            uri = srv.register(mock.Mock())
        self.assertEqual(uri, "PYRO:mui@127.0.0.1:7701")


class TestCameraUpdateTransform(unittest.TestCase):
    def test_update_transform_reads_base_transform(self):
        cam = camera.CameraDevice("cam", {"transform": "(1,0,0)"})
        cam.update_transform((False, True, False))
        self.assertEqual(cam._transform, (True, True, False))


class TestMicroscopeCameraInitialize(unittest.TestCase):
    def test_initialize_applies_base_transform_and_roi(self):
        cam = microscope_camera.MicroscopeCamera(
            "cam",
            {
                "uri": "PYRO:cam@host:7000",
                "transform": "(1,0,0)",
                "roi": "(0,0,10,10)",
            },
        )
        with mock.patch("Pyro4.Proxy") as proxy_cls, mock.patch(
            "cockpit.util.listener.Listener"
        ) as listener_cls, mock.patch("cockpit.events.publish"):
            cam.initialize()
        proxy = proxy_cls.return_value
        proxy_cls.assert_called_once_with("PYRO:cam@host:7000")
        proxy.set_transform.assert_called_once_with((True, False, False))
        proxy.set_roi.assert_called_once_with(ROI(0, 0, 10, 10))
        self.assertIs(listener_cls.call_args.args[0], proxy)


def _make_camera():
    cam = microscope_camera.MicroscopeCamera(
        "cam", {"uri": "PYRO:cam@host:7000"}
    )
    cam._proxy = mock.MagicMock()
    return cam


class TestMicroscopeCameraCallbacks(unittest.TestCase):
    def test_handler_callbacks_are_called_positionally(self):
        # CameraHandler calls callbacks[...](name, value) positionally, so the
        # PEP 8 parameter names must work with that call form.
        cam = _make_camera()
        cam._proxy.get_exposure_time.return_value = 0.05
        cam._proxy.get_cycle_time.return_value = 0.1
        self.assertEqual(float(cam.get_exposure_time("cam", True)), 50.0)
        self.assertIsInstance(cam.get_exposure_time("cam", True), decimal.Decimal)
        self.assertIsInstance(cam.get_exposure_time("cam", False), float)
        self.assertIsInstance(
            cam.get_time_between_exposures("cam", True), decimal.Decimal
        )
        cam.set_exposure_time("cam", 20)
        cam._proxy.set_exposure_time.assert_called_once_with(0.02)

    def test_handler_callback_keys_map_to_renamed_methods(self):
        # The handler protocol keys are unchanged; only the bound methods
        # they refer to are PEP 8.
        cam = _make_camera()
        with mock.patch(
            "cockpit.handlers.camera.CameraHandler"
        ) as handler_cls:
            cam.get_handlers()
        callbacks = handler_cls.call_args.args[2]
        expected = {
            "get_image_size": cam.get_image_size,
            "get_time_between_exposures": cam.get_time_between_exposures,
            "prepare_for_experiment": cam.prepare_for_experiment,
            "get_exposure_time": cam.get_exposure_time,
            "set_exposure_time": cam.set_exposure_time,
            "get_shuttering_mode": cam.get_shuttering_mode,
            "get_savefile_info": cam.get_savefile_info,
            "set_roi": cam.set_roi,
            "get_roi": cam.get_roi,
            "get_sensor_shape": cam.get_sensor_shape,
            "make_ui": cam.make_ui,
            "soft_trigger": cam.soft_trigger,
        }
        for key, method in expected.items():
            with self.subTest(key=key):
                self.assertEqual(callbacks[key], method)

    def test_get_image_size_uses_roi_and_binning(self):
        cam = _make_camera()
        cam._proxy.get_roi.return_value = ROI(0, 0, 100, 60)
        cam._proxy.get_binning.return_value = (2, 3)
        self.assertEqual(cam.get_image_size("cam"), (50, 20))

    def test_soft_trigger_only_fires_when_enabled(self):
        cam = _make_camera()
        cam.enabled = False
        cam.soft_trigger()
        cam._proxy.soft_trigger.assert_not_called()
        cam.enabled = True
        cam.soft_trigger()
        cam._proxy.soft_trigger.assert_called_once_with()

    def test_handler_keyword_names_are_unchanged(self):
        # Experiment code calls the handler with is_exact=..., so the handler
        # must accept that keyword and pass it through to the callback.
        from cockpit.handlers.camera import CameraHandler

        callbacks = {
            "get_exposure_time": mock.Mock(return_value=50.0),
            "get_time_between_exposures": mock.Mock(return_value=10.0),
            "get_min_exposure_time": mock.Mock(return_value=1.0),
        }
        handler = CameraHandler("cam", "group", callbacks, "exposure")
        for name in ("get_exposure_time", "get_time_between_exposures"):
            with self.subTest(name=name):
                getattr(handler, name)(is_exact=True)
                callbacks[name].assert_called_with("cam", True)
        # get_min_exposure_time takes only the name from its callback and
        # converts the result to Decimal when is_exact is set.
        self.assertIsInstance(handler.get_min_exposure_time(is_exact=True), decimal.Decimal)
        callbacks["get_min_exposure_time"].assert_called_with("cam")
        handler.get_exposure_time()
        callbacks["get_exposure_time"].assert_called_with("cam", False)

    def test_event_subscriptions_use_renamed_methods(self):
        cam = _make_camera()
        with mock.patch.object(
            microscope_camera.events, "subscribe"
        ) as subscribe:
            cam.perform_subscriptions()
        subscribe.assert_any_call(
            microscope_camera.events.CLEANUP_AFTER_EXPERIMENT,
            cam.cleanup_after_experiment,
        )
        subscribe.assert_any_call(
            microscope_camera.events.OBJECTIVE_CHANGE, cam.on_objective_change
        )

    def test_objective_change_updates_transform(self):
        cam = _make_camera()
        cam.base_transform = (True, False, False)
        handler = mock.Mock(transform=(False, True, False))
        with mock.patch.object(microscope_camera.events, "publish"):
            cam.on_objective_change(handler)
        cam._proxy.set_transform.assert_called_once_with((True, True, False))

    def test_set_enabled_callback_is_enable_camera(self):
        # The handler protocol key stays "set_enabled"; only the Python
        # method name is PEP 8.
        cam = _make_camera()
        with mock.patch(
            "cockpit.handlers.camera.CameraHandler"
        ) as handler_cls:
            cam.get_handlers()
        callbacks = handler_cls.call_args.args[2]
        self.assertEqual(callbacks["set_enabled"], cam.enable_camera)
        self.assertTrue(callable(callbacks["set_enabled"]))


class TestMicroscopeLaser(unittest.TestCase):
    def setUp(self):
        from cockpit.devices import microscope_device

        self.laser = microscope_device.MicroscopeLaser(
            "laser", {"uri": "PYRO:laser@h:1", "wavelength": "488"}
        )
        self.laser._proxy = mock.MagicMock()
        self.handlers = self.laser.get_handlers()
        self.light = self.handlers[-1]
        self.power = self.handlers[0]

    def test_exposure_time_set_then_get_round_trips(self):
        callbacks = self.light.callbacks
        callbacks["set_exposure_time"]("laser", 42)
        self.assertEqual(callbacks["get_exposure_time"]("laser"), 42)
        self.assertEqual(self.light.get_exposure_time(), 42)

    def test_exposure_time_defaults_to_100(self):
        self.assertEqual(self.light.get_exposure_time(), 100)

    def test_enable_callback_reaches_proxy(self):
        callbacks = self.light.callbacks
        callbacks["set_enabled"]("laser", True)
        self.laser._proxy.enable.assert_called_once_with()
        callbacks["set_enabled"]("laser", False)
        self.laser._proxy.disable.assert_called_once_with()

    def test_power_callbacks_reach_proxy(self):
        self.laser._proxy.power = 0.5
        self.assertEqual(self.power.callbacks["get_power"](), 0.5)
        # setPower runs in a new thread, so wait for the proxy write.
        self.power.callbacks["set_power"](0.25)
        self.assertTrue(self._wait_for(lambda: self.laser._proxy.power == 0.25))
        self.assertEqual(self.laser._proxy.power, 0.25)

    @staticmethod
    def _wait_for(predicate, timeout=2.0):
        # Bounded poll for the async setPower thread; the caller asserts the result.
        import time

        end = time.time() + timeout
        while not predicate() and time.time() < end:
            time.sleep(0.01)
        return predicate()


class TestMicroscopeValueLoggerPull(unittest.TestCase):
    def test_pull_mode_polls_remote_values(self):
        from cockpit.devices import microscope_device

        dev = microscope_device.MicroscopeValueLogger(
            "vl",
            {"uri": "PYRO:vl@h:1", "pulldata": "True", "pollinterval": "5"},
        )
        with mock.patch("Pyro4.Proxy") as proxy_cls, mock.patch.object(
            microscope_device.value_logger, "PollingLogger"
        ) as polling_cls, mock.patch.object(
            microscope_device.events, "publish"
        ) as publish:
            proxy = proxy_cls.return_value
            proxy.get_num_sensors.return_value = 2
            proxy.getValues.return_value = [1.0, 2.0]
            dev.initialize()
            polling_cls.assert_called_once()
            args = polling_cls.call_args.args
            self.assertEqual(args[1], 5)
            self.assertEqual(args[2], dev.get_remote_values)
            self.assertEqual(dev.get_remote_values(), [1.0, 2.0])
            publish.assert_any_call(microscope_device.events.VALUELOGGER_INPUT, [1.0, 2.0])
        proxy.getValues.assert_called_with()


class TestMicroscopeFilterCallbacks(unittest.TestCase):
    def test_filter_callbacks_reach_proxy(self):
        from cockpit.devices import microscope_device

        filt = microscope_device.MicroscopeFilter.__new__(
            microscope_device.MicroscopeFilter
        )
        filt.name = "filt"
        filt._proxy = mock.MagicMock()
        filt._proxy._pyroUri = "PYRO:filt@h:1"
        filt.filters = ["f0", "f1"]
        filt.cameras = []
        filt.lights = []
        with mock.patch(
            "cockpit.handlers.filter.FilterHandler"
        ) as handler_cls, mock.patch(
            "Pyro4.Proxy"
        ) as async_proxy_cls:
            filt.get_handlers()
            callbacks = handler_cls.call_args.args[3]
            self.assertEqual(callbacks["set_position"], filt.set_position)
            self.assertEqual(callbacks["get_position"], filt.get_position)
            self.assertEqual(callbacks["get_filters"], filt.get_filters)
            filt._proxy.get_position.return_value = 3
            self.assertEqual(callbacks["get_position"](), 3)
            self.assertEqual(callbacks["get_filters"](), ["f0", "f1"])
            callbacks["set_position"](1)
        async_proxy = async_proxy_cls.return_value
        async_proxy.set_position.assert_called_once_with(1)


class TestMicroscopeStageAxisMotion(unittest.TestCase):
    def setUp(self):
        from cockpit.devices import microscope_device

        self.axis = mock.MagicMock()
        self.axis.limits.lower = 0.0
        self.axis.limits.upper = 100.0
        self.axis.position = 500.0
        self.wrapper = microscope_device._MicroscopeStageAxis(
            self.axis, 0, 10.0, "stage"
        )
        self.callbacks = self.wrapper.get_handler().callbacks

    def test_callbacks_are_renamed_methods(self):
        self.assertEqual(
            self.callbacks["move_absolute"], self.wrapper.move_absolute
        )
        self.assertEqual(
            self.callbacks["move_relative"], self.wrapper.move_relative
        )
        self.assertEqual(
            self.callbacks["get_position"], self.wrapper.get_position
        )
        self.assertEqual(
            self.callbacks["get_movement_time"], self.wrapper.get_movement_time
        )

    def test_get_position_scales_by_units_per_micron(self):
        self.assertEqual(self.callbacks["get_position"](0), 50.0)

    def test_move_absolute_scales_to_device_units(self):
        from cockpit.devices import microscope_device

        with mock.patch.object(microscope_device.events, "publish"):
            self.callbacks["move_absolute"](0, 12.5)
        self.axis.move_to.assert_called_once_with(125.0)

    def test_move_relative_scales_to_device_units(self):
        from cockpit.devices import microscope_device

        with mock.patch.object(microscope_device.events, "publish"):
            self.callbacks["move_relative"](0, -2.0)
        self.axis.move_by.assert_called_once_with(-20.0)


class TestMicroscopeStageHandlers(unittest.TestCase):
    def test_get_handlers_returns_each_axis_handler(self):
        from cockpit.devices import microscope_device

        stage = microscope_device.MicroscopeStage(
            "stage", {"uri": "PYRO:stage@h:1"}
        )
        axis = mock.MagicMock()
        axis.limits.lower = 0.0
        axis.limits.upper = 100.0
        axis.position = 50.0
        stage._axes = [
            microscope_device._MicroscopeStageAxis(axis, 0, 1.0, "stage")
        ]
        handlers = stage.get_handlers()
        self.assertEqual(len(handlers), 1)
        self.assertIs(handlers[0], stage._axes[0].get_handler())
        self.assertEqual(handlers[0].callbacks["get_position"](0), 50.0)


class TestDIOOutputWindow(unittest.TestCase):
    def test_window_builds_and_updates_state(self):
        import wx
        from cockpit.devices import microscope_device

        dio = microscope_device.MicroscopeDIO(
            "dio", {"uri": "PYRO:dio@h:1", "labels": "['a', 'b']"}
        )
        dio.num_lines = 2
        dio.labels = ["a", "b"]
        dio.io_map = [True, False]
        dio._proxy = mock.MagicMock()
        dio._proxy.read_all_lines.return_value = [None, None]
        dio._proxy.get_IO_state.side_effect = lambda line: dio.io_map[line]
        dio._proxy.read_line.return_value = 0
        app = wx.App(False)
        frame = None
        try:
            with mock.patch.object(
                microscope_device.events, "subscribe"
            ), mock.patch.object(microscope_device.events, "publish"):
                frame = microscope_device.DIOOutputWindow(dio, parent=None)
                self.assertEqual(sorted(frame.line_to_button), [0, 1])
                frame.output_changed(0, True)
                frame.input_changed(1, False)
                frame.update_state()
                dio._proxy.set_IO_state.assert_any_call(0, True)
                dio._proxy.set_IO_state.assert_any_call(1, False)
        finally:
            if frame is not None:
                frame.Destroy()
            app.Destroy()


class TestHandlerCleanupProtocol(unittest.TestCase):
    def test_handler_cleanup_after_experiment_is_callable(self):
        # experiment.py calls handler.cleanup_after_experiment(), so the handler
        # protocol name must stay camelCase even though the device-side
        # subscription method is cleanup_after_experiment.
        from cockpit.handlers import camera as handler_camera
        from cockpit.handlers import device as handler_device

        for cls in (handler_device.DeviceHandler, handler_camera.CameraHandler):
            with self.subTest(cls=cls.__name__):
                self.assertTrue(
                    callable(getattr(cls, "cleanup_after_experiment", None))
                )
        handler = handler_device.DeviceHandler("dev", "group", False, {}, "t")
        handler.cleanup_after_experiment()
        handler.cleanup_after_experiment(is_cleanup_final=False)

    def test_experiment_cleanup_calls_handler_protocol_method(self):
        from cockpit.experiment import experiment as experiment_mod
        from cockpit.handlers import device as handler_device

        handler = mock.Mock(spec=handler_device.DeviceHandler)
        exp = experiment_mod.Experiment.__new__(experiment_mod.Experiment)
        exp.all_handlers = [handler]
        exp.initial_altitude = None
        with mock.patch.object(experiment_mod.events, "publish") as publish, \
                mock.patch.object(experiment_mod.gc, "collect"):
            exp.cleanup()
        handler.cleanup_after_experiment.assert_called_once_with()
        publish.assert_any_call(experiment_mod.events.CLEANUP_AFTER_EXPERIMENT)


class TestMicroscopeBaseSettings(unittest.TestCase):
    def test_update_settings_applies_to_proxy(self):
        from cockpit.devices import microscope_device

        dev = microscope_device.MicroscopeBase("dev", {"uri": "PYRO:d@h:1"})
        dev._proxy = mock.MagicMock()
        dev.update_settings({"a": 1})
        dev._proxy.update_settings.assert_called_once_with({"a": 1})
        self.assertTrue(callable(dev.show_settings))


class TestMicroscopeDIO(unittest.TestCase):
    def setUp(self):
        from cockpit.devices import microscope_device

        self.md = microscope_device
        self.dio = microscope_device.MicroscopeDIO(
            "dio",
            {
                "uri": "PYRO:dio@h:1",
                "io_map": "1,0",
                "labels": "['a', 'b']",
                "paths": "{'p': [1, 0]}",
            },
        )
        with mock.patch("Pyro4.Proxy") as proxy_cls, mock.patch(
            "cockpit.util.listener.Listener"
        ), mock.patch.object(
            microscope_device.value_logger, "ValueLogger"
        ), mock.patch.object(
            microscope_device.events, "subscribe"
        ), mock.patch.object(
            microscope_device.events, "publish"
        ):
            proxy_cls.return_value.get_num_lines.return_value = 2
            self.dio.initialize()

    def test_initialize_reads_config_into_renamed_attributes(self):
        self.assertEqual(self.dio.num_lines, 2)
        self.assertEqual(self.dio.io_map, [True, False])
        self.assertEqual(self.dio.labels, ["a", "b"])
        self.assertEqual(self.dio.paths, {"p": [1, 0]})
        self.assertEqual(self.dio.get_labels(), ["a", "b"])
        self.assertEqual(self.dio.get_paths(), {"p": [1, 0]})
        self.dio._proxy.set_all_IO_state.assert_called_once_with([True, False])

    def test_handler_callback_keys_are_unchanged(self):
        handler = self.dio.get_handlers()[0]
        for key in (
            "set_outputs",
            "set_io_state",
            "get_outputs",
            "get_io_state",
            "get_paths",
            "write_line",
            "get_labels",
            "enable",
        ):
            with self.subTest(key=key):
                self.assertIn(key, handler.callbacks)

    def test_read_line_update_gui_keyword(self):
        self.dio._proxy.read_line.return_value = True
        with mock.patch.object(self.md.events, "publish") as publish:
            self.assertTrue(self.dio.read_line(1, update_gui=False))
            publish.assert_not_called()
            self.dio.read_line(1)
            publish.assert_called_once_with(self.md.events.DIO_INPUT, 1, True)

    def test_set_and_get_io_state_use_io_map(self):
        self.dio.set_io_state(1, True)
        self.assertEqual(self.dio.io_map[1], True)
        self.assertTrue(self.dio.get_io_state(1, cache=True))
        self.assertFalse(hasattr(self.dio, "IOMap"))


class TestStanfordShutterTelnet(unittest.TestCase):
    def test_initialize_connects_to_configured_ip_address(self):
        shutter = sr470.StanfordShutter(
            "shutter", {"ip_address": "192.168.0.10", "port": "5024"}
        )
        with mock.patch.object(sr470.telnetlib, "Telnet") as telnet_cls:
            shutter.initialize()
        telnet_cls.assert_called_once_with("192.168.0.10", 5024, timeout=5)
        telnet_cls.return_value.write.assert_any_call(b"POLR 0\n")


if __name__ == "__main__":
    unittest.main()
