"""Tests for executor devices: adapted actions and protocol boundaries."""

import json
import unittest
from unittest import mock

import numpy as np

from cockpit.devices import executor_devices
from cockpit.devices import ni_crio_fpga


def _json_commands_from_send(sock):
    """Decode JSON command payloads passed to Connection.connection.send."""
    commands = []
    for call in sock.send.call_args_list:
        payload = call.args[0]
        if payload == b"\r\n":
            continue
        commands.append(json.loads(payload.decode()))
    return commands


class _AdaptingExecutor(executor_devices.ExecutorDevice):
    def _adapt_actions(self, actions):
        return ["adapted", len(actions)]


class TestAdaptActions(unittest.TestCase):
    def test_default_adapt_actions_returns_input(self):
        dev = executor_devices.ExecutorDevice("executor", {})
        actions = [(0.0, (1, [0, 0]))]
        self.assertIs(dev._adapt_actions(actions), actions)

    def test_execute_table_sends_adapted_actions(self):
        dev = _AdaptingExecutor("executor", {})
        dev.connection = mock.MagicMock()
        table = [(0.0, (1, [0, 0])), (1.0, (0, [0, 0]))]
        with mock.patch.object(executor_devices.events, "publish"), \
                mock.patch.object(executor_devices.events, "execute_and_wait_for"):
            dev.execute_table(table, 0, 2, 1, None)
        dev.connection.PrepareActions.assert_called_once_with(["adapted", 2], 1)


class TestNIcRIOAdaptActions(unittest.TestCase):
    def setUp(self):
        # Bypass __init__ so no hardware connection is needed.
        self.dev = ni_crio_fpga.NIcRIO.__new__(ni_crio_fpga.NIcRIO)
        self.dev.tickrate = 10
        self.dev._alines = 2
        self.dev._last_analogs = [0, 0]
        self.dev._last_digital = 0

    def test_adapt_actions_builds_profile(self):
        actions = [(0.0, (1, [0, 0])), (1.0, (0, [5, 0]))]
        description, digitals, analogs = self.dev._adapt_actions(actions)
        self.assertEqual(description["count"], 10)
        self.assertEqual(description["nDigital"], 2)
        self.assertEqual(description["nAnalog"], [2, 1])
        self.assertEqual(description["InitDio"], 0)
        np.testing.assert_array_equal(digitals, [[0, 1], [10, 0]])
        np.testing.assert_array_equal(analogs[0], [[0, 0], [10, 5]])
        np.testing.assert_array_equal(analogs[1], [[0, 0]])
        self.assertIs(self.dev._last_profile[0], description)


class TestLegacyDSPProtocol(unittest.TestCase):
    def setUp(self):
        self.dev = executor_devices.LegacyDSP("dsp", {})
        self.dev.connection = mock.MagicMock()

    def test_set_analog_updates_position_and_moves_remote(self):
        self.dev.set_analog(2, 1234.7)
        self.assertEqual(self.dev._current_analogs[2], 1234.7)
        self.dev.connection.MoveAbsoluteADU.assert_called_once_with(2, 1234)

    def test_get_analog_returns_current_position(self):
        self.dev._current_analogs[1] = 42
        self.assertEqual(self.dev.get_analog(1), 42)

    def test_on_prepare_for_experiment_snapshots_positions(self):
        self.dev.receive_uri = "PYRO:receiver"
        self.dev._current_analogs = [1, 2, 3, 4]
        self.dev.connection.ReadDigital.return_value = 7
        self.dev.on_prepare_for_experiment()
        self.dev.connection.receiveClient.assert_called_once_with("PYRO:receiver")
        self.assertEqual(self.dev._last_analogs, [1, 2, 3, 4])
        self.assertEqual(self.dev._last_digital, 7)

    def test_execute_table_builds_profile(self):
        table = [(0.0, (1, [0, 0, 0, 0])), (1.0, (0, [10, 0, 0, 0]))]
        with mock.patch.object(executor_devices.events, "publish"), \
                mock.patch.object(executor_devices.events, "execute_and_wait_for"):
            self.dev.execute_table(table, 0, 2, 1, None)
        description = self.dev._last_profile[0]
        self.assertEqual(description["count"], 10)
        self.assertEqual(description["nDigital"], 2)
        self.assertEqual(description["nAnalog"], [2, 1, 1, 1])
        self.dev.connection.profileSet.assert_called_once()
        self.dev.connection.DownloadProfile.assert_called_once()


class TestActionsFromTable(unittest.TestCase):
    def test_slices_table_and_rebases_time(self):
        table = [(1.0, (1, [0, 0])), (2.0, (0, [5, 0])), (3.0, (0, [6, 0]))]
        actions = executor_devices.actions_from_table(table, 1, 3, None)
        self.assertEqual(actions, [(0.0, (0, [5, 0])), (1.0, (0, [6, 0]))])

    def test_rep_duration_appends_repeat_of_last_action(self):
        table = [(0.0, (1, [0, 0])), (1.0, (0, [5, 0]))]
        actions = executor_devices.actions_from_table(table, 0, 2, 3)
        self.assertEqual(len(actions), 3)
        self.assertEqual(actions[-1], (3.0, (0, [5, 0])))


class TestNIcRIOExperimentDone(unittest.TestCase):
    def test_experiment_done_publishes_executor_done(self):
        dev = ni_crio_fpga.NIcRIO.__new__(ni_crio_fpga.NIcRIO)
        dev.name = "cRIO"
        with mock.patch.object(ni_crio_fpga.events, "publish") as publish:
            dev.experiment_done()
        publish.assert_called_once_with(ni_crio_fpga.events.EXECUTOR_DONE % "cRIO")


class TestExecutorDeviceDebugMethods(unittest.TestCase):
    def setUp(self):
        self.dev = executor_devices.ExecutorDevice("executor", {})
        self.dev.connection = mock.MagicMock()

    def test_trigger_now_toggles_line_via_connection(self):
        self.dev.connection.ReadDigital.return_value = 5
        with mock.patch.object(executor_devices.time, "sleep") as sleep:
            self.dev.trigger_now(1, dt=0.02)
        sleep.assert_called_once_with(0.02)
        self.dev.connection.WriteDigital.assert_has_calls(
            [mock.call(4), mock.call(4)]
        )
        self.assertEqual(self.dev.connection.ReadDigital.call_count, 2)

    def test_set_digital_writes_value(self):
        self.dev.set_digital(7)
        self.dev.connection.WriteDigital.assert_called_once_with(7)


class TestNIcRIORunSequence(unittest.TestCase):
    def test_run_sequence_converts_ms_to_ticks(self):
        dev = ni_crio_fpga.NIcRIO.__new__(ni_crio_fpga.NIcRIO)
        dev.tickrate = 10
        dev.connection = mock.MagicMock()
        dev.run_sequence([(0.0, 1), (1.5, 0)])
        dev.connection.run_sequence_on_fpga.assert_called_once_with([(0, 1), (15, 0)])

    def test_connection_run_sequence_renamed_and_wire_key_unchanged(self):
        self.assertTrue(hasattr(ni_crio_fpga.Connection, "run_sequence_on_fpga"))
        self.assertFalse(hasattr(ni_crio_fpga.NIcRIO, "runSequence"))
        self.assertTrue(hasattr(ni_crio_fpga.NIcRIO, "run_sequence"))
        conn = ni_crio_fpga.Connection(None, "127.0.0.1", [1, 2], "127.0.0.1")
        self.assertEqual(conn.command_dict["runSequence"], 413)


class TestNIcRIOConnectionKeywords(unittest.TestCase):
    def test_initialize_passes_keyword_arguments_to_connection(self):
        dev = ni_crio_fpga.NIcRIO.__new__(ni_crio_fpga.NIcRIO)
        dev.ip_address = "10.0.0.2"
        dev.port = [5000, 6666]
        with mock.patch.object(ni_crio_fpga, "Connection") as conn_cls:
            ni_crio_fpga.NIcRIO.initialize(dev)
        conn_cls.assert_called_once_with(
            parent=dev,
            ip_address="10.0.0.2",
            port=[5000, 6666],
            local_ip=ni_crio_fpga.MASTER_IP,
        )
        conn_cls.return_value.connect.assert_called_once_with()

    def test_connection_stores_ip_address_and_local_ip(self):
        conn = ni_crio_fpga.Connection(
            None, ip_address="10.0.0.2", port=[5000, 6666], local_ip="10.0.0.1"
        )
        self.assertEqual(conn.ip_address, "10.0.0.2")
        self.assertEqual(conn.local_ip, "10.0.0.1")


class TestNIcRIOConnectionFPGACommands(unittest.TestCase):
    def setUp(self):
        self.device = mock.MagicMock()
        self.conn = ni_crio_fpga.Connection(
            self.device, "127.0.0.1", [5000, 0], "127.0.0.1"
        )
        self.conn.connection = mock.MagicMock()
        self.conn.connection.recv.return_value = b'{"status": false}'
        self.status = None

    def tearDown(self):
        if self.status is not None and self.status.socket is not None:
            self.status.socket.close()

    def _bind_status(self, current_fpga_status=None):
        self.status = ni_crio_fpga.FPGAStatus(self.conn, "127.0.0.1", 0)
        if current_fpga_status is not None:
            self.status.current_fpga_status = current_fpga_status
        self.conn.status = self.status
        return self.status

    def test_prepare_actions_sends_commands_in_order(self):
        description = {"count": 2}
        digitals = [(0, 1), (10, 0)]
        analogs = [[(0, 0), (10, 5)]]
        self.conn.PrepareActions([description, digitals, analogs], 3)
        commands = _json_commands_from_send(self.conn.connection)
        self.assertEqual(
            [item["Command"] for item in commands],
            [
                self.conn.command_dict["flushFIFOs"],
                self.conn.command_dict["sendDigitals"],
                self.conn.command_dict["sendAnalogues"],
                self.conn.command_dict["sendStartStopIndexes"],
                self.conn.command_dict["initProfile"],
            ],
        )
        self.assertEqual(commands[-1]["Number of Messages"], 2)
        init_buf = self.conn.connection.sendall.call_args_list[-1].args[0]
        self.assertEqual(
            init_buf, (str(3).rjust(20, "0") + str(0).rjust(20, "0")).encode()
        )

    def test_run_sequence_on_fpga_sends_code_413_with_packed_values(self):
        sequence = [(0, 1), (15, 0)]
        self.conn.run_sequence_on_fpga(sequence)
        commands = _json_commands_from_send(self.conn.connection)
        self.assertEqual(len(commands), 1)
        self.assertEqual(commands[0]["Command"], 413)
        self.assertEqual(
            commands[0]["Command"], self.conn.command_dict["runSequence"]
        )
        packed = [
            int(np.binary_repr(t, 32) + np.binary_repr(d, 32), 2)
            for t, d in sequence
        ]
        expected = "".join(str(value).rjust(20, "0") for value in packed).encode()
        self.conn.connection.sendall.assert_called_once_with(expected)

    def test_fpga_status_feeds_analog_digital_and_idle_reads(self):
        self._bind_status(
            {
                "Analogue 0": 42,
                "Analogue 1": 99,
                "Digitals": 5,
                "Action State": ni_crio_fpga.FPGA_IDLE_STATE,
                "Aborted": False,
            }
        )
        dev = ni_crio_fpga.NIcRIO.__new__(ni_crio_fpga.NIcRIO)
        dev.connection = self.conn
        self.assertEqual(dev.get_analog(0), 42)
        self.assertEqual(self.conn.ReadPosition(1), 99)
        self.assertEqual(self.conn.ReadDigital(), 5)
        self.assertEqual(self.conn.ReadDigital(1), 1)
        self.assertTrue(self.conn.is_idle())
        self.assertFalse(self.conn.is_aborted())
        self.status.current_fpga_status["Action State"] = 0
        self.status.current_fpga_status["Aborted"] = True
        self.assertFalse(self.conn.is_idle())
        self.assertTrue(self.conn.is_aborted())

    def test_publish_fpga_status_changes_done_event(self):
        self._bind_status()
        new_status = {"Event": "done"}
        result = self.status.publish_fpga_status_changes(new_status=new_status)
        self.device.experiment_done.assert_called_once_with()
        self.assertEqual(result["Event"], "")
        self.assertIs(result, new_status)


if __name__ == "__main__":
    unittest.main()
