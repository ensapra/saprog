"""Hardware limit validation and cache replacement without ROS or servos."""

import json
import sys
import tempfile
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ros_manager.wsl_bridge.commands import RobotCommands  # noqa: E402
from widgets.player.servo_limits import hardware_report, save_report  # noqa: E402


class ServoLimitsTest(unittest.TestCase):
    def reading(self, **changes):
        return dict(servo_id=11, zero_position=512, direction=-1,
                    raw_lower=300, raw_upper=600, **changes)

    def test_rejects_bad_or_partial_hardware_responses(self):
        valid = self.reading()
        bad_entries = [[], [valid, valid], [dict(valid, direction=0)],
                       [dict(valid, raw_lower=600)], [dict(valid, raw_upper=1024)],
                       [dict(valid, raw_lower=True)], [dict(valid, zero_position=-1)]]
        for entries in bad_entries:
            with self.subTest(entries=entries), self.assertRaises(ValueError):
                hardware_report(json.dumps({"servos": entries}), ["servo_11"])
        for message in ('{}', 'null', '{"servos":null}', 'not json'):
            with self.subTest(message=message), self.assertRaises(ValueError):
                hardware_report(message, ["servo_11"])

    def test_atomic_save_preserves_metadata_and_previous_file_on_failure(self):
        report = hardware_report(json.dumps({"servos": [self.reading()]}), ["servo_11"])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "limits.json"
            original = json.dumps({"knee_mimic_multiplier": 2.4, "servos": [
                {"servo_id": 11, "urdf_joint": "front_right_11", "raw_lower": 0},
            ]})
            path.write_text(original, encoding="utf-8")
            with patch("widgets.player.servo_limits.os.replace", side_effect=OSError("busy")):
                with self.assertRaises(OSError):
                    save_report(report, path)
            self.assertEqual(path.read_text(encoding="utf-8"), original)
            self.assertEqual(list(Path(directory).glob("*.tmp")), [])
            save_report(report, path)
            saved = json.loads(path.read_text(encoding="utf-8"))
            self.assertEqual(saved["servos"][0]["raw_lower"], 300)
            self.assertEqual(saved["servos"][0]["urdf_joint"], "front_right_11")
            self.assertEqual(saved["knee_mimic_multiplier"], 2.4)


class ServoLimitsBridgeTest(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.client = self.node.create_client.return_value
        self.future = Future()
        self.client.call_async.return_value = self.future
        self.commands = RobotCommands(self.node, Mock(), Mock(), SimpleNamespace(Request=Mock()))
        self.event_patch = patch("ros_manager.wsl_bridge.commands.emit")
        self.emit = self.event_patch.start()
        self.addCleanup(self.event_patch.stop)

    def test_reads_through_port_owner_and_forwards_hardware_response(self):
        self.assertTrue(self.commands.handle({"op": "refresh_servo_limits", "id": "read"}))
        self.commands.process_pending()
        self.node.create_client.assert_called_once_with(
            self.commands.trigger_type, "/saprog/read_servo_limits",
        )
        self.assertEqual(self.commands.pending_commands, [])
        response = SimpleNamespace(success=True, message='{"servos":[]}')
        self.future.set_result(response)
        self.emit.assert_called_once_with(
            "command_result", id="read", success=True, message=response.message,
        )

    def test_unavailable_service_retries_then_reports_failure(self):
        self.client.service_is_ready.return_value = False
        with patch("ros_manager.wsl_bridge.commands.time.monotonic", return_value=10):
            self.commands.handle({"op": "refresh_servo_limits", "id": "read"})
            self.commands.process_pending()
        self.assertEqual(len(self.commands.pending_commands), 1)
        with patch("ros_manager.wsl_bridge.commands.time.monotonic", return_value=41):
            self.commands.process_pending()
        self.assertEqual(self.commands.pending_commands, [])
        self.client.call_async.assert_not_called()
        self.emit.assert_called_once_with(
            "command_result", id="read", success=False,
            message="Servo limit service is unavailable",
        )

    def test_device_failure_is_not_reported_as_success(self):
        self.commands.handle({"op": "refresh_servo_limits", "id": "read"})
        self.commands.process_pending()
        self.future.set_result(SimpleNamespace(success=False, message="servo_11: disconnected"))
        self.emit.assert_called_once_with(
            "command_result", id="read", success=False, message="servo_11: disconnected",
        )
