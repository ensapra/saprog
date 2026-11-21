"""Shared live EEPROM limits, invalidation and port-owner routing."""

import json
import sys
import os
import tempfile
import unittest
import xml.etree.ElementTree as ET
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

from limit_fixtures import limits_message, prime_limits  # noqa: E402
from PySide6.QtCore import QObject, Signal  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402
from ros_manager.wsl_bridge.commands import RobotCommands  # noqa: E402
from servo_limits import get_servo_limits, hardware_report  # noqa: E402

_APP = QApplication.instance() or QApplication([])


class FakeManager(QObject):
    command_result = Signal(str, bool, str)
    message_received = Signal(str, object)
    status_changed = Signal(str, str)
    position_published = Signal(str, object)
    torque_release_requested = Signal()

    def __init__(self):
        super().__init__()
        self.request_topic = Mock(return_value="limits-state")
        self.refresh_servo_limits = Mock(side_effect=["read-1", "read-2", "read-3"])
        self.cancel_commands = Mock()
        self.release = Mock()
        self.publish_joint_command = Mock(return_value="move-1")


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

    def test_single_shared_read_is_read_only_and_has_no_disk_cache(self):
        manager = FakeManager()
        service = get_servo_limits(manager)
        self.assertIs(service, get_servo_limits(manager))
        bounds = service.bounds
        with patch("builtins.open", side_effect=AssertionError("disk cache access")):
            service.refresh()
            service.refresh()
            manager.refresh_servo_limits.assert_called_once()
            manager.command_result.emit("read-1", True, limits_message())
        self.assertEqual(len(bounds), 12)
        self.assertIs(bounds, service.bounds)
        with self.assertRaises(TypeError):
            bounds["servo_11"] = (-10, 10)
        service.close()

    def test_failure_invalidates_all_consumers_and_late_timeouts_are_ignored(self):
        manager = FakeManager()
        service = prime_limits(manager)
        bounds = service.bounds
        service.refresh()
        manager.command_result.emit("read-1", False, "disconnected")
        self.assertFalse(bounds)
        service.refresh()
        service.timed_out()
        manager.cancel_commands.assert_called_with(["read-2"])
        manager.command_result.emit("read-2", True, limits_message())
        self.assertFalse(bounds)
        service.refresh()
        manager.command_result.emit("read-3", True, limits_message(400, 600))
        self.assertEqual(len(bounds), 12)
        service.close()

    def test_animator_playback_and_leg_share_limits_and_receive_changes(self):
        from tabs.controller.playback import ControllerPlayback
        from tabs.leg_test.urdf_tab import SingleLegTestTab
        from widgets.player import StaticAnimatorWidget

        manager = FakeManager()
        animator = StaticAnimatorWidget(ros_manager=manager)
        playback = ControllerPlayback(manager, ".")
        leg = SingleLegTestTab(manager)
        self.assertIs(animator.limits, playback.limits)
        self.assertIs(playback.limits, leg.limits)
        self.assertIs(leg.model.bounds, playback.bounds)
        animator.refresh_servo_limits()
        manager.command_result.emit("read-1", True, limits_message())
        old_upper = playback.bounds["servo_13"][1]
        leg.limits.refresh()
        manager.command_result.emit("read-2", True, limits_message(400, 600))
        self.assertLess(playback.bounds["servo_13"][1], old_upper)
        self.assertIs(animator.creator_view.servo_limits, playback.bounds)
        self.assertNotIn("lower", leg.mapping_fields[2])
        self.assertIn("rad", leg.mapping_fields[2]["limits"].text())
        animator.close()
        leg.close()
        playback.close()
        animator.deleteLater()
        leg.deleteLater()
        playback.deleteLater()
        get_servo_limits(manager).close()

    def test_old_urdf_stops_do_not_override_hardware_bounds(self):
        from tabs.leg_test.urdf_model import DESCRIPTION, SingleLegModel

        manager = FakeManager()
        bounds = prime_limits(manager).bounds
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "old.urdf"
            tree = ET.parse(DESCRIPTION / "urdf/front_right.urdf")
            motor = tree.find("joint[@name='front_right_joint_13']/limit")
            motor.set("lower", ".26615")
            motor.set("upper", ".88034")
            tree.write(path)
            model = SingleLegModel(path, bounds=bounds)
            self.assertTrue(model.in_limits({"servo_11": 0, "servo_12": 0, "servo_13": .2}))
        get_servo_limits(manager).close()

    def test_disconnect_clears_limits_and_reconnect_reads_again(self):
        manager = FakeManager()
        service = get_servo_limits(manager)
        service.refresh()
        manager.command_result.emit("read-1", True, limits_message())
        manager.status_changed.emit("/saprog/joint_states", "Publisher unavailable")
        self.assertFalse(service.available)
        manager.status_changed.emit("/saprog/joint_states", "Connected")
        self.assertEqual(service.pending, "read-2")
        manager.command_result.emit("read-2", True, limits_message())
        self.assertTrue(service.available)
        service.close()

    def test_manager_blocks_unknown_limits_clamps_and_cancels_old_queued_positions(self):
        from ros_manager import RosDataManager

        manager = RosDataManager()
        with patch.object(manager, "_ensure_worker_locked"):
            with self.assertRaisesRegex(ValueError, "EEPROM limits are unavailable"):
                manager.publish_joint_command(["servo_11"], [100], 100, 300)
            pending = manager._drain_commands()
            reads = [command for command in pending if command["op"] == "refresh_servo_limits"]
            self.assertEqual(len(reads), 1)
            self.assertFalse(any(command["op"] == "publish_position" for command in pending))
            manager.command_result.emit(reads[0]["id"], True, limits_message())
            command_id = manager.publish_joint_command(["servo_11"], [100], 100, 300)
            command = manager._drain_commands()[0]
            self.assertEqual(command["positions"], [manager.servo_limits.bounds["servo_11"][1]])
            manager.refresh_servo_limits()
            read = manager._drain_commands()[0]
            manager.command_result.emit(read["id"], True, limits_message(400, 600))
            queued = manager._drain_commands()
            self.assertTrue(any(command["op"] == "cancel_commands" and command_id in command["ids"]
                                for command in queued))
        manager.close()

    def test_direction_and_zero_are_converted_once(self):
        from documents.snapshot_units import AX_TICK_RADIANS
        from servo_limits import report_bounds

        report = hardware_report(json.dumps({"servos": [self.reading()]}), ["servo_11"])
        lower, upper = report_bounds(report)["servo_11"]
        self.assertAlmostEqual(lower, -88*AX_TICK_RADIANS)
        self.assertAlmostEqual(upper, 212*AX_TICK_RADIANS)


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
