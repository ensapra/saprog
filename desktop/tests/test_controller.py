"""Offscreen controller tests: no physical gamepad, ROS, WSL or robot required."""

import json
import math
import os
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtCore import QEvent, QObject, QSettings, Signal  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402
from ros_manager import RosDataManager  # noqa: E402
from ros_manager.ros_worker import RosWorker  # noqa: E402
from ros_manager.wsl_bridge import cancel_pending_commands, publish_position_command  # noqa: E402
from tabs.controller import ControllerTab  # noqa: E402
from tabs.controller.devices import ControllerMonitor  # noqa: E402
from tabs.controller.playback import ControllerPlayback  # noqa: E402
from widgets.player import StaticAnimatorWidget  # noqa: E402

_APP = QApplication.instance() or QApplication([])


def dispose(obj):
    """Destroy Qt test objects while their event dispatcher is still alive."""
    obj.deleteLater()
    _APP.sendPostedEvents(None, QEvent.Type.DeferredDelete)


class FakeManager(QObject):
    command_result = Signal(str, bool, str)
    position_published = Signal(str, object)
    message_received = Signal(str, object)
    torque_release_requested = Signal()

    def __init__(self):
        super().__init__()
        self.commands = []
        self.cancelled = []
        self.released = []

    def request_topic(self, *args, **kwargs):
        return "state-token"

    def release(self, token):
        self.released.append(token)

    def _send(self, kind, *args):
        command_id = str(len(self.commands))
        self.commands.append((command_id, kind, args))
        return command_id

    def publish_joint_command(self, *args):
        return self._send("position", *args)

    def set_servo_torque(self, *args):
        return self._send("torque", *args)

    def release_all_torque(self):
        self.torque_release_requested.emit()
        return self._send("release_all")

    def cancel_commands(self, ids):
        self.cancelled.extend(ids)

    def acknowledge(self, success=True):
        if success and self.commands[-1][1] == "position":
            self.position_published.emit(self.commands[-1][0], time.time_ns())
        self.command_result.emit(self.commands[-1][0], success, "result")


class FakeMonitor(QObject):
    devices_changed = Signal(object)
    selected_changed = Signal(object)
    inputs_changed = Signal(object)
    status_changed = Signal(str)

    def start(self):
        pass

    def refresh(self):
        pass

    def select(self, instance):
        self.inputs_changed.emit(set())
        self.selected_changed.emit({
            "id": instance, "name": "Test gamepad", "guid": f"guid-{instance}",
            "controls": [(f"button:{i}", f"Button {i + 1}") for i in range(3)],
        } if instance is not None else None)

    def close(self):
        self.select(None)


class ControllerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = _APP

    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.directory = Path(self.temp.name)
        self.snapshots = self.directory / "snapshots"
        self.sequences = self.directory / "sequences"
        self.snapshots.mkdir()
        self.sequences.mkdir()
        self.pose = self.snapshots / "pose.json"
        self.pose.write_text(json.dumps({
            "schema_version": 1, "name": "pose", "positions": {"servo_11": 0.0},
        }), encoding="utf-8")
        self.sequence = self.sequences / "sequence.json"
        self.motion = {
            "snapshot": "pose.json", "motion": {"speed": 100, "torque": 700},
            "duration": {"mode": "seconds", "seconds": 0.25},
        }
        self.write_sequence([self.motion])
        self.manager = FakeManager()
        self.addCleanup(dispose, self.manager)
        self.now = 100.0
        self.playback = ControllerPlayback(
            self.manager, self.snapshots, clock=lambda: self.now,
        )
        self.addCleanup(dispose, self.playback)
        self.addCleanup(self.playback.close)

    def write_sequence(self, steps):
        self.sequence.write_text(json.dumps({
            "kind": "saprog_timeline", "schema_version": 2, "steps": steps,
            "loop": False, "on_end": "off",
        }), encoding="utf-8")

    def make_tab(self, monitor=None):
        tab = ControllerTab(
            ros_manager=self.manager, monitor=monitor or FakeMonitor(),
            settings=QSettings(str(self.directory / "settings.ini"), QSettings.Format.IniFormat),
            snapshot_directory=self.snapshots, sequence_directory=self.sequences,
        )
        self.addCleanup(dispose, tab)
        self.addCleanup(tab.close)
        return tab

    def map(self, tab, key, kind, path=None):
        _label, action, target = tab.rows[key]
        action.setCurrentIndex(action.findData(kind))
        if path is not None:
            target.setCurrentIndex(target.findData(str(path.resolve())))

    def test_sequence_loops_while_held_regardless_of_file_flags(self):
        self.playback.start("button:0", self.sequence)
        self.assertEqual(self.manager.commands[-1][1:], ("position", (["servo_11"], [0.0], 100, 700)))
        self.manager.acknowledge()
        self.now += 0.26
        self.playback.tick()
        self.assertEqual(len(self.manager.commands), 2)
        self.assertTrue(all(command[1] == "position" for command in self.manager.commands))
        self.playback.release("button:0")
        self.manager.acknowledge()
        self.now += 100
        self.playback.tick()
        self.assertEqual(len(self.manager.commands), 2)
        self.assertIn("1", self.manager.cancelled)
        self.assertIsNone(self.playback.owner)

    def test_smooth_timed_playback_scales_speed_and_handles_loop_boundary(self):
        other = self.snapshots / "other.json"
        other.write_text(json.dumps({
            "schema_version": 1, "name": "other", "positions": {"servo_11": 0.1},
        }), encoding="utf-8")
        self.write_sequence([self.motion, dict(self.motion, snapshot="other.json")])
        self.playback.start("button:0", self.sequence, smooth=True)
        self.assertEqual(self.manager.commands[-1][2][2], 100)
        self.manager.acknowledge()
        self.now += 0.25
        self.playback.tick()
        expected = min(100, max(1, math.ceil(
            0.1 / 0.25 / self.playback.SPEED_UNIT_RADIANS
        )))
        self.assertEqual(self.manager.commands[-1][2][2], expected)
        self.assertEqual(self.manager.commands[-1][2][3], 700)
        self.manager.acknowledge()
        self.now += 0.25
        self.playback.tick()
        self.assertEqual(self.manager.commands[-1][2][2], expected)

    def test_controller_uses_saved_smooth_setting(self):
        document = json.loads(self.sequence.read_text(encoding="utf-8"))
        document["smooth_playback"] = True
        self.sequence.write_text(json.dumps(document), encoding="utf-8")
        self.playback.start("button:0", self.sequence)
        self.assertTrue(self.playback.smooth)

    def test_smooth_mode_falls_back_after_wait_and_for_arrival_steps(self):
        self.write_sequence([
            self.motion, {"kind": "wait", "seconds": 0.1},
            self.motion, dict(self.motion, duration={"mode": "until_reached"}),
        ])
        self.playback.start("button:0", self.sequence, repeat=False, smooth=True)
        self.manager.acknowledge()
        self.now += 0.25
        self.playback.tick()
        self.now += 0.1
        self.playback.tick()
        self.assertEqual(self.manager.commands[-1][2][2], 100)
        self.manager.acknowledge()
        self.now += 0.25
        self.playback.tick()
        self.assertEqual(self.manager.commands[-1][2][2], 100)

    def test_release_before_ack_cannot_schedule_later_steps(self):
        self.playback.start("button:0", self.sequence)
        self.playback.release("button:0")
        self.manager.acknowledge()
        self.now += 1
        self.playback.tick()
        self.assertEqual(len(self.manager.commands), 1)
        self.assertEqual(self.playback.phase, "idle")

    def test_one_shot_runs_all_steps_after_release_and_never_loops(self):
        self.write_sequence([
            {"kind": "wait", "seconds": 1}, self.motion,
            {"kind": "wait", "seconds": 2}, self.motion,
        ])
        document = json.loads(self.sequence.read_text(encoding="utf-8"))
        document["loop"] = True
        self.sequence.write_text(json.dumps(document), encoding="utf-8")
        messages = []
        self.playback.status_changed.connect(messages.append)
        self.playback.start("button:0", self.sequence, repeat=False)
        self.playback.release("button:0")
        self.assertEqual(self.playback.owner, "button:0")
        self.assertFalse(self.manager.commands)
        self.now += 1
        self.playback.tick()
        self.playback.release("button:0")
        self.assertEqual(self.playback.phase, "publishing")
        self.assertFalse(self.manager.cancelled)
        self.manager.acknowledge()
        self.now += 0.25
        self.playback.tick()
        self.now += 1
        self.playback.tick()
        self.assertEqual(len(self.manager.commands), 1)
        self.now += 1
        self.playback.tick()
        self.assertEqual(len(self.manager.commands), 2)
        self.manager.acknowledge()
        self.playback.release("button:0")
        self.playback.tick()
        self.assertEqual(self.playback.owner, "button:0")
        self.now += 0.25
        self.playback.tick()
        self.assertIsNone(self.playback.owner)
        self.assertFalse(self.playback.timer.isActive())
        self.assertEqual(messages[-1], "Sequence finished")
        self.now += 100
        self.playback.tick()
        self.assertEqual(len(self.manager.commands), 2)
        self.assertTrue(all(command[1] == "position" for command in self.manager.commands))

    def test_one_shot_waits_for_arrival_after_release(self):
        self.write_sequence([dict(self.motion, duration={"mode": "until_reached"})])
        self.playback.start("button:0", self.sequence, repeat=False)
        self.manager.acknowledge()
        self.playback.release("button:0")
        self.now += 1
        self.playback.tick()
        self.assertEqual(self.playback.phase, "arrival")
        stamp = self.playback.sent_at
        with patch("tabs.controller.playback.time.time_ns", return_value=stamp + 1000):
            for offset in (1, 2, 3):
                timestamp = stamp + offset
                self.playback.observe(self.playback.STATE_TOPIC, {
                    "_ros_received_at_ns": timestamp + 1000,
                    "header": {"stamp": {"sec": timestamp // 1_000_000_000,
                                          "nanosec": timestamp % 1_000_000_000}},
                    "name": ["servo_11"], "position": [0.0],
                })
        self.playback.tick()
        self.assertIsNone(self.playback.owner)
        self.assertEqual(len(self.manager.commands), 1)

    def test_latest_press_owns_sequence_and_old_release_does_not_stop_it(self):
        self.playback.start("button:0", self.sequence)
        self.playback.start("button:1", self.sequence)
        self.playback.release("button:0")
        self.manager.command_result.emit("0", True, "late")
        self.assertEqual(self.playback.owner, "button:1")
        self.assertEqual(self.playback.phase, "publishing")

    def test_external_release_stops_each_sequence_phase_without_restarting_while_held(self):
        manager = RosDataManager()
        self.addCleanup(dispose, manager)
        playback = ControllerPlayback(manager, self.snapshots, clock=lambda: self.now)
        self.addCleanup(dispose, playback)
        self.addCleanup(playback.close)
        with patch.object(manager, "_ensure_worker_locked"):
            for phase in ("publishing", "wait", "arrival", "initial_wait"):
                with self.subTest(phase=phase):
                    duration = {"mode": "until_reached"} if phase == "arrival" else {
                        "mode": "seconds", "seconds": 1,
                    }
                    steps = [dict(self.motion, duration=duration)]
                    if phase == "initial_wait":
                        steps.insert(0, {"kind": "wait", "seconds": 5})
                    self.write_sequence(steps)
                    playback.start("button:0", self.sequence)
                    commands = manager._drain_commands()
                    if phase in ("wait", "arrival"):
                        manager.command_result.emit(commands[0]["id"], True, "Published")
                    release_id = manager.release_all_torque()
                    manager.command_result.emit(release_id, True, "Torque released")
                    for command in commands:
                        manager.command_result.emit(command["id"], True, "Late publication")
                    self.now += 40
                    playback.tick()
                    self.assertIsNone(playback.owner)
                    self.assertFalse(playback.timer.isActive())
                    remaining = manager._drain_commands()
                    self.assertFalse(any(item["op"] == "publish_position" for item in remaining))
                    # A new press may start again after the global stop.
                    playback.start("button:1", self.sequence)
                    self.assertEqual(playback.owner, "button:1")
                    playback.stop()
                    manager._drain_commands()

    def test_wait_step_can_be_interrupted(self):
        self.write_sequence([{"kind": "wait", "seconds": 5}, self.motion])
        self.playback.start("button:0", self.sequence)
        self.assertFalse(self.manager.commands)
        self.playback.release("button:0")
        self.now += 10
        self.playback.tick()
        self.assertFalse(self.manager.commands)

    def test_motion_groups_are_acknowledged_before_more_commands(self):
        self.pose.write_text(json.dumps({
            "schema_version": 1, "name": "pose",
            "positions": {"servo_11": 0.0, "servo_12": 0.1},
        }), encoding="utf-8")
        self.write_sequence([dict(self.motion, overrides={"servo_12": {"speed": 20, "torque": 30}})])
        self.playback.start("button:0", self.sequence)
        self.assertEqual(len(self.manager.commands), 1)
        self.manager.acknowledge()
        self.assertEqual(len(self.manager.commands), 2)
        self.assertEqual(self.manager.commands[-1][2], (["servo_12"], [0.1], 20, 30))
        self.playback.release("button:0")
        self.manager.acknowledge()
        self.assertIsNone(self.playback.owner)

    def test_until_reached_needs_fresh_complete_stable_telemetry(self):
        self.write_sequence([dict(self.motion, duration={"mode": "until_reached"})])
        self.playback.start("button:0", self.sequence)
        self.manager.acknowledge()
        stamp = time.time_ns()

        def sample(timestamp, position=0.0, names=None):
            return {
                "_ros_received_at_ns": timestamp + 1000,
                "header": {"stamp": {"sec": timestamp // 1_000_000_000,
                                      "nanosec": timestamp % 1_000_000_000}},
                "name": names or ["servo_11"], "position": [position],
            }

        with patch("tabs.controller.playback.time.time_ns", return_value=stamp + 1000):
            for _ in range(4):
                self.playback.observe(self.playback.STATE_TOPIC, sample(self.playback.sent_at - 1))
            self.assertEqual(self.playback.stable_samples, 0)
            self.playback.observe(self.playback.STATE_TOPIC, sample(stamp, names=["servo_12"]))
            for offset in (1, 2, 3):
                self.playback.observe(self.playback.STATE_TOPIC, sample(stamp + offset))
            self.assertEqual(self.playback.phase, "wait")
        self.playback.tick()
        self.assertEqual(len(self.manager.commands), 2)

    def test_arrival_uses_ros_clock_and_controller_tolerance(self):
        self.write_sequence([dict(self.motion, duration={"mode": "until_reached"})])
        for windows_offset in (-600_000_000_000, 600_000_000_000):
            with self.subTest(windows_offset=windows_offset):
                self.playback.start("button:0", self.sequence, repeat=False)
                command_id = self.manager.commands[-1][0]
                ros_stamp = 10_000_000_000
                self.manager.position_published.emit(command_id, ros_stamp)
                self.manager.command_result.emit(command_id, True, "Published")
                with patch("tabs.controller.playback.time.time_ns", return_value=ros_stamp + windows_offset):
                    for offset in (1, 2, 3):
                        stamp = ros_stamp + offset
                        self.playback.observe(self.playback.STATE_TOPIC, {
                            "_ros_received_at_ns": stamp + 1000,
                            "header": {"stamp": {"sec": stamp // 1_000_000_000,
                                                  "nanosec": stamp % 1_000_000_000}},
                            "name": ["servo_11"], "position": [0.06],
                        })
                self.playback.tick()
                self.assertIsNone(self.playback.owner)

    def test_arrival_rejects_stale_duplicate_and_out_of_tolerance_samples(self):
        self.write_sequence([dict(self.motion, duration={"mode": "until_reached"})])
        self.playback.start("button:0", self.sequence)
        self.manager.acknowledge()
        barrier = self.playback.sent_at

        def observe(stamp, position=0.0, age=1000):
            self.playback.observe(self.playback.STATE_TOPIC, {
                "_ros_received_at_ns": stamp + age,
                "header": {"stamp": {"sec": stamp // 1_000_000_000,
                                      "nanosec": stamp % 1_000_000_000}},
                "name": ["servo_11"], "position": [position],
            })

        observe(barrier - 1)
        observe(barrier + 1, age=3_000_000_000)
        observe(barrier + 2, age=-1)
        self.assertEqual(self.playback.stable_samples, 0)
        observe(barrier + 3)
        observe(barrier + 3)
        self.assertEqual(self.playback.stable_samples, 1)
        observe(barrier + 4, position=0.11)
        self.assertEqual(self.playback.stable_samples, 0)
        self.assertEqual(self.playback.phase, "arrival")

    def test_stop_returns_manual_control_in_every_phase_and_ignores_late_events(self):
        manager = RosDataManager()
        self.addCleanup(dispose, manager)
        playback = ControllerPlayback(manager, self.snapshots, clock=lambda: self.now)
        self.addCleanup(dispose, playback)
        self.addCleanup(playback.close)
        with patch.object(manager, "_ensure_worker_locked"):
            for phase in ("publishing", "wait", "arrival", "initial_wait"):
                with self.subTest(phase=phase):
                    step = dict(self.motion, duration={"mode": "until_reached"}) if phase == "arrival" else self.motion
                    steps = [{"kind": "wait", "seconds": 100}, step] if phase == "initial_wait" else [step]
                    playback.start_steps("timeline", steps, repeat=True)
                    sent = manager._drain_commands() if phase in ("wait", "arrival") else []
                    for command in sent:
                        manager.position_published.emit(command["id"], 1)
                        manager.command_result.emit(command["id"], True, "Published")
                    pending = set(playback.pending)
                    playback.stop("Timeline stopped")
                    manual_id = manager.publish_joint_command(["servo_11"], [0.1], 512, 1023)
                    for command_id in pending | {cmd["id"] for cmd in sent}:
                        manager.position_published.emit(command_id, 2)
                        manager.command_result.emit(command_id, True, "Late result")
                    self.now += 101
                    playback.tick()
                    commands = manager._drain_commands()
                    positions = [cmd for cmd in commands if cmd["op"] == "publish_position"]
                    self.assertEqual([cmd["id"] for cmd in positions], [manual_id])
                    self.assertEqual(positions[0]["speed"], 512)
                    self.assertFalse(playback.timer.isActive())
                    self.assertIsNone(playback.owner)
                    self.assertEqual(playback.targets, {})

    def test_failed_ack_or_timeout_stops_loop(self):
        self.playback.start("button:0", self.sequence)
        self.manager.acknowledge(False)
        self.assertIsNone(self.playback.owner)
        self.playback.start("button:0", self.sequence)
        self.now += 31
        self.playback.tick()
        self.assertIsNone(self.playback.owner)
        self.assertIn("1", self.manager.cancelled)

    def test_invalid_snapshot_prevents_any_commands(self):
        for value in (float("nan"), 100):
            self.pose.write_text(json.dumps({
                "schema_version": 1, "name": "pose", "positions": {"servo_11": value},
            }), encoding="utf-8")
            with self.assertRaises(ValueError):
                self.playback.start("button:0", self.sequence)
        self.assertFalse(self.manager.commands)

    def test_torque_off_preempts_and_uses_manager_release(self):
        self.playback.start("button:0", self.sequence)
        self.playback.torque(False)
        self.assertIsNone(self.playback.owner)
        self.assertEqual(self.manager.commands[-1][1], "release_all")
        self.assertEqual(self.manager.commands[-1][2], ())

    def test_snapshot_and_torque_on_use_shared_manager(self):
        self.playback.snapshot(self.pose, 99, 333)
        self.assertEqual(self.manager.commands[-1][2], (["servo_11"], [0.0], 99, 333))
        self.playback.torque(True)
        self.assertEqual(len([cmd for cmd in self.manager.commands if cmd[1] == "torque"]), 12)
        self.playback.close()
        self.assertIn("state-token", self.manager.released)

    def test_unsupported_snapshot_units_send_no_commands(self):
        self.pose.write_text(json.dumps({
            "schema_version": 2, "units": "normalized", "name": "pose",
            "positions": {"servo_11": 0.0},
        }), encoding="utf-8")
        self.playback.bounds = {"servo_11": (-0.2, 0.6)}
        with self.assertRaisesRegex(ValueError, "must use radians"):
            self.playback.snapshot(self.pose)
        self.assertFalse(self.manager.commands)

    def test_selection_shows_all_buttons_and_persists_mapping_per_guid(self):
        tab = self.make_tab()
        tab.monitor.devices_changed.emit([
            {"id": 1, "name": "One", "guid": "guid-1"},
            {"id": 2, "name": "Two", "guid": "guid-2"},
        ])
        self.assertEqual(len(tab.rows), 3)
        self.map(tab, "button:0", "snapshot", self.pose)
        tab.device_combo.setCurrentIndex(2)
        self.assertEqual(tab._mapping("button:0"), {})
        tab.device_combo.setCurrentIndex(1)
        self.assertEqual(tab._mapping("button:0")["kind"], "snapshot")
        second = self.make_tab()
        second.monitor.select(1)
        self.assertEqual(second.rows["button:0"][1].currentData(), "snapshot")
        self.assertEqual(second.rows["button:0"][2].currentData(), str(self.pose.resolve()))

    def test_input_edges_highlight_and_do_not_retrigger_held_snapshots(self):
        tab = self.make_tab()
        tab.monitor.select(1)
        self.map(tab, "button:0", "snapshot", self.pose)
        tab.monitor.inputs_changed.emit(set())
        for _ in range(3):
            tab.monitor.inputs_changed.emit({"button:0"})
        self.assertEqual(len(self.manager.commands), 1)
        self.assertIn("#34d399", tab.rows["button:0"][0].styleSheet())
        tab.monitor.inputs_changed.emit(set())
        self.assertEqual(tab.rows["button:0"][0].styleSheet(), "")

    def test_one_shot_mapping_persists_and_runs_once_per_press(self):
        tab = self.make_tab()
        tab.monitor.select(1)
        self.map(tab, "button:0", "sequence_once", self.sequence)
        self.assertTrue(tab.rows["button:0"][2].isEnabled())
        tab.close()
        tab = self.make_tab()
        tab.monitor.select(1)
        self.assertEqual(tab.rows["button:0"][1].currentData(), "sequence_once")
        self.assertEqual(tab.rows["button:0"][2].currentData(), str(self.sequence.resolve()))
        tab.playback.clock = lambda: self.now
        tab.monitor.inputs_changed.emit(set())
        tab.monitor.inputs_changed.emit({"button:0"})
        self.manager.acknowledge()
        self.now += 0.25
        tab.playback.tick()
        self.assertIsNone(tab.playback.owner)
        tab.monitor.inputs_changed.emit({"button:0"})
        self.assertEqual(len(self.manager.commands), 1)
        tab.monitor.inputs_changed.emit(set())
        tab.monitor.inputs_changed.emit({"button:0"})
        tab.monitor.inputs_changed.emit(set())
        self.assertEqual(tab.playback.owner, "button:0")
        self.assertEqual(len(self.manager.commands), 2)

    def test_one_shot_can_be_interrupted(self):
        tab = self.make_tab()
        for action in ("stop", "torque_off", "snapshot", "disconnect", "mapping", "folder"):
            with self.subTest(action=action):
                tab.monitor.select(1)
                tab.folders["sequence"].setText(str(self.sequences))
                tab.refresh_files()
                self.map(tab, "button:0", "sequence_once", self.sequence)
                if action in ("torque_off", "snapshot"):
                    self.map(tab, "button:1", action, self.pose if action == "snapshot" else None)
                tab.monitor.inputs_changed.emit(set())
                tab.monitor.inputs_changed.emit({"button:0"})
                command_id = self.manager.commands[-1][0]
                if action == "stop":
                    tab._stop()
                elif action in ("torque_off", "snapshot"):
                    tab.monitor.inputs_changed.emit({"button:1"})
                elif action == "disconnect":
                    tab.monitor.select(None)
                elif action == "mapping":
                    self.map(tab, "button:0", "sequence", self.sequence)
                else:
                    tab.folders["sequence"].setText(str(self.directory))
                    tab.refresh_files()
                self.assertIsNone(tab.playback.owner)
                self.assertIn(command_id, self.manager.cancelled)
                self.manager.command_result.emit(command_id, True, "Late publication")
                self.assertEqual(tab.playback.phase, "idle")

    def test_initially_held_input_must_be_released_before_activation(self):
        tab = self.make_tab()
        tab.monitor.select(1)
        self.map(tab, "button:0", "sequence", self.sequence)
        tab.monitor.inputs_changed.emit({"button:0"})
        tab.monitor.inputs_changed.emit({"button:0"})
        self.assertFalse(self.manager.commands)
        tab.monitor.inputs_changed.emit(set())
        tab.monitor.inputs_changed.emit({"button:0"})
        self.assertEqual(len(self.manager.commands), 1)

    def test_disconnect_and_mapping_change_stop_held_sequence(self):
        tab = self.make_tab()
        tab.monitor.select(1)
        self.map(tab, "button:0", "sequence", self.sequence)
        tab.monitor.inputs_changed.emit(set())
        tab.monitor.inputs_changed.emit({"button:0"})
        self.map(tab, "button:1", "torque_on")
        self.assertIsNone(tab.playback.owner)
        tab.monitor.inputs_changed.emit(set())
        tab.monitor.inputs_changed.emit({"button:0"})
        tab.monitor.select(None)
        self.assertIsNone(tab.playback.owner)
        self.assertEqual(tab.rows, {})

    def test_simultaneous_torque_off_wins_over_snapshot(self):
        tab = self.make_tab()
        tab.monitor.select(1)
        self.map(tab, "button:0", "torque_off")
        self.map(tab, "button:1", "snapshot", self.pose)
        tab.monitor.inputs_changed.emit(set())
        tab.monitor.inputs_changed.emit({"button:0", "button:1"})
        self.assertEqual([cmd[1] for cmd in self.manager.commands], ["release_all"])

    def test_missing_file_is_visible_and_never_sends(self):
        tab = self.make_tab()
        tab.monitor.select(1)
        self.map(tab, "button:0", "snapshot", self.pose)
        self.pose.unlink()
        tab.refresh_files()
        self.assertTrue(tab.rows["button:0"][2].currentText().startswith("Missing:"))
        tab.monitor.inputs_changed.emit(set())
        tab.monitor.inputs_changed.emit({"button:0"})
        self.assertFalse(self.manager.commands)
        self.assertIn("failed", tab.status_label.text())

    def test_refresh_does_not_interrupt_sequence_unless_source_folder_changes(self):
        tab = self.make_tab()
        tab.monitor.select(1)
        self.map(tab, "button:0", "sequence", self.sequence)
        tab.monitor.inputs_changed.emit(set())
        tab.monitor.inputs_changed.emit({"button:0"})
        tab.refresh_files()
        self.assertEqual(tab.playback.owner, "button:0")
        tab.folders["snapshot"].setText(str(self.directory))
        tab.refresh_files()
        self.assertIsNone(tab.playback.owner)


class CancellationTest(unittest.TestCase):
    def test_torque_off_matches_real_manager_api(self):
        manager = RosDataManager()
        playback = ControllerPlayback(manager, Path.cwd() / "snapshots")
        with patch.object(manager, "_ensure_worker_locked"):
            playback.torque(False)
            self.assertEqual(manager._drain_commands()[0]["op"], "release_all_torque")
            playback.close()
        dispose(playback)
        dispose(manager)

    def test_manager_cancels_only_requested_commands_and_forwards_to_wsl(self):
        manager = RosDataManager()
        results = []
        manager.command_result.connect(lambda *args: results.append(args))
        with patch.object(manager, "_ensure_worker_locked"):
            first = manager.publish_joint_command(["servo_11"], [0], 100, 100)
            other = manager.set_servo_torque("servo_12", True)
        manager.cancel_commands([first])
        remaining = manager._drain_commands()
        self.assertEqual(remaining[0]["id"], other)
        self.assertEqual(remaining[1], {"op": "cancel_commands", "ids": [first]})
        self.assertEqual([item[:2] for item in results], [(first, False)])
        dispose(manager)

    def test_release_all_reports_every_locally_discarded_command_outside_lock(self):
        manager = RosDataManager()
        self.addCleanup(dispose, manager)
        results, unlocked = [], []

        def result(command_id, success, message):
            results.append((command_id, success))
            acquired = manager._command_lock.acquire(blocking=False)
            unlocked.append(acquired)
            if acquired:
                manager._command_lock.release()

        manager.command_result.connect(result)
        with patch.object(manager, "_ensure_worker_locked"):
            first = manager.publish_joint_command(["servo_11"], [0], 100, 100)
            second = manager.set_servo_torque("servo_12", False)
            # An earlier cancellation marker has no command ID to acknowledge.
            manager.cancel_commands(["already-dispatched"])
            release_id = manager.release_all_torque()
        self.assertCountEqual(results, [(first, False), (second, False)])
        self.assertTrue(all(unlocked))
        self.assertEqual(manager._drain_commands(), [{"op": "release_all_torque", "id": release_id}])

    def test_controller_release_clears_static_animator_pending_puppet_controls(self):
        manager = RosDataManager()
        self.addCleanup(dispose, manager)
        with tempfile.TemporaryDirectory() as directory:
            with patch.object(manager, "_ensure_worker_locked"):
                widget = StaticAnimatorWidget(directory=directory, ros_manager=manager)
                playback = ControllerPlayback(manager, directory)
                try:
                    button = widget.creator_view.puppet_buttons["servo_11"]
                    button.click()
                    self.assertFalse(button.isEnabled())
                    playback.torque(False)
                    queued = manager._drain_commands()
                    release = next(item for item in queued if item["op"] == "release_all_torque")
                    manager.command_result.emit(release["id"], True, "Torque released")
                    self.assertEqual(widget._pending_commands, {})
                    self.assertTrue(button.isEnabled())
                finally:
                    playback.close()
                    widget.close()
                    dispose(widget)
                    dispose(playback)

    def test_bridge_cancels_only_unpublished_requested_commands(self):
        pending = [{"id": "controller"}, {"id": "other-widget"}]
        with patch("ros_manager.wsl_bridge.commands.emit") as emit:
            cancel_pending_commands(pending, ["controller", "already-published"])
        self.assertEqual(pending, [{"id": "other-widget"}])
        self.assertEqual(emit.call_count, 1)
        self.assertEqual(emit.call_args.kwargs["id"], "controller")


class PublisherMatchingTest(unittest.TestCase):
    def setUp(self):
        self.publisher = Mock()
        self.publisher.get_subscription_count.return_value = 0
        self.node = Mock()
        self.node.create_publisher.return_value = self.publisher
        # Graph discovery may already report a subscriber before writer matching.
        self.node.count_subscribers.return_value = 1
        self.node.get_clock.return_value.now.return_value.to_msg.return_value = SimpleNamespace(
            sec=12, nanosec=345,
        )
        self.command = {
            "id": "first-pose", "_queued_at": 100, "joints": ["servo_11"],
            "positions": [0.1], "speed": 100, "torque": 700,
        }

    @staticmethod
    def message_type():
        return SimpleNamespace(header=SimpleNamespace(stamp=None))

    def test_first_pose_stays_pending_until_its_publisher_matches(self):
        with patch("ros_manager.wsl_bridge.commands.emit") as emit, patch(
            "ros_manager.wsl_bridge.commands.time.monotonic", return_value=100,
        ):
            publisher, completed = publish_position_command(
                self.node, None, self.command, self.message_type,
            )
            self.assertFalse(completed)
            self.publisher.publish.assert_not_called()
            emit.assert_not_called()
            self.publisher.get_subscription_count.return_value = 1
            publisher, completed = publish_position_command(
                self.node, publisher, self.command, self.message_type,
            )
            self.assertTrue(completed)
            self.node.create_publisher.assert_called_once()
            self.publisher.publish.assert_called_once()
            self.assertEqual(self.publisher.publish.call_args.args[0].position, [0.1])
            self.assertTrue(emit.call_args.kwargs["success"])
            self.assertEqual(emit.call_args.kwargs["published_at_ns"], 12_000_000_345)

    def test_unmatched_publisher_times_out_without_publishing(self):
        with patch("ros_manager.wsl_bridge.commands.emit") as emit, patch(
            "ros_manager.wsl_bridge.commands.time.monotonic", return_value=131,
        ):
            _publisher, completed = publish_position_command(
                self.node, None, self.command, self.message_type,
            )
        self.assertTrue(completed)
        self.publisher.publish.assert_not_called()
        self.assertFalse(emit.call_args.kwargs["success"])


class WorkerProtocolTest(unittest.TestCase):
    def test_ros_publication_barrier_is_forwarded_before_acknowledgement(self):
        callbacks = Mock()
        worker = RosWorker(
            Mock(), Mock(), callbacks.message, Mock(), callbacks.result, Mock(),
            on_position_published=callbacks.published,
        )
        worker._handle_event({
            "event": "command_result", "id": "pose", "success": True,
            "message": "Published", "published_at_ns": 12_000_000_345,
        })
        self.assertEqual([call[0] for call in callbacks.mock_calls], ["published", "result"])
        callbacks.published.assert_called_once_with("pose", 12_000_000_345)

        data = {"name": ["servo_11"], "position": [0.0]}
        worker._handle_event({
            "event": "message", "topic": "/saprog/joint_states", "data": data,
            "received_at_ns": 12_000_000_999,
        })
        callbacks.message.assert_called_once_with(
            "/saprog/joint_states", {**data, "_ros_received_at_ns": 12_000_000_999},
        )
        self.assertNotIn("_ros_received_at_ns", data)

    def test_failed_publication_does_not_set_an_arrival_barrier(self):
        published, result = Mock(), Mock()
        worker = RosWorker(
            Mock(), Mock(), Mock(), Mock(), result, Mock(), on_position_published=published,
        )
        worker._handle_event({
            "event": "command_result", "id": "pose", "success": False,
            "message": "Cancelled", "published_at_ns": 12_000_000_345,
        })
        published.assert_not_called()
        result.assert_called_once_with("pose", False, "Cancelled")


class DiscoveryTest(unittest.TestCase):
    def test_transient_read_and_discovery_errors_restore_the_selected_device(self):
        for error_source in ("poll", "refresh"):
            with self.subTest(error_source=error_source):
                monitor = ControllerMonitor()
                self.addCleanup(dispose, monitor)
                self.addCleanup(monitor.close)
                stick = SimpleNamespace(
                    init=lambda: None, quit=lambda: None, get_instance_id=lambda: 42,
                    get_name=lambda: "Gamepad", get_guid=lambda: "guid",
                    get_numbuttons=lambda: 1, get_numhats=lambda: 0, get_numaxes=lambda: 0,
                    get_button=lambda index: True,
                )
                monitor.pygame = SimpleNamespace(
                    event=SimpleNamespace(pump=Mock(), get=Mock(return_value=[])),
                    joystick=SimpleNamespace(get_count=lambda: 1, Joystick=lambda index: stick),
                    error=RuntimeError, JOYDEVICEADDED=1, JOYDEVICEREMOVED=2,
                )
                selected, inputs, discovered = [], [], []
                monitor.selected_changed.connect(selected.append)
                monitor.inputs_changed.connect(inputs.append)
                monitor.devices_changed.connect(discovered.append)
                monitor.refresh()
                monitor.select(42)
                call = monitor.pygame.event.get if error_source == "poll" else monitor.pygame.event.pump
                call.side_effect = RuntimeError("temporary read failure")
                getattr(monitor, error_source)()
                self.assertIsNone(monitor.selected)
                self.assertIsNone(selected[-1])
                self.assertEqual(inputs[-1], set())
                # Repeated failures must retain the original selection for recovery.
                getattr(monitor, error_source)()
                call.side_effect = None
                monitor.refresh()
                self.assertEqual(monitor.selected, 42)
                self.assertEqual(selected[-1]["id"], 42)
                self.assertEqual(len(discovered), 1)
                self.assertEqual(inputs[-1], set())
                monitor.poll()
                self.assertEqual(inputs[-1], {"button:0"})

    def test_native_discovery_inputs_and_hot_unplug(self):
        app = QApplication.instance() or QApplication([])
        monitor = ControllerMonitor()
        stick = SimpleNamespace(
            init=lambda: None, quit=lambda: None, get_instance_id=lambda: 42,
            get_name=lambda: "Gamepad", get_guid=lambda: "guid",
            get_numbuttons=lambda: 2, get_numhats=lambda: 1, get_numaxes=lambda: 1,
            get_button=lambda index: index == 1, get_hat=lambda index: (-1, 1),
            get_axis=lambda index: 0.9,
        )
        devices = [stick]
        events = []
        monitor.pygame = SimpleNamespace(
            event=SimpleNamespace(pump=lambda: None, get=lambda: list(events)),
            joystick=SimpleNamespace(get_count=lambda: len(devices), Joystick=lambda index: devices[index]),
            error=RuntimeError, JOYDEVICEADDED=1, JOYDEVICEREMOVED=2,
        )
        selected, inputs, discovered = [], [], []
        monitor.selected_changed.connect(selected.append)
        monitor.inputs_changed.connect(inputs.append)
        monitor.devices_changed.connect(discovered.append)
        monitor.refresh()
        self.assertEqual(discovered[-1][0]["id"], 42)
        monitor.select(42)
        self.assertEqual(len(selected[-1]["controls"]), 8)
        monitor.poll()
        self.assertEqual(inputs[-1], {"button:1", "hat:0:left", "hat:0:up", "axis:0:+"})
        devices.clear()
        events.append(SimpleNamespace(type=monitor.pygame.JOYDEVICEREMOVED))
        monitor.poll()
        self.assertIsNone(selected[-1])
        self.assertEqual(inputs[-1], set())
        self.assertEqual(discovered[-1], [])
        monitor.close()
        dispose(monitor)
        app.processEvents()


if __name__ == "__main__":
    unittest.main()
