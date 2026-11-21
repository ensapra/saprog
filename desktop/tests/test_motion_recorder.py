"""Motion recorder and timeline export checks without ROS or robot hardware."""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from documents.timeline_domain import mirrored_positions
from documents.timeline_storage import load_snapshot_reference, load_timeline
from limit_fixtures import prime_limits
from PySide6.QtCore import QObject, Qt, Signal
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QMessageBox, QPushButton
from tab_workspace import WorkspaceMainWindow
from tabs.motion_recorder import MotionRecorderTab
from widgets.player import StaticAnimatorWidget


class FakeManager(QObject):
    message_received = Signal(str, object)
    command_result = Signal(str, bool, str)
    position_published = Signal(str, object)
    torque_release_requested = Signal()

    def __init__(self):
        super().__init__()
        self.commands = []
        self.cancelled = []
        prime_limits(self)

    def refresh_servo_limits(self):
        return "limits-read"

    def request_topic(self, *args, **kwargs):
        return "state-token"

    def release(self, token):
        pass

    def set_servo_torque(self, joint, enabled):
        command_id = str(len(self.commands))
        self.commands.append((command_id, joint, enabled))
        return command_id

    def publish_joint_command(self, joints, positions, speed, torque):
        command_id = str(len(self.commands))
        self.commands.append((command_id, joints, positions, speed, torque))
        return command_id

    def cancel_commands(self, command_ids):
        self.cancelled.extend(command_ids)


class MotionRecorderTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.snapshots = self.root / "snapshots"
        self.sequences = self.root / "sequences"
        self.snapshots.mkdir()
        self.sequences.mkdir()
        self.now = 10.0
        self.manager = FakeManager()
        self.widget = MotionRecorderTab(
            self.manager,
            snapshot_directory=self.snapshots,
            sequence_directory=self.sequences,
            clock=lambda: self.now,
        )
        self.addCleanup(self.widget.deleteLater)
        self.receive_pose(0.1)

    def receive_pose(self, start, torque=1):
        self.manager.message_received.emit("/saprog/joint_states", {
            "name": ["servo_11", "servo_12", "servo_13"],
            "position": [start, start + 0.1, start + 0.2],
            "effort": [torque, torque, torque],
        })

    def receive_both_legs(self, first, second, *, first_torque=1, second_torque=0):
        self.manager.message_received.emit("/saprog/joint_states", {
            "name": [
                "servo_11", "servo_12", "servo_13",
                "servo_21", "servo_22", "servo_23",
            ],
            "position": [
                first, first + 0.1, first + 0.2,
                second, second + 0.1, second + 0.2,
            ],
            "effort": [first_torque] * 3 + [second_torque] * 3,
        })

    def acknowledge_torque(self, success=True):
        for command_id, _joint, _enabled in self.manager.commands:
            self.manager.command_result.emit(command_id, success, "torque request result")

    def test_selected_leg_torque_off_acknowledgement_gates_recording(self):
        self.assertTrue(self.widget.set_leg_torque(False))
        self.assertEqual(
            [joint for _command_id, joint, enabled in self.manager.commands if not enabled],
            ["servo_11", "servo_12", "servo_13"],
        )
        self.assertFalse(self.widget.start_recording())
        self.acknowledge_torque()
        self.assertTrue(self.widget.start_recording())
        self.assertAlmostEqual(self.widget.frames[0]["positions"]["servo_13"], 0.3)
        self.widget.stop_recording()

    def test_recorded_poses_save_as_loadable_timeline(self):
        self.widget.set_leg_torque(False)
        self.acknowledge_torque()
        self.assertTrue(self.widget.capture_pose())
        self.now += 60
        self.receive_pose(0.1, torque=0)
        self.assertTrue(self.widget.start_recording())
        self.now += self.widget.sample_interval.value()
        self.receive_pose(0.4)
        self.widget._record_tick()
        self.widget.stop_recording()
        target = self.sequences / "guided_leg.json"
        self.assertTrue(self.widget.save_timeline(target))

        document = load_timeline(target, self.snapshots)
        self.assertEqual(len(document["steps"]), 4)
        self.assertAlmostEqual(
            document["steps"][0]["duration"]["seconds"],
            self.widget.sample_interval.value(),
        )
        for step in document["steps"]:
            snapshot = load_snapshot_reference(step["snapshot"], self.snapshots)
            self.assertEqual(set(snapshot["positions"]), {"servo_11", "servo_12", "servo_13"})
            self.assertEqual(step["motion"], {"speed": 512, "torque": 512})
            self.assertEqual(step["duration"]["mode"], "seconds")
        saved = load_snapshot_reference(document["steps"][2]["snapshot"], self.snapshots)
        self.assertEqual(saved["positions"]["servo_11"], 0.4)
        raw = json.loads(target.read_text(encoding="utf-8"))
        self.assertEqual(raw["kind"], "saprog_timeline")
        self.assertEqual(raw["schema_version"], 2)

    def test_failed_torque_off_does_not_allow_recording(self):
        self.widget.set_leg_torque(False)
        self.acknowledge_torque(success=False)
        self.assertIn("failed", self.widget.status.text().lower())
        self.assertFalse(self.widget.start_recording())

    def test_stale_joint_feedback_cannot_be_recorded(self):
        self.now += 3
        self.assertFalse(self.widget.capture_pose())
        self.assertFalse(self.widget.start_recording())

    def test_snapshot_pose_import_uses_only_selected_leg(self):
        snapshot = self.snapshots / "FrontStep3.json"
        snapshot.write_text(json.dumps({
            "schema_version": 1,
            "name": "FrontStep3",
            "positions": {
                "servo_11": 100.0, "servo_12": 0.5, "servo_13": 0.4,
                "servo_21": 0.0, "servo_22": 0.5, "servo_23": 0.4,
            },
        }), encoding="utf-8")
        self.assertTrue(self.widget.add_snapshot_pose(snapshot), self.widget.status.text())
        self.assertAlmostEqual(
            self.widget.frames[0]["positions"]["servo_11"],
            self.widget.playback.bounds["servo_11"][1],
        )
        self.assertEqual(self.widget.frames[0]["positions"]["servo_12"], 0.5)
        self.assertEqual(self.widget.frames[0]["positions"]["servo_13"], 0.4)
        self.assertIn("FrontStep3", self.widget.pose_list.item(0, 0).text())
        self.assertIn("11:", self.widget.pose_list.item(0, 1).text())
        self.assertEqual(self.widget.pose_list.item(0, 2).text(), "—")
        self.widget.pose_list.setCurrentCell(0, 0)
        self.assertTrue(self.widget.remove_selected_pose())
        self.assertEqual(self.widget.frames, [])

    def test_recorded_poses_are_split_by_leg_and_can_be_deleted(self):
        self.widget._append_pose({
            "servo_11": 0.1, "servo_12": 0.2, "servo_13": 0.3,
            "servo_31": 0.4, "servo_32": 0.5, "servo_33": 0.6,
        })
        self.assertEqual(self.widget.pose_list.columnCount(), 5)
        self.assertEqual(self.widget.pose_list.horizontalHeaderItem(1).text(), "Front Right Leg")
        self.assertIn("11: 0.100", self.widget.pose_list.item(0, 1).text())
        self.assertIn("31: 0.400", self.widget.pose_list.item(0, 3).text())

        self.widget.base_loop_locked = True
        self.widget._update_controls()
        self.assertTrue(self.widget.remove_pose_button.isEnabled())
        self.widget.remove_pose_button.click()
        self.assertEqual(self.widget.frames, [])
        self.assertTrue(self.widget.base_loop_locked)

    def test_removing_leg_data_does_not_remove_the_timeline_step(self):
        self.widget._append_pose({
            "servo_11": 0.1, "servo_12": 0.2, "servo_13": 0.3,
            "servo_31": 0.4, "servo_32": 0.5, "servo_33": 0.6,
        })
        self.widget.pose_list.setCurrentCell(0, 0)
        self.widget.remove_leg_data_button.click()
        self.assertEqual(self.widget.pose_list.rowCount(), 1)
        self.assertEqual(len(self.widget.frames), 1)
        self.assertFalse({
            "servo_11", "servo_12", "servo_13",
        } & self.widget.frames[0]["positions"].keys())
        self.assertIn("servo_31", self.widget.frames[0]["positions"])

        self.widget.remove_pose_button.click()
        self.assertEqual(self.widget.pose_list.rowCount(), 0)
        self.assertEqual(self.widget.timeline_entries, [])

    def test_removing_loaded_leg_filters_overrides_for_save_and_playback(self):
        pose = self.snapshots / "two_legs.json"
        pose.write_text(json.dumps({
            "schema_version": 1, "name": "two legs",
            "positions": {"servo_11": 0.1, "servo_12": 0.2, "servo_13": 0.3,
                          "servo_21": 0.1, "servo_22": 0.2, "servo_23": 0.3},
        }), encoding="utf-8")
        timeline = self.sequences / "overrides.json"
        timeline.write_text(json.dumps({
            "kind": "saprog_timeline", "schema_version": 2,
            "smooth_playback": True,
            "steps": [{"snapshot": pose.name, "motion": {"speed": 512, "torque": 512},
                       "overrides": {"servo_11": {"speed": 100, "torque": 200},
                                     "servo_21": {"speed": 300, "torque": 400}}}],
        }), encoding="utf-8")
        self.assertTrue(self.widget.load_timeline(timeline), self.widget.status.text())
        self.assertTrue(self.widget.smooth_playback.isChecked())
        self.assertTrue(self.widget.remove_selected_leg_data())
        self.assertTrue(self.widget.play_recorded_motion(), self.widget.status.text())
        self.assertTrue(self.widget.playback.smooth)
        self.widget.stop_playback()
        self.assertTrue(self.widget.save_timeline(), self.widget.status.text())
        saved = load_timeline(timeline, self.snapshots)
        self.assertTrue(saved["smooth_playback"])
        self.assertEqual(saved["steps"][0]["overrides"], {
            "servo_21": {"speed": 300, "torque": 400},
        })

    def test_applying_pose_with_new_leg_includes_it_in_later_capture(self):
        self.assertTrue(self.widget.capture_pose())
        self.receive_both_legs(0.1, 0.5)
        self.widget.leg_picker.setCurrentIndex(1)
        frame = self.widget.frames[0]
        self.assertTrue(self.widget.apply_pose_edit(frame, dict(frame["positions"], **{
            joint: self.widget.positions[joint] for joint in self.widget._selected_joints()
        })))
        self.widget.leg_picker.setCurrentIndex(0)
        self.assertTrue(self.widget.capture_pose())
        self.assertEqual(set(self.widget.frames[-1]["positions"]), {
            "servo_11", "servo_12", "servo_13", "servo_21", "servo_22", "servo_23",
        })

    def test_copy_leg_pastes_with_wrapped_offset_and_preserves_other_legs(self):
        for value in (0.2, 0.4, 0.6):
            self.widget._append_pose({"servo_11": value, "servo_12": value, "servo_13": value})
        self.assertTrue(self.widget.copy_leg())
        self.widget.leg_picker.setCurrentIndex(1)
        self.widget.paste_offset.setValue(1)
        self.assertTrue(self.widget.paste_leg())
        self.assertEqual([frame["positions"]["servo_21"] for frame in self.widget.frames],
                         [0.6, 0.2, 0.4])
        self.assertEqual([frame["positions"]["servo_11"] for frame in self.widget.frames],
                         [0.2, 0.4, 0.6])
        target = self.sequences / "copied.json"
        self.assertTrue(self.widget.save_timeline(target), self.widget.status.text())
        saved = load_timeline(target, self.snapshots)
        pose = load_snapshot_reference(saved["steps"][0]["snapshot"], self.snapshots)
        self.assertEqual(pose["positions"]["servo_21"], 0.6)

    def test_copy_rejects_missing_source_and_paste_rejects_changed_pose_count(self):
        self.widget._append_pose({"servo_11": 0.2})
        self.assertFalse(self.widget.copy_leg())
        self.widget.frames[0]["positions"].update({"servo_12": 0.2, "servo_13": 0.2})
        self.assertTrue(self.widget.copy_leg())
        self.widget._append_pose({"servo_11": 0.3, "servo_12": 0.3, "servo_13": 0.3})
        self.widget.leg_picker.setCurrentIndex(1)
        self.assertFalse(self.widget.paste_leg())
        self.assertNotIn("servo_21", self.widget.frames[0]["positions"])

    def test_playback_marks_pose_and_wait_and_clears_on_stop(self):
        self.widget._append_pose({"servo_11": 0.2, "servo_12": 0.2, "servo_13": 0.2})
        self.widget.timeline_entries.append({"kind": "wait", "seconds": 0.1})
        self.widget._refresh_pose_list()
        self.assertTrue(self.widget.play_recorded_motion())
        self.manager.command_result.emit(self.manager.commands[-1][0], True, "published")
        self.assertTrue(self.widget.pose_list.item(0, 0).text().startswith("▶ "))
        self.now += self.widget.sample_interval.value()
        self.widget.playback.tick()
        self.assertFalse(self.widget.pose_list.item(0, 0).text().startswith("▶ "))
        self.assertTrue(self.widget.pose_list.item(1, 0).text().startswith("▶ "))
        self.now += 0.1
        self.widget.playback.tick()
        self.manager.command_result.emit(self.manager.commands[-1][0], True, "published")
        self.assertTrue(self.widget.pose_list.item(0, 0).text().startswith("▶ "))
        self.widget.stop_playback()
        self.assertFalse(self.widget.pose_list.item(0, 0).text().startswith("▶ "))

    def test_selected_step_plays_once_with_its_settings_and_original_row_highlight(self):
        self.widget._append_pose({"servo_11": 0.1})
        self.widget.timeline_entries.append({"kind": "wait", "seconds": 2})
        self.widget._append_pose({"servo_11": 0.4})
        frame = self.widget.frames[-1]
        frame["timeline_step"] = {
            "snapshot": "unused.json", "mirror": "Horizontal",
            "motion": {"speed": 123, "torque": 456},
            "overrides": {"servo_11": {"speed": 321, "torque": 654}},
            "duration": {"mode": "seconds", "seconds": 1.25},
        }
        frame["positions_changed"] = True
        self.widget._refresh_pose_list()
        self.widget.pose_list.setCurrentCell(2, 0)
        self.widget.play_step_button.click()
        self.assertIs(self.widget.playback.owner, self.widget)
        self.assertFalse(self.widget.playback.repeat)
        self.assertEqual(len(self.widget.playback.steps), 1)
        self.assertEqual(self.manager.commands[-1][1:], (["servo_11"], [0.4], 321, 654))
        self.assertFalse(self.widget.play_step_button.isEnabled())
        self.assertFalse(self.widget.edit_pose_button.isEnabled())
        self.widget.pose_list.setCurrentCell(0, 0)
        self.manager.command_result.emit(self.manager.commands[-1][0], True, "published")
        self.assertTrue(self.widget.pose_list.item(2, 0).text().startswith("▶ "))
        self.assertFalse(self.widget.pose_list.item(0, 0).text().startswith("▶ "))
        self.widget.leg_picker.setCurrentIndex(1)
        self.receive_both_legs(0.1, 0.5, second_torque=0)
        self.assertFalse(self.widget.record_button.isEnabled())
        self.assertFalse(self.widget.start_overlay_recording())
        self.now += 1.25
        self.widget.playback.tick()
        self.assertIsNone(self.widget.playback.owner)
        self.assertIsNone(self.widget._playback_temp_dir)
        self.assertIsNone(self.widget._preview_row)
        self.assertEqual(len(self.manager.commands), 1)
        self.assertFalse(self.widget.pose_list.item(2, 0).text().startswith("▶ "))
        self.assertTrue(self.widget.play_step_button.isEnabled())

    def test_selected_wait_runs_once_without_sending_pose_commands(self):
        self.widget._append_pose({"servo_11": 0.1})
        self.widget.timeline_entries.append({"kind": "wait", "seconds": 0.5})
        self.widget._refresh_pose_list()
        self.widget.pose_list.setCurrentCell(1, 0)
        self.assertTrue(self.widget.play_step_button.isEnabled())
        self.assertTrue(self.widget.play_selected_step())
        self.assertTrue(self.widget.pose_list.item(1, 0).text().startswith("▶ "))
        self.assertEqual(self.manager.commands, [])
        self.now += 0.5
        self.widget.playback.tick()
        self.assertIsNone(self.widget.playback.owner)
        self.assertEqual(self.manager.commands, [])

    def test_selected_step_requires_selection_connection_and_idle_recorder(self):
        self.assertFalse(self.widget.play_step_button.isEnabled())
        self.assertFalse(self.widget.play_selected_step())
        self.widget._append_pose({"servo_11": 0.1})
        self.widget.recording = True
        self.widget._update_controls()
        self.assertFalse(self.widget.play_step_button.isEnabled())
        self.assertFalse(self.widget.play_selected_step())
        self.widget.recording = False
        self.widget.manager = None
        self.widget._update_controls()
        self.assertFalse(self.widget.play_step_button.isEnabled())
        self.assertFalse(self.widget.play_selected_step())
        self.assertEqual(self.manager.commands, [])
        self.assertNotIn("Update selected pose", [
            button.text() for button in self.widget.findChildren(QPushButton)
        ])

    def test_partial_combined_mirror_round_trips_edited_pose(self):
        pose = self.snapshots / "partial.json"
        pose.write_text(json.dumps({
            "schema_version": 1, "name": "partial",
            "positions": {"servo_11": 0.1, "servo_21": 0.3, "servo_31": 0.5},
        }), encoding="utf-8")
        timeline = self.sequences / "mirrored.json"
        timeline.write_text(json.dumps({
            "kind": "saprog_timeline", "schema_version": 2,
            "steps": [{"snapshot": pose.name, "mirror": "Horizontal + Vertical",
                       "motion": {"speed": 512, "torque": 512}}],
        }), encoding="utf-8")
        self.assertTrue(self.widget.load_timeline(timeline), self.widget.status.text())
        frame = self.widget.frames[0]
        self.assertTrue(self.widget.apply_pose_edit(frame, dict(frame["positions"], servo_11=0.2)))
        expected = dict(self.widget.frames[0]["positions"])
        self.assertTrue(self.widget.save_timeline(), self.widget.status.text())
        step = load_timeline(timeline, self.snapshots)["steps"][0]
        saved = load_snapshot_reference(step["snapshot"], self.snapshots)
        self.assertEqual(mirrored_positions(saved["positions"], step["mirror"]), expected)

    def test_cannot_add_poses_while_playback_is_running(self):
        self.assertTrue(self.widget.capture_pose())
        self.assertTrue(self.widget.play_recorded_motion())
        self.assertFalse(self.widget.capture_button.isEnabled())
        self.assertFalse(self.widget.import_pose_button.isEnabled())
        self.assertFalse(self.widget.capture_pose())
        self.assertFalse(self.widget.add_snapshot_pose(self.snapshots / "unused.json"))
        self.assertEqual(len(self.widget.frames), 1)
        self.widget.stop_playback()
        self.assertTrue(self.widget.capture_button.isEnabled())
        self.assertTrue(self.widget.import_pose_button.isEnabled())

    def test_failed_first_playback_command_cleans_up_playback(self):
        self.assertTrue(self.widget.capture_pose())
        with patch.object(self.manager, "publish_joint_command", side_effect=RuntimeError("offline")):
            self.assertFalse(self.widget.play_recorded_motion())
        self.assertIsNone(self.widget.playback.owner)
        self.assertFalse(self.widget.playback.timer.isActive())
        self.assertIsNone(self.widget._playback_temp_dir)
        self.assertTrue(self.widget.play_recorded_motion(), self.widget.status.text())
        self.widget.stop_playback()

    def test_shortened_loop_can_play_and_record_another_leg_over_it(self):
        self.assertTrue(self.widget.set_leg_torque(False))
        self.acknowledge_torque()
        self.assertTrue(self.widget.start_recording())
        self.widget.stop_recording()
        self.assertEqual(len(self.widget.frames), 2)

        self.widget.pose_list.setCurrentCell(0, 0)
        self.widget.remove_pose_button.click()
        self.assertEqual(len(self.widget.frames), 1)
        self.assertEqual(len(self.widget.timeline_entries), 1)
        self.assertTrue(self.widget.base_loop_locked)

        self.assertTrue(self.widget.play_recorded_motion())
        playback_command_id = self.manager.commands[-1][0]
        self.manager.command_result.emit(playback_command_id, True, "position published")
        self.widget.leg_picker.setCurrentIndex(1)
        self.receive_both_legs(0.1, 0.5, first_torque=1, second_torque=0)
        self.assertTrue(self.widget.record_button.isEnabled())
        self.assertTrue(self.widget.start_recording(), self.widget.status.text())
        self.assertFalse(self.widget.recording)
        self.assertEqual(len(self.widget.frames), 1)
        self.assertIn("servo_21", self.widget.frames[0]["positions"])

    def test_captured_feedback_is_clamped_in_recorded_poses(self):
        self.receive_pose(100)
        self.assertTrue(self.widget.capture_pose())
        positions = self.widget.frames[0]["positions"]
        for joint in ("servo_11", "servo_12", "servo_13"):
            self.assertEqual(positions[joint], self.widget.playback.bounds[joint][1])

    def test_selected_pose_requests_shared_editor_and_applies_with_servo_limits(self):
        self.widget._append_pose({
            "servo_11": 0.1, "servo_12": 0.2, "servo_13": 0.3,
        })
        self.widget.pose_list.setCurrentCell(0, 0)
        self.assertTrue(self.widget.edit_pose_button.isEnabled())
        upper = self.widget.playback.bounds["servo_11"][1]

        requests = []
        self.widget.pose_edit_requested.connect(lambda *args: requests.append(args))
        self.widget.edit_pose_button.click()
        self.assertEqual(len(requests), 1)
        recorder, frame = requests[0]
        self.assertIs(recorder, self.widget)
        self.assertIs(frame, self.widget.frames[0])
        self.assertEqual(frame["positions"]["servo_11"], 0.1)
        recorder.apply_pose_edit(frame, dict(frame["positions"], servo_11=100))
        self.assertAlmostEqual(self.widget.frames[0]["positions"]["servo_11"], upper)
        self.assertEqual(self.widget.frames[0]["positions"]["servo_12"], 0.2)

    def make_workspace(self):
        window = WorkspaceMainWindow()
        animator = StaticAnimatorWidget(directory=str(self.snapshots), ros_manager=self.manager)
        window.add_tab(animator, "Static Animator")
        window.add_tab(self.widget, "Motion Recorder")
        window.tabs.setCurrentWidget(self.widget)
        window.show()
        self.app.processEvents()

        def cleanup():
            for detached in list(window._detached_windows.values()):
                detached.close()
            window.close()
            window.deleteLater()
            self.app.processEvents()

        self.addCleanup(cleanup)
        return window, animator

    def test_edit_reuses_attached_and_detached_animator_and_saves_original_step(self):
        window, animator = self.make_workspace()
        view = animator.creator_view
        self.widget._append_pose({"servo_11": 0.1, "servo_12": 0.2})
        frame = self.widget.frames[0]
        self.widget._append_pose({"servo_11": 0.3})
        for detached in (False, True):
            with self.subTest(detached=detached):
                if detached:
                    window.detach_tab(window.tabs.indexOf(animator))
                    window.detach_tab(window.tabs.indexOf(self.widget))
                self.widget.pose_list.setCurrentCell(0, 0)
                window.tabs.setCurrentWidget(self.widget)
                self.widget.edit_pose_button.click()
                self.app.processEvents()
                if detached:
                    self.assertIs(window._detached_windows[animator].centralWidget(), animator)
                    self.assertTrue(window._detached_windows[animator].isVisible())
                    self.assertEqual(len(window._detached_windows), 2)
                else:
                    self.assertIs(window.tabs.currentWidget(), animator)
                    self.assertFalse(window._detached_windows)
                self.assertEqual(float(view.target_inputs["servo_11"].text()), 0.1)
                self.assertEqual(view.target_inputs["servo_13"].text(), "")
                self.assertIn("Motion Recorder", view.save_button.text())
                self.widget.pose_list.setCurrentCell(1, 0)
                view.target_inputs["servo_12"].setText("100")
                view.save_button.click()
                self.assertEqual(frame["positions"]["servo_12"], view.servo_limits["servo_12"][1])
                self.assertEqual(set(frame["positions"]), {"servo_11", "servo_12"})
                self.assertEqual(self.widget.frames[1]["positions"], {"servo_11": 0.3})
                self.assertTrue(frame["positions_changed"])
        destination = self.sequences / "edited.json"
        self.assertTrue(self.widget.save_timeline(destination), self.widget.status.text())
        saved = load_timeline(destination, self.snapshots, bounds=self.widget.playback.bounds)
        pose = load_snapshot_reference(saved["steps"][0]["snapshot"], self.snapshots)
        self.assertEqual(pose["positions"], frame["positions"])

    def test_edit_loads_full_recorded_pose_without_moving_robot(self):
        window, animator = self.make_workspace()
        document = {"name": "pose_0009", "positions": {
            "servo_31": -0.12027928862717521, "servo_32": 0.8803420273988994,
            "servo_33": 0.4504075489017625, "servo_21": -0.12027928862717521,
            "servo_22": 0.8803420273988994, "servo_23": 0.4504075489017625,
            "servo_11": -0.12027928862717521, "servo_12": 0.3633969996821038,
            "servo_13": 0.7626218725723023, "servo_41": -0.12027928862717521,
            "servo_42": 0.3633969996821038, "servo_43": 0.7626218725723023,
        }}
        self.widget._append_pose(document["positions"], label=document["name"])
        frame = self.widget.frames[0]
        animator.creator_view.set_puppet_mode("servo_11", True)
        self.widget.edit_pose_button.click()
        self.assertIs(window.tabs.currentWidget(), animator)
        self.assertEqual(animator.creator_view.name_input.text(), "pose_0009")
        for joint, value in frame["positions"].items():
            self.assertEqual(float(animator.creator_view.target_inputs[joint].text()), value)
        self.assertFalse(animator.creator_view.puppet_buttons["servo_11"].isChecked())
        self.assertEqual(self.manager.commands, [])

    def test_double_click_opens_clicked_pose_in_existing_animator(self):
        window, animator = self.make_workspace()
        self.widget._append_pose({"servo_11": 0.1})
        self.widget._append_pose({"servo_11": 0.4})
        self.widget.timeline_entries.append({"kind": "wait", "seconds": 0.5})
        self.widget._refresh_pose_list()
        self.widget.pose_list.setCurrentCell(0, 0)
        self.app.processEvents()
        table = self.widget.pose_list
        point = table.visualItemRect(table.item(1, 1)).center()
        QTest.mouseClick(table.viewport(), Qt.MouseButton.LeftButton, pos=point)
        QTest.mouseDClick(table.viewport(), Qt.MouseButton.LeftButton, pos=point)
        self.app.processEvents()
        self.assertIs(window.tabs.currentWidget(), animator)
        self.assertEqual(float(animator.creator_view.target_inputs["servo_11"].text()), 0.4)
        self.assertEqual(self.manager.commands, [])
        for row, recording in ((2, False), (0, True)):
            window.tabs.setCurrentWidget(self.widget)
            self.widget.recording = recording
            table.cellDoubleClicked.emit(row, 0)
            self.assertIs(window.tabs.currentWidget(), self.widget)
        self.widget.recording = False

    def test_pose_apply_rejects_invalid_targets_recording_and_removed_steps(self):
        _window, animator = self.make_workspace()
        self.widget._append_pose({"servo_11": 0.1})
        self.widget.edit_pose_button.click()
        view = animator.creator_view
        for invalid in ("", "abc", "nan", "inf"):
            view.target_inputs["servo_11"].setText(invalid)
            view.save_button.click()
            self.assertIn("Could not apply pose", animator.keyframes_view.status_label.text())
            self.assertEqual(self.widget.frames[0]["positions"], {"servo_11": 0.1})
        view.target_inputs["servo_11"].setText("0.2")
        self.widget.recording = True
        view.save_button.click()
        self.assertIn("Stop recording", animator.keyframes_view.status_label.text())
        self.widget.recording = False
        self.widget.remove_selected_pose()
        self.widget._append_pose({"servo_11": 0.1})
        view.save_button.click()
        self.assertIn("no longer", animator.keyframes_view.status_label.text())
        self.assertEqual(self.widget.frames[0]["positions"], {"servo_11": 0.1})
        view.finish_edit_button.click()
        self.assertEqual(view.save_button.text(), "Save targets")
        self.assertFalse(view.name_input.isReadOnly())

    def test_loading_another_keyframe_ends_recorder_edit(self):
        _window, animator = self.make_workspace()
        self.widget._append_pose({"servo_11": 0.1})
        self.widget.edit_pose_button.click()
        source = self.snapshots / "another.json"
        source.write_text(json.dumps({
            "schema_version": 1, "name": "another", "positions": {"servo_11": 0.2},
        }), encoding="utf-8")
        animator._load_keyframe(str(source))
        self.assertIsNone(animator._pose_edit)
        self.assertEqual(animator.creator_view.save_button.text(), "Save targets")
        self.assertEqual(self.widget.frames[0]["positions"], {"servo_11": 0.1})

    def test_load_update_remove_and_save_preserve_other_timeline_details(self):
        source_pose = self.snapshots / "source_pose.json"
        source_pose.write_text(json.dumps({
            "schema_version": 1,
            "name": "source_pose",
            "positions": {
                "servo_11": 0.1, "servo_12": 0.2, "servo_13": 0.3,
                "servo_31": 0.4, "servo_32": 0.5, "servo_33": 0.6,
            },
        }), encoding="utf-8")
        second_pose = self.snapshots / "second_pose.json"
        second_pose.write_text(json.dumps({
            "schema_version": 1,
            "name": "second_pose",
            "positions": {"servo_11": 0.2, "servo_12": 0.3, "servo_13": 0.4},
        }), encoding="utf-8")
        source_timeline = self.sequences / "source_timeline.json"
        source_timeline.write_text(json.dumps({
            "kind": "saprog_timeline",
            "schema_version": 2,
            "on_end": "off",
            "loop": False,
            "steps": [
                {"kind": "wait", "seconds": 0.5},
                {
                    "snapshot": "source_pose.json",
                    "mirror": "Horizontal",
                    "motion": {"speed": 123, "torque": 456},
                    "duration": {"mode": "until_reached", "seconds": 1.5},
                },
                {"kind": "wait", "seconds": 0.25},
                {
                    "snapshot": "second_pose.json",
                    "motion": {"speed": 700, "torque": 600},
                    "duration": {"mode": "seconds", "seconds": 2.5},
                },
            ],
        }), encoding="utf-8")

        self.assertTrue(self.widget.load_timeline(source_timeline), self.widget.status.text())
        self.assertEqual(self.widget.pose_list.rowCount(), 4)
        self.assertEqual(self.widget.pose_list.item(0, 0).text(), "Wait — 0.5 s")
        self.assertEqual(self.widget.pose_list.item(2, 0).text(), "Wait — 0.25 s")
        self.assertTrue(self.widget.play_recorded_motion(), self.widget.status.text())
        self.assertEqual(len(self.widget.playback.steps), 4)
        self.assertEqual(self.widget.playback.steps[0]["kind"], "wait")
        self.assertFalse(self.widget.playback.repeat)
        self.widget.stop_playback()

        self.receive_pose(0.6)
        self.widget.pose_list.setCurrentCell(1, 0)
        frame = self.widget.frames[0]
        self.assertTrue(self.widget.apply_pose_edit(frame, dict(frame["positions"], servo_11=0.6)))
        self.assertAlmostEqual(self.widget.frames[0]["positions"]["servo_11"], 0.6)

        self.widget.pose_list.setCurrentCell(3, 0)
        self.assertTrue(self.widget.remove_selected_pose())
        with patch("tabs.motion_recorder.QFileDialog.getSaveFileName") as save_dialog:
            self.widget.save_button.click()
        save_dialog.assert_not_called()
        destination = source_timeline
        saved = load_timeline(
            destination, self.snapshots, bounds=self.widget.playback.bounds,
        )
        self.assertEqual((saved["loop"], saved["on_end"]), (False, "off"))
        self.assertEqual(
            [step.get("kind", "pose") for step in saved["steps"]],
            ["wait", "pose", "wait"],
        )
        pose_step = saved["steps"][1]
        self.assertEqual(pose_step["motion"], {"speed": 123, "torque": 456})
        self.assertEqual(
            pose_step["duration"], {"mode": "until_reached", "seconds": 1.5},
        )
        self.assertEqual(pose_step["mirror"], "Horizontal")
        snapshot = load_snapshot_reference(
            pose_step["snapshot"], self.snapshots, bounds=self.widget.playback.bounds,
        )
        effective = mirrored_positions(snapshot["positions"], pose_step["mirror"])
        self.assertAlmostEqual(effective["servo_11"], 0.6)

        with patch(
            "tabs.motion_recorder.QMessageBox.question",
            return_value=QMessageBox.StandardButton.Yes,
        ):
            self.assertTrue(self.widget.new_timeline())
        self.assertEqual(self.widget.timeline_entries, [])
        self.assertIsNone(self.widget.timeline_path)

    def test_save_button_opens_dialog_and_saves_timeline(self):
        self.widget._append_pose(
            {"servo_11": 0.1, "servo_12": 0.2, "servo_13": 0.3}
        )
        target = self.sequences / "button_saved.json"
        with patch(
            "tabs.motion_recorder.QFileDialog.getSaveFileName",
            return_value=(str(target), "Timeline JSON (*.json)"),
        ) as prompt:
            self.widget.save_button.click()
        prompt.assert_called_once()
        self.assertTrue(target.is_file())

    def test_playback_can_run_while_recording_another_leg_on_top(self):
        self.widget.set_leg_torque(False)
        self.acknowledge_torque()
        self.assertTrue(self.widget.start_recording())
        self.now += self.widget.sample_interval.value()
        self.receive_pose(0.2, torque=0)
        self.widget._record_tick()
        self.widget.stop_recording()

        self.assertTrue(self.widget.play_recorded_motion())
        self.assertIs(self.widget.playback.owner, self.widget)
        playback_command_id = self.manager.commands[-1][0]
        self.assertEqual(self.manager.commands[-1][1], ["servo_11", "servo_12", "servo_13"])
        self.manager.command_result.emit(playback_command_id, True, "position published")
        self.assertFalse(self.widget.set_leg_torque(False))

        self.widget.leg_picker.setCurrentIndex(1)
        self.receive_both_legs(0.2, 0.5)
        self.assertTrue(self.widget.set_leg_torque(False))
        torque_command_ids = [
            command[0] for command in self.manager.commands
            if len(command) == 3 and command[2] is False
        ]
        for command_id in torque_command_ids:
            self.manager.command_result.emit(command_id, True, "torque off")
        self.assertTrue(self.widget.start_recording())
        self.assertEqual(len(self.widget.overlay_samples), 1)
        for index, first, second in ((1, 0.2, 0.6), (2, 0.3, 0.7)):
            self.now += self.widget.sample_interval.value()
            self.widget.playback.tick()
            playback_command_id = self.manager.commands[-1][0]
            self.receive_both_legs(first, second)
            self.manager.command_result.emit(
                playback_command_id, True, f"published step {index}",
            )
        self.assertFalse(self.widget.recording)
        self.assertEqual(len(self.widget.frames), 3)

        self.assertEqual(
            set(self.widget.frames[-1]["positions"]),
            {"servo_11", "servo_12", "servo_13", "servo_21", "servo_22", "servo_23"},
        )
        self.assertAlmostEqual(self.widget.frames[-1]["positions"]["servo_11"], 0.2)
        self.assertAlmostEqual(self.widget.frames[-1]["positions"]["servo_21"], 0.7)
        self.assertIsNone(self.widget.playback.owner)
        timeline = self.sequences / "layered_motion.json"
        self.assertTrue(self.widget.save_timeline(timeline))
        saved = load_timeline(timeline, self.snapshots)
        self.assertTrue(saved["loop"])
        self.assertEqual(len(saved["steps"]), 3)
        final_pose = load_snapshot_reference(saved["steps"][-1]["snapshot"], self.snapshots)
        self.assertEqual(
            set(final_pose["positions"]),
            {"servo_11", "servo_12", "servo_13", "servo_21", "servo_22", "servo_23"},
        )


if __name__ == "__main__":
    unittest.main()
