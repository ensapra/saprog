"""Player command ordering and snapshot checks without ROS or robot hardware."""

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtCore import QObject, Signal  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402
from widgets.player import StaticAnimatorWidget  # noqa: E402

ROOT = Path(__file__).resolve().parents[2]


class FakeManager(QObject):
    message_received = Signal(str, object)
    command_result = Signal(str, bool, str)

    def __init__(self):
        super().__init__()
        self.commands = []
        self.cancelled = []

    def request_topic(self, *args, **kwargs):
        return "state-token"

    def release(self, token):
        pass

    def _send(self, operation, *args):
        command_id = str(len(self.commands))
        self.commands.append((command_id, operation, args))
        return command_id

    def publish_joint_command(self, *args):
        return self._send("position", *args)

    def set_servo_torque(self, *args):
        return self._send("torque", *args)

    def release_all_torque(self):
        return self._send("release_all")

    def refresh_servo_limits(self):
        return self._send("limits")

    def cancel_commands(self, command_ids):
        self.cancelled.extend(command_ids)


class PlayerTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name)
        self.manager = FakeManager()
        self.widget = StaticAnimatorWidget(directory=str(self.directory), ros_manager=self.manager)
        self.view = self.widget.creator_view
        self.view.target_inputs["servo_11"].setText("0.1")

    def tearDown(self):
        self.widget.close()
        self.widget.deleteLater()
        self.app.processEvents()

    def acknowledge(self, command_id=None, success=True):
        command_id = command_id if command_id is not None else self.manager.commands[-1][0]
        self.manager.command_result.emit(command_id, success, "result")

    def pose(self, **updates):
        document = {"schema_version": 1, "name": "pose", "positions": {"servo_11": 0.1}}
        document.update(updates)
        path = self.directory / "pose.json"
        path.write_text(json.dumps(document), encoding="utf-8")
        return str(path)

    def test_go_to_sends_goal_and_settings_without_pre_enabling_torque(self):
        for state in (None, False, True):
            with self.subTest(torque_state=state):
                self.view.torque_states["servo_11"] = state
                self.view.speed_slider.setValue(123)
                self.view.torque_slider.setValue(234)
                self.view.go_to_buttons["servo_11"].click()
                self.assertEqual(self.manager.commands[-1][1:], (
                    "position", (["servo_11"], [0.1], 123, 234),
                ))
                count = len(self.manager.commands)
                self.acknowledge()
                self.assertEqual(len(self.manager.commands), count)

    def limits_response(self):
        return json.dumps({"servos": [
            {"servo_id": int(name[6:]), "zero_position": 512, "direction": -1,
             "raw_lower": 400, "raw_upper": 600}
            for name in self.view.target_inputs
        ]})

    def test_entering_animator_reads_limits_updates_labels_and_persists(self):
        from widgets.player.servo_limits import TICK_RADIANS

        self.widget.show()
        self.app.processEvents()
        self.assertEqual(self.manager.commands[-1][1], "limits")
        self.widget.refresh_servo_limits()
        self.assertEqual(len(self.manager.commands), 1)
        with patch("widgets.player.save_report") as save:
            self.manager.command_result.emit(
                self.manager.commands[-1][0], True, self.limits_response(),
            )
        save.assert_called_once()
        lower, upper = self.view.servo_limits["servo_11"]
        self.assertAlmostEqual(lower, -88 * TICK_RADIANS)
        self.assertAlmostEqual(upper, 112 * TICK_RADIANS)
        self.assertEqual(self.view.limit_labels["servo_11"][0].text(), "-0.45")
        self.view.target_inputs["servo_11"].setText("0.9")
        self.view.go_to_buttons["servo_11"].click()
        self.assertEqual(len(self.manager.commands), 2)
        self.assertEqual(self.manager.commands[-1][1], "position")
        self.assertAlmostEqual(self.manager.commands[-1][2][1][0], upper)
        self.widget.hide()
        self.widget.show()
        self.assertEqual(len(self.manager.commands), 3)
        self.assertEqual(self.manager.commands[-1][1], "limits")

    def test_failed_or_incomplete_limit_read_preserves_cached_bounds(self):
        previous = dict(self.view.servo_limits)
        for success, message in ((False, "servo_11 disconnected"), (True, '{"servos":[]}')):
            self.widget.refresh_servo_limits()
            with patch("widgets.player.save_report") as save:
                self.manager.command_result.emit(self.manager.commands[-1][0], success, message)
            save.assert_not_called()
            self.assertEqual(self.view.servo_limits, previous)
            self.assertIn("using stored values", self.widget.keyframes_view.status_label.text())

    def test_limit_timeout_ignores_late_read_and_can_retry(self):
        previous = dict(self.view.servo_limits)
        self.widget.refresh_servo_limits()
        command_id = self.manager.commands[-1][0]
        self.widget._limits_timed_out()
        self.assertIn(command_id, self.manager.cancelled)
        with patch("widgets.player.save_report") as save:
            self.manager.command_result.emit(command_id, True, self.limits_response())
        save.assert_not_called()
        self.assertEqual(self.view.servo_limits, previous)
        self.widget.refresh_servo_limits()
        self.assertIsNotNone(self.widget._limits_command)

    def test_save_failure_still_uses_fresh_limits_in_memory(self):
        self.widget.refresh_servo_limits()
        with patch("widgets.player.save_report", side_effect=OSError("read only")):
            self.manager.command_result.emit(
                self.manager.commands[-1][0], True, self.limits_response(),
            )
        self.assertIn("could not save", self.widget.keyframes_view.status_label.text())
        self.assertLess(self.view.servo_limits["servo_11"][1], 0.6)

    def test_entering_puppet_cancels_queued_motion_and_ignores_late_results(self):
        self.view.go_to_buttons["servo_11"].click()
        motion_id = self.manager.commands[-1][0]
        self.view.puppet_buttons["servo_11"].click()
        self.assertEqual(self.manager.cancelled, [motion_id])
        self.assertEqual(self.manager.commands[-1][1:], ("torque", ("servo_11", False)))
        self.assertFalse(self.view.puppet_buttons["servo_11"].isEnabled())
        self.acknowledge(motion_id)
        self.assertEqual(len(self.manager.commands), 2)
        self.acknowledge()
        self.assertTrue(self.view.puppet_buttons["servo_11"].isEnabled())
        self.assertTrue(self.view.puppet_buttons["servo_11"].isChecked())
        self.assertFalse(self.view.torque_states["servo_11"])

    def test_puppet_cancels_a_queued_snapshot_containing_the_joint(self):
        self.widget._play_keyframe(self.pose(positions={"servo_11": 0.1, "servo_12": 0.2}))
        motion_id = self.manager.commands[-1][0]
        self.view.puppet_buttons["servo_11"].click()
        self.assertIn(motion_id, self.manager.cancelled)

    def test_puppet_preserves_unrelated_servo_commands(self):
        self.view.go_to_buttons["servo_11"].click()
        self.view.puppet_buttons["servo_12"].click()
        self.assertFalse(self.manager.cancelled)

    def test_puppet_blocks_go_to_and_snapshot_playback(self):
        path = self.pose()
        self.view.puppet_buttons["servo_11"].click()
        self.acknowledge()
        self.widget._move_servo("servo_11")
        self.widget._play_keyframe(path)
        self.assertEqual(len(self.manager.commands), 1)
        self.assertIn("Exit Puppet Mode", self.widget.keyframes_view.status_label.text())

    def test_puppet_tracks_precise_position_and_saves_the_hand_positioned_pose(self):
        for servo, (low, high) in self.view.servo_limits.items():
            self.view.target_inputs[servo].setText(str((low + high) / 2))
        self.view.update_servo_state("servo_11", position=0.0, torque_enabled=True)
        self.view.puppet_buttons["servo_11"].click()
        self.acknowledge()
        position = 0.8123456789
        self.manager.message_received.emit("/saprog/joint_states", {
            "name": ["servo_11"], "position": [position], "effort": [0.0],
        })
        self.assertEqual(float(self.view.target_inputs["servo_11"].text()), position)
        self.assertFalse(self.view.target_inputs["servo_11"].isEnabled())
        self.view.puppet_buttons["servo_11"].click()
        self.assertTrue(self.view.target_inputs["servo_11"].isEnabled())
        self.view.name_input.setText("hand_pose")
        saved = self.view.save_targets(self.directory)
        self.assertEqual(json.loads(saved.read_text())["positions"]["servo_11"], position)
        self.assertEqual(len(self.manager.commands), 1)

    def test_failed_torque_disable_restores_editing_without_another_command(self):
        self.view.update_servo_state("servo_11", torque_enabled=True)
        self.view.puppet_buttons["servo_11"].click()
        self.acknowledge(success=False)
        self.assertFalse(self.view.puppet_buttons["servo_11"].isChecked())
        self.assertTrue(self.view.puppet_buttons["servo_11"].isEnabled())
        self.assertTrue(self.view.go_to_buttons["servo_11"].isEnabled())
        self.assertTrue(self.view.target_inputs["servo_11"].isEnabled())
        self.assertTrue(self.view.torque_states["servo_11"])
        self.assertEqual(len(self.manager.commands), 1)

    def test_synchronous_torque_failure_restores_editing(self):
        with patch.object(self.manager, "set_servo_torque", side_effect=RuntimeError("offline")):
            self.view.puppet_buttons["servo_11"].click()
        self.assertFalse(self.view.puppet_buttons["servo_11"].isChecked())
        self.assertTrue(self.view.target_inputs["servo_11"].isEnabled())
        self.assertIn("offline", self.widget.keyframes_view.status_label.text())

    def test_release_all_replaces_pending_puppet_request(self):
        self.view.puppet_buttons["servo_11"].click()
        torque_id = self.manager.commands[-1][0]
        self.view.release_button.click()
        self.acknowledge(torque_id, success=False)
        self.assertTrue(self.view.puppet_buttons["servo_11"].isChecked())
        self.acknowledge()
        self.assertTrue(self.view.puppet_buttons["servo_11"].isEnabled())
        self.assertFalse(self.view.torque_states["servo_11"])
        self.assertEqual(len(self.manager.commands), 2)

    def test_failed_release_all_restores_pending_puppet_controls(self):
        self.view.puppet_buttons["servo_11"].click()
        self.view.release_button.click()
        self.acknowledge(success=False)
        self.assertFalse(self.view.puppet_buttons["servo_11"].isChecked())
        self.assertTrue(self.view.target_inputs["servo_11"].isEnabled())

    def test_other_snapshot_units_are_rejected_without_loading_or_moving(self):
        for version in (1, 2):
            path = self.pose(schema_version=version, units="normalized")
            self.widget._load_keyframe(path)
            self.assertIn("must use radians", self.widget.keyframes_view.status_label.text())
            self.assertEqual(self.view.target_inputs["servo_11"].text(), "0.1")
            self.widget._play_keyframe(path)
            self.assertIn("must use radians", self.widget.keyframes_view.status_label.text())
        self.assertFalse(self.manager.commands)

    def test_invalid_targets_leave_loaded_fields_and_hardware_unchanged(self):
        for positions in (
            {"servo_999": 0.0}, {"servo_11": True},
            {"servo_11": 0.2, "servo_12": None},
        ):
            with self.subTest(positions=positions):
                path = self.pose(positions=positions)
                self.widget._load_keyframe(path)
                self.assertIn("Could not load", self.widget.keyframes_view.status_label.text())
                self.assertEqual(self.view.target_inputs["servo_11"].text(), "0.1")
                self.widget._play_keyframe(path)
                self.assertIn("Could not play", self.widget.keyframes_view.status_label.text())
        self.assertFalse(self.manager.commands)

    def test_out_of_range_targets_are_clamped_for_load_move_and_save(self):
        upper = self.view.servo_limits["servo_11"][1]
        path = self.pose(positions={"servo_11": 100})
        self.widget._load_keyframe(path)
        self.assertAlmostEqual(float(self.view.target_inputs["servo_11"].text()), upper)

        self.view.target_inputs["servo_11"].setText("100")
        self.view.go_to_buttons["servo_11"].click()
        self.assertEqual(self.manager.commands[-1][2][1], [upper])
        self.assertAlmostEqual(float(self.view.target_inputs["servo_11"].text()), upper)

        self.view.target_inputs["servo_11"].setText("100")
        for servo, field in self.view.target_inputs.items():
            if servo != "servo_11":
                low, high = self.view.servo_limits[servo]
                field.setText(str((low + high) / 2))
        self.view.name_input.setText("clamped_pose")
        saved = self.view.save_targets(self.directory)
        self.assertAlmostEqual(
            json.loads(saved.read_text())["positions"]["servo_11"], upper,
        )

    def test_explicit_radian_snapshot_preserves_targets(self):
        path = self.pose(schema_version=2, units="radians")
        self.widget._play_keyframe(path)
        self.assertEqual(self.manager.commands[-1][2][1], [0.1])

    def test_twoleg_snapshot_loads_and_plays_its_radian_values(self):
        path = ROOT / "snapshots" / "twoleg.json"
        positions = json.loads(path.read_text())["positions"]
        self.widget._load_keyframe(str(path))
        self.assertEqual({
            servo: float(field.text()) for servo, field in self.view.target_inputs.items()
        }, positions)
        self.widget._play_keyframe(str(path))
        joints, values, _speed, _torque = self.manager.commands[-1][2]
        self.assertEqual(dict(zip(joints, values)), positions)


if __name__ == "__main__":
    unittest.main()
