import json
import math
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from snapshot_window import SnapshotWindow
from saprog_desktop.snapshot_units import LEG_JOINTS


class Value:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class SnapshotWindowTest(unittest.TestCase):
    def servo_app(self):
        app = SnapshotWindow.__new__(SnapshotWindow)
        app.joints = ["servo_13"]
        app.fixed, app.torque_state, app.pending = set(), {}, {}
        app.playback_state = None
        app.external_busy = lambda: False
        app.target_values = {"servo_13": Value("0.4")}
        app.target_edited = {"servo_13"}
        app.target_inputs = {"servo_13"}
        app.command_targets, app.command_wait = {}, {}
        app.positions = {}
        app.last_state = 0.0
        app.bounds = {"servo_13": (-1, 1)}
        app.current_labels = {"servo_13": Mock()}
        app.torque_buttons = {"servo_13": Mock()}
        app.manual_buttons = {}
        for widget in (*app.current_labels.values(), *app.torque_buttons.values()):
            widget._display_options = {}
        app.status = Value()
        app.moving_speed, app.torque_strength = Value(250), Value(600)
        app.dependencies = NS(JointState=lambda: NS(header=NS()))
        app.node = NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: NS(sec=1, nanosec=1))))
        app.command_pub = Mock()
        return app

    def report(self, app, torque, position=0.1):
        app._state(NS(name=["servo_13"], position=[position], effort=[torque],
                      header=NS(stamp=NS(sec=2, nanosec=0))))

    def test_fix_sends_typed_target_and_waits_for_reported_torque(self):
        app = self.servo_app()
        self.report(app, 0.0)
        app.toggle_torque("servo_13")
        message = app.command_pub.publish.call_args.args[0]
        self.assertEqual(message.name, ["servo_13"])
        self.assertEqual(message.position, [0.4])
        self.assertEqual(message.effort, [600.0])
        self.assertNotIn("servo_13", app.fixed)
        self.report(app, 1.0, 0.3)
        self.assertIn("servo_13", app.fixed)
        self.assertEqual(app.target_values["servo_13"].get(), "0.4")
        self.assertIn("Current 0.300 rad | Torque on",
                      app.current_labels["servo_13"].configure.call_args.kwargs["text"])
        self.assertEqual(app.torque_buttons["servo_13"].configure.call_args.kwargs["text"],
                         "Release")
        app.torque = Mock()
        app.toggle_torque("servo_13")
        app.torque.assert_called_once_with("servo_13", False)

    def test_telemetry_updates_during_operations_and_ack_does_not_override_it(self):
        app = self.servo_app()
        token = object()
        app.pending["servo_13"] = (token, 0, False)
        app.playback_state = {"phase": "sending"}
        self.report(app, 1.0)
        self.assertIn("servo_13", app.fixed)
        app._apply_torque_result("servo_13", False, token, "success")
        self.assertIn("servo_13", app.fixed)
        self.report(app, 0.0)
        self.assertNotIn("servo_13", app.fixed)
        self.assertEqual(app.torque_buttons["servo_13"].configure.call_args.kwargs["text"],
                         "Fix")

    def test_released_servo_follows_motion_but_preserves_input_and_loaded_targets(self):
        app = self.servo_app()
        self.report(app, 0.0, 0.1)
        self.assertEqual(app.target_values["servo_13"].get(), "0.4")
        app.target_inputs.clear()
        self.report(app, 0.0, 0.2)
        self.assertEqual(float(app.target_values["servo_13"].get()), 0.2)
        app.target_values["servo_13"].set("0.5")
        self.report(app, 0.0, 0.2)
        self.assertEqual(app.target_values["servo_13"].get(), "0.5")
        self.report(app, 1.0, 0.3)
        self.assertEqual(app.target_values["servo_13"].get(), "0.5")

    def test_fix_shows_progress_immediately_and_move_does_not_require_other_servos(self):
        app = self.servo_app()
        self.report(app, 0.0)
        app.toggle_torque("servo_13")
        self.assertEqual(app.torque_buttons["servo_13"]._display_options["text"], "Fixing...")
        self.assertEqual(app.torque_buttons["servo_13"]._display_options["state"], "disabled")
        app.target_inputs.clear()
        self.report(app, 0.0, 0.2)
        self.assertEqual(app.target_values["servo_13"].get(), "0.4")
        self.report(app, 1.0, 0.3)
        self.assertEqual(app.torque_buttons["servo_13"]._display_options["text"], "Release")
        app.target_values["servo_13"].set("0.6")
        app._target_changed("servo_13")
        self.assertEqual(app.torque_buttons["servo_13"]._display_options["text"], "Move")
        app.joints.append("servo_12")
        app.toggle_torque("servo_13")
        self.assertEqual(app.command_pub.publish.call_args.args[0].position, [0.6])
        self.assertEqual(app.command_wait, {})
        app._update_joint_buttons("servo_13")
        self.assertEqual(app.torque_buttons["servo_13"]._display_options["text"], "Release")

    def test_creator_builds_leg_panels_and_configures_each_window(self):
        import tkinter as tk
        from tkinter import ttk

        try:
            root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(str(exc))
        self.addCleanup(root.destroy)
        root.withdraw()
        dependencies = NS(
            tk=tk, ttk=ttk, AsyncParameterClient=Mock(), Trigger=Mock(),
            JointState=Mock(), SetBool=Mock(),
        )
        joints = [joint for _name, members in LEG_JOINTS for joint in members]
        config = NS(joints=joints, bounds={joint: (-1, 1) for joint in joints}, tolerance=0.05)
        with tempfile.TemporaryDirectory() as directory, patch(
            "saprog_desktop.views.snapshot_window.SnapshotConfigurationClient"
        ):
            for embedded in (False, True):
                with self.subTest(embedded=embedded):
                    container = ttk.Frame(root) if embedded else None
                    app = SnapshotWindow(
                        root, Mock(), directory, container=container, dependencies=dependencies
                    )
                    self.assertEqual(set(app.leg_frames), {name for name, _ in LEG_JOINTS})
                    app._configure(config)
                    self.assertEqual(app.joints, joints, app.status.get())
                    self.assertEqual(set(app.target_values), set(joints))
                    for name, members in LEG_JOINTS:
                        self.assertEqual(len(app.leg_frames[name].winfo_children()), len(members))
                    app.close()

    def test_creator_load_sends_every_joint_in_one_command(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pose.json"
            targets = {f"servo_{i}": i / 100 for i in range(12)}
            path.write_text(json.dumps(dict(schema_version=1, name="pose", positions=targets)))
            app = SnapshotWindow.__new__(SnapshotWindow)
            app.joints = list(targets)
            app.bounds = {joint: (-1, 1) for joint in targets}
            app.selected, app.selected_path = Value("pose"), str(path)
            app.pending, app.playback_state = {}, None
            app._complete_state = lambda: True
            app.hold_after_load, app.status = Value(False), Value()
            app.moving_speed, app.torque_strength = Value(250), Value(600)
            app.fixed, app.torque_state = set(), {}
            app.command_targets, app.command_wait = {}, {}
            app._set_playback_controls = lambda locked: None
            app.target_values = {joint: Value() for joint in targets}
            app.target_edited = set()
            app.target_tolerance = 0.05
            app.dependencies = NS(JointState=lambda: NS(header=NS()))
            app.node = NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: None, nanoseconds=1)))
            messages = []
            app.command_pub = NS(publish=messages.append)
            app.load()
            app._playback_tick()
            self.assertEqual(len(messages), 1)
            self.assertEqual(dict(zip(messages[0].name, messages[0].position)), targets)
            self.assertEqual(messages[0].effort, [600.0] * len(targets))
            self.assertAlmostEqual(messages[0].velocity[0], 250 * 0.111 * 2 * math.pi / 60)
            self.assertIsNone(app.playback_state)
            self.assertEqual(app.target_edited, set(targets))

    def test_save_uses_typed_targets_without_telemetry(self):
        with tempfile.TemporaryDirectory() as directory:
            app = SnapshotWindow.__new__(SnapshotWindow)
            app.snapshot_name, app.status = Value("pose"), Value()
            app.joints, app.bounds = ["servo_13"], {"servo_13": (-0.8, -0.3)}
            app.pending, app.playback_state = {}, None
            app.target_values = {"servo_13": Value("-.37")}
            app.positions = {"servo_13": -0.8}
            app.directory, app.refresh_files = directory, lambda: None
            app.save()
            saved = json.loads((Path(directory) / "pose.json").read_text())
            self.assertEqual(saved["schema_version"], 1)
            self.assertNotIn("units", saved)
            self.assertEqual(saved["positions"]["servo_13"], -0.37)
            app.snapshot_name.set("radian_pose")
            app.target_values["servo_13"].set("-.55")
            app.save()
            saved = json.loads((Path(directory) / "radian_pose.json").read_text())
            self.assertAlmostEqual(saved["positions"]["servo_13"], -0.55)


if __name__ == "__main__":
    unittest.main()
