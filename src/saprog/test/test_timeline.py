import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from timeline_model import (control_groups, regroup_motion, motion_settings, save_timeline,
                            validate_timeline, timeline_rows, lifecycle_settings,
                            transition_duration, timeline_action_data)
from timeline_window import TimelineWindow


class TimelineTest(unittest.TestCase):
    def setUp(self):
        self.step = {"snapshot": {"schema_version": 1, "name": "pose",
                                  "positions": {"servo_11": 0.2, "servo_12": -0.2}},
                     "motion": {"speed": 200, "torque": 500},
                     "overrides": {"servo_12": {"speed": 100, "torque": 300}}}
        self.document = {"kind": "saprog_timeline", "schema_version": 1,
                         "steps": [self.step, copy.deepcopy(self.step)]}
        self.joints = ["servo_11", "servo_12"]
        self.bounds = {j: (-1, 1) for j in self.joints}

    def test_transition_duration_defaults_roundtrip_and_validation(self):
        self.assertEqual(transition_duration(self.step)["mode"], "until_reached")
        self.step["duration"] = {"mode": "seconds", "seconds": 2.5}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "timeline.json"
            save_timeline(path, [self.step])
            loaded = validate_timeline(json.loads(path.read_text()))
            self.assertEqual(loaded[0]["duration"], self.step["duration"])
        for duration in ({"mode": "unknown"}, {"mode": "seconds", "seconds": -1},
                         {"mode": "seconds", "seconds": float("nan")},
                         {"mode": "seconds", "seconds": True}):
            self.step["duration"] = duration
            with self.assertRaises(ValueError):
                validate_timeline(self.document)

    def test_action_goal_flattens_steps_without_speed_grouping(self):
        fixed = copy.deepcopy(self.step)
        fixed["duration"] = {"mode": "seconds", "seconds": 2.5}
        wait = {"kind": "wait", "seconds": 0.75}
        fields = timeline_action_data(
            [fixed, self.step, wait], self.joints, end_torque_enabled=False)
        self.assertEqual(fields["joint_names"], self.joints)
        self.assertEqual(fields["step_kinds"], [0, 0, 1])
        self.assertEqual(fields["duration_modes"], [1, 0, 1])
        self.assertEqual(fields["durations"], [2.5, 1.0, 0.75])
        self.assertEqual(fields["positions"], [0.2, -0.2, 0.2, -0.2, 0.0, 0.0])
        self.assertEqual(fields["speeds"], [200, 100, 200, 100, 1, 1])
        self.assertEqual(fields["torque_limits"], [500, 300, 500, 300, 1, 1])
        self.assertTrue(fields["run_end_action"])
        self.assertFalse(fields["end_torque_enabled"])

    def test_action_goal_suppresses_end_torque_for_loop_and_selected_step(self):
        loop_goal = timeline_action_data([self.step], self.joints, loop=True)
        selected_goal = timeline_action_data([self.step], self.joints, run_end_action=False)
        self.assertTrue(loop_goal["loop"])
        self.assertFalse(loop_goal["run_end_action"])
        self.assertFalse(selected_goal["run_end_action"])

    def test_lifecycle_roundtrip_and_validation(self):
        self.assertEqual(lifecycle_settings(self.document), {"on_end": "on"})
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "timeline.json"
            save_timeline(path, [self.step], "off")
            document = json.loads(path.read_text())
            validate_timeline(document)
            self.assertEqual(document["on_end"], "off")
            self.assertNotIn("on_start", document)
            self.assertEqual(lifecycle_settings(document), {"on_end": "off"})
        document["on_start"] = "invalid"
        self.assertEqual(lifecycle_settings(document), {"on_end": "off"})
        validate_timeline(document)
        document["on_end"] = "invalid"
        with self.assertRaises(ValueError):
            validate_timeline(document)

    def test_loop_setting_roundtrip_and_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "timeline.json"
            save_timeline(path, [self.step], loop=True)
            document = json.loads(path.read_text())
            validate_timeline(document)
            self.assertIs(document["loop"], True)
        for value in (1, "true", None):
            document["loop"] = value
            with self.assertRaises(ValueError):
                validate_timeline(document)

    def test_wait_roundtrip_and_row_mapping(self):
        steps = [self.step, {"kind": "wait", "seconds": 2.75}, self.step,
                 {"kind": "wait", "seconds": 0}]
        self.assertEqual(timeline_rows(steps), [(0, "transition"), (0, "snapshot"),
                         (1, "wait"), (2, "transition"), (2, "snapshot"), (3, "wait")])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "timeline.json"
            save_timeline(path, steps)
            self.assertEqual(validate_timeline(json.loads(path.read_text()), self.joints, self.bounds), steps)

    def test_invalid_wait_rejected_before_playback(self):
        for seconds in (-1, float("nan"), float("inf"), True, "2", None):
            with self.subTest(seconds=seconds):
                self.document["steps"] = [self.step, {"kind": "wait", "seconds": seconds}]
                with self.assertRaises(ValueError):
                    validate_timeline(self.document, self.joints, self.bounds)

    def test_add_wait_inserts_after_selection_and_can_be_reordered(self):
        app = TimelineWindow.__new__(TimelineWindow)
        app.steps = copy.deepcopy(self.document["steps"])
        app.selected_index = 0
        app.apply = lambda: True
        selected = []
        app.redraw = selected.append
        app.add_wait()
        self.assertEqual(app.steps[1], {"kind": "wait", "seconds": 1.0})
        self.assertEqual(selected, [1])
        app.selected_index = 1
        app.move(-1)
        self.assertEqual(app.steps[0]["kind"], "wait")

    def test_roundtrip_embeds_repeated_snapshots_and_overrides(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "timeline.json"
            save_timeline(path, self.document["steps"])
            result = validate_timeline(json.loads(path.read_text()), self.joints, self.bounds)
            self.assertEqual(result, self.document["steps"])
            result[0]["motion"]["speed"] = 10
            self.assertEqual(result[1]["motion"]["speed"], 200)

    def test_global_and_override_resolution(self):
        self.assertEqual(motion_settings(self.step, self.joints), {
            "servo_11": {"speed": 200, "torque": 500},
            "servo_12": {"speed": 100, "torque": 300}})

    def test_global_to_custom_copies_latest_values(self):
        self.step["overrides"] = {}
        self.step["motion"] = {"speed": 123, "torque": 456}
        custom = regroup_motion(self.step, "Custom Control")
        self.assertEqual(custom["control_mode"], "Custom Control")
        self.assertEqual(motion_settings(custom, self.joints), {
            j: {"speed": 123, "torque": 456} for j in self.joints})
        custom["overrides"]["servo_11"]["speed"] = 999
        self.assertEqual(custom["overrides"]["servo_12"]["speed"], 123)

    def test_dropdown_commits_current_sliders_before_switching(self):
        for mode in ("Global Control", "Leg Control"):
            with self.subTest(mode=mode):
                app = TimelineWindow.__new__(TimelineWindow)
                app.steps = [copy.deepcopy(self.step)]
                app.editor_index = 0
                app.edit_mode = mode
                app.duration_mode = SimpleNamespace(get=lambda: "Until reached")
                app.duration_seconds = SimpleNamespace(get=lambda: "1.0")
                app.mode_var = SimpleNamespace(get=lambda: "Custom Control")
                app.groups = control_groups(mode, self.joints)
                app.group_vars = {
                    label: {"speed": SimpleNamespace(get=lambda: 321),
                            "torque": SimpleNamespace(get=lambda: 654)}
                    for label in app.groups}
                app._clear_editor = lambda: None
                shown = []
                app._show_transition = shown.append
                app._change_mode()
                self.assertEqual(app.steps[0]["control_mode"], "Custom Control")
                self.assertEqual(motion_settings(app.steps[0], self.joints), {
                    j: {"speed": 321, "torque": 654} for j in self.joints})
                self.assertEqual(shown, app.steps)

    def test_global_to_leg_to_custom_preserves_leg_values(self):
        joints = [f"servo_{i}" for i in (11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43)]
        self.step["snapshot"]["positions"] = {j: 0 for j in joints}
        self.step["overrides"] = {}
        legs = regroup_motion(self.step, "Leg Control")
        self.assertTrue(all(v == self.step["motion"] for v in legs["overrides"].values()))
        groups = control_groups("Leg Control", joints)
        self.assertEqual(groups["Front left"], ["servo_31", "servo_32", "servo_33"])
        for index, names in enumerate(groups.values()):
            for j in names:
                legs["overrides"][j] = {"speed": 100 + index, "torque": 500 + index}
        custom = regroup_motion(legs, "Custom Control")
        self.assertEqual(motion_settings(custom, joints), motion_settings(legs, joints))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "timeline.json"
            save_timeline(path, [legs, custom])
            loaded = validate_timeline(json.loads(path.read_text()))
        self.assertEqual(loaded, [legs, custom])

    def test_combining_unequal_values_averages_per_group(self):
        legs = regroup_motion(self.step, "Leg Control")
        self.assertEqual(legs["overrides"]["servo_11"], {"speed": 150, "torque": 400})
        self.assertEqual(legs["overrides"]["servo_12"], {"speed": 150, "torque": 400})
        global_step = regroup_motion(self.step, "Global Control")
        self.assertEqual(global_step["motion"], {"speed": 150, "torque": 400})
        self.assertEqual(global_step["overrides"], {})

    def test_legacy_disabled_motion_is_not_silently_enabled(self):
        self.step["motion"]["enabled"] = False
        with self.assertRaisesRegex(ValueError, "torque-off"):
            validate_timeline(self.document)

    def test_rejects_invalid_motion_and_unknown_servos(self):
        for value in (0, -1, 1024, True, 1.5, float("nan")):
            with self.subTest(value=value):
                document = copy.deepcopy(self.document)
                document["steps"][1]["motion"]["speed"] = value
                with self.assertRaises(ValueError):
                    validate_timeline(document, self.joints, self.bounds)
        self.step["overrides"]["servo_99"] = {"speed": 1, "torque": 1}
        with self.assertRaises(ValueError):
            validate_timeline(self.document, self.joints, self.bounds)

    def test_later_invalid_pose_rejects_entire_timeline(self):
        self.document["steps"][1]["snapshot"]["positions"]["servo_12"] = 2
        with self.assertRaises(ValueError):
            validate_timeline(self.document, self.joints, self.bounds)

    def test_stop_requests_cancellation_from_ros_action(self):
        app = TimelineWindow.__new__(TimelineWindow)
        app.run_steps = self.document["steps"]
        app.timeline_goal_handle = SimpleNamespace(
            cancel_goal_async=lambda: SimpleNamespace(add_done_callback=lambda callback: None))
        app.cancel_when_accepted = False
        messages = []
        app.status = SimpleNamespace(set=messages.append)
        app.stop()
        self.assertIsNotNone(app.run_steps)
        self.assertIn("Cancel requested", messages[-1])

    def test_editor_construction_and_reordering(self):
        import tkinter as tk
        try:
            root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(str(exc))
        root.withdraw()
        try:
            app = TimelineWindow.__new__(TimelineWindow)
            app.root = root
            app.steps = copy.deepcopy(self.document["steps"])
            app.steps[1]["snapshot"]["name"] = "second"
            app.run_steps = app.editor_index = None
            app.edit_widgets = []
            app.dirty = False
            app._displayed_status = None
            app.status = tk.StringVar(root, value="Ready")
            app._build()
            app.redraw()
            self.assertEqual(app.sequence.size(), 4)
            self.assertTrue(app.sequence.get(0).startswith("Transition"))
            self.assertIn("Snapshot:", app.sequence.get(1))
            app.group_vars["servo_11"]["speed"].set(125)
            app.move(1)
            self.assertEqual(app.steps[1]["snapshot"]["name"], "pose")
            self.assertEqual(app.steps[1]["overrides"]["servo_11"]["speed"], 125)
            app.duplicate()
            self.assertEqual(len(app.steps), 3)
            app.remove()
            self.assertEqual(len(app.steps), 2)
            app.sequence.selection_clear(0, "end")
            app.sequence.selection_set(1)
            app.select()
            self.assertIsNone(app.editor_index)
            self.assertEqual(app.selected_index, 0)
            root.update_idletasks()
        finally:
            root.destroy()


if __name__ == "__main__":
    unittest.main()
