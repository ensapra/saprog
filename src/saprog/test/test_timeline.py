import copy
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import timeline_window as timeline_window_module
from saprog_desktop.timeline_presenter import TimelineEditorPresenter
from timeline_model import (
    control_groups,
    lifecycle_settings,
    load_snapshot_reference,
    motion_settings,
    reference_for_legacy_snapshot,
    regroup_motion,
    resolve_timeline_steps,
    save_timeline,
    timeline_action_data,
    timeline_rows,
    transition_duration,
    validate_timeline,
)
from timeline_window import TimelineWindow


class FakeFuture:
    def __init__(self, value=None, error=None):
        self.value = value
        self.error = error
        self.callbacks = []

    def add_done_callback(self, callback):
        self.callbacks.append(callback)

    def complete(self, value=None, error=None):
        self.value, self.error = value, error
        for callback in self.callbacks:
            callback(self)

    def result(self):
        if self.error:
            raise self.error
        return self.value


class FakeRoot:
    def __init__(self):
        self.callbacks = {}
        self.next_id = 0

    def after(self, _milliseconds, callback):
        self.next_id += 1
        callback_id = f"after-{self.next_id}"
        self.callbacks[callback_id] = callback
        return callback_id

    def after_cancel(self, callback_id):
        self.callbacks.pop(callback_id, None)

    def fire(self, callback_id):
        self.callbacks.pop(callback_id)()


class TimelineTest(unittest.TestCase):
    def setUp(self):
        self.step = {
            "snapshot": {
                "schema_version": 1,
                "name": "pose",
                "positions": {"servo_11": 0.2, "servo_12": -0.2},
            },
            "motion": {"speed": 200, "torque": 500},
            "overrides": {"servo_12": {"speed": 100, "torque": 300}},
        }
        self.document = {
            "kind": "saprog_timeline",
            "schema_version": 1,
            "steps": [self.step, copy.deepcopy(self.step)],
        }
        self.joints = ["servo_11", "servo_12"]
        self.bounds = {j: (-1, 1) for j in self.joints}

    def action_window(self):
        app = TimelineWindow.__new__(TimelineWindow)
        app.root = FakeRoot()
        app.run_steps = [copy.deepcopy(self.step)]
        app.run_index = 0
        app.timeline_session_id = 1
        app.timeline_timeout_ms = 8000
        app.timeline_timeout_callbacks = {}
        app.timeline_state = "goal_pending"
        app.timeline_goal_future = None
        app.timeline_goal_handle = None
        app.timeline_result_future = None
        app.cancel_when_accepted = False
        app.timeline_close_state = "open"
        app.status = SimpleNamespace(messages=[], set=lambda value: app.status.messages.append(value))
        app._update_buttons = lambda: None
        app._close_view = lambda: setattr(app, "closed", True)
        app.closed = False
        return app

    def reference_steps(self, directory, steps):
        referenced = copy.deepcopy(steps)
        for step in referenced:
            if step.get("kind") == "wait":
                continue
            snapshot = step["snapshot"]
            if isinstance(snapshot, dict):
                filename = snapshot["name"] + ".json"
                (directory / filename).write_text(json.dumps(snapshot))
                step["snapshot"] = filename
        return referenced

    def resolver(self, directory):
        return lambda reference: load_snapshot_reference(reference, directory)

    def test_transition_duration_defaults_roundtrip_and_validation(self):
        self.assertEqual(transition_duration(self.step)["mode"], "until_reached")
        self.step["duration"] = {"mode": "seconds", "seconds": 2.5}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "timeline.json"
            root = Path(directory)
            steps = self.reference_steps(root, [self.step])
            resolver = self.resolver(root)
            save_timeline(path, steps, snapshot_resolver=resolver)
            loaded = validate_timeline(json.loads(path.read_text()), snapshot_resolver=resolver)
            self.assertEqual(loaded[0]["duration"], self.step["duration"])
        for duration in (
            {"mode": "unknown"},
            {"mode": "seconds", "seconds": -1},
            {"mode": "seconds", "seconds": float("nan")},
            {"mode": "seconds", "seconds": True},
        ):
            self.step["duration"] = duration
            with self.assertRaises(ValueError):
                validate_timeline(self.document)

    def test_action_goal_flattens_steps_without_speed_grouping(self):
        fixed = copy.deepcopy(self.step)
        fixed["duration"] = {"mode": "seconds", "seconds": 2.5}
        wait = {"kind": "wait", "seconds": 0.75}
        fields = timeline_action_data(
            [fixed, self.step, wait], self.joints, end_torque_enabled=False
        )
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
            root = Path(directory)
            path = root / "timeline.json"
            steps = self.reference_steps(root, [self.step])
            resolver = self.resolver(root)
            save_timeline(path, steps, "off", snapshot_resolver=resolver)
            document = json.loads(path.read_text())
            validate_timeline(document, snapshot_resolver=resolver)
            self.assertEqual(document["on_end"], "off")
            self.assertNotIn("on_start", document)
            self.assertEqual(lifecycle_settings(document), {"on_end": "off"})
        document["on_start"] = "invalid"
        self.assertEqual(lifecycle_settings(document), {"on_end": "off"})
        document["on_end"] = "invalid"
        with self.assertRaises(ValueError):
            validate_timeline(document)

    def test_loop_setting_roundtrip_and_validation(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "timeline.json"
            steps = self.reference_steps(root, [self.step])
            resolver = self.resolver(root)
            save_timeline(path, steps, loop=True, snapshot_resolver=resolver)
            document = json.loads(path.read_text())
            validate_timeline(document, snapshot_resolver=resolver)
            self.assertIs(document["loop"], True)
        for value in (1, "true", None):
            document["loop"] = value
            with self.assertRaises(ValueError):
                validate_timeline(document, snapshot_resolver=resolver)

    def test_wait_roundtrip_and_row_mapping(self):
        steps = [
            self.step,
            {"kind": "wait", "seconds": 2.75},
            self.step,
            {"kind": "wait", "seconds": 0},
        ]
        self.assertEqual(
            timeline_rows(steps),
            [
                (0, "transition"),
                (0, "snapshot"),
                (1, "wait"),
                (2, "transition"),
                (2, "snapshot"),
                (3, "wait"),
            ],
        )
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "timeline.json"
            referenced = self.reference_steps(root, steps)
            resolver = self.resolver(root)
            save_timeline(path, referenced, snapshot_resolver=resolver)
            self.assertEqual(
                validate_timeline(json.loads(path.read_text()), self.joints, self.bounds, resolver),
                referenced,
            )

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

    def test_editor_presenter_moves_whole_steps_and_duplicates_deeply(self):
        presenter = TimelineEditorPresenter([self.step, {"kind": "wait", "seconds": 1.0}])
        duplicate_index = presenter.duplicate(0)
        self.assertEqual(duplicate_index, 1)
        presenter.steps[1]["motion"]["speed"] = 77
        self.assertEqual(presenter.steps[0]["motion"]["speed"], 200)
        moved_index = presenter.move(1, 1)
        self.assertEqual(moved_index, 2)
        self.assertEqual(presenter.steps[2]["snapshot"], self.step["snapshot"])
        self.assertEqual(presenter.steps[1]["kind"], "wait")
        self.assertTrue(presenter.dirty)

    def test_failed_edit_restores_selection_to_transition_pair(self):
        app = TimelineWindow.__new__(TimelineWindow)
        app.run_steps = None
        app.steps = [copy.deepcopy(self.step), copy.deepcopy(self.step)]
        app.rows = [(0, "transition"), (0, "snapshot"), (1, "transition"), (1, "snapshot")]
        app.editor_index = app.selected_index = 0
        app.sequence = SimpleNamespace(
            curselection=lambda: (2,),
            selection_clear=lambda *_args: setattr(app, "cleared", True),
            selection_set=lambda row: setattr(app, "restored_row", row),
        )
        app.apply = lambda: False

        app.select()

        self.assertTrue(app.cleared)
        self.assertEqual(app.restored_row, 0)
        self.assertEqual(app.editor_index, 0)
        self.assertEqual(app.selected_index, 0)

    def test_discard_prompt_preserves_cancel_and_accepts_confirmed_discard(self):
        answers = iter((False, True))
        app = TimelineWindow.__new__(TimelineWindow)
        app.dirty = True
        app.root = object()
        app.apply = lambda: True
        app.dependencies = SimpleNamespace(
            messagebox=SimpleNamespace(askyesno=lambda *_args, **_kwargs: next(answers))
        )

        self.assertFalse(app.discard())
        self.assertTrue(app.discard())

    def test_running_action_locks_editor_and_playback_controls(self):
        changes = []
        original = timeline_window_module.configure_changed
        timeline_window_module.configure_changed = lambda widget, **options: changes.append((widget, options))
        try:
            app = TimelineWindow.__new__(TimelineWindow)
            widgets = [object(), object()]
            app.run_steps = [self.step]
            app.edit_widgets = [widgets[0]]
            app.transition_widgets = [widgets[1]]
            app.dependencies = SimpleNamespace(ttk=SimpleNamespace(Combobox=type("Combo", (), {})))
            app.sequence = SimpleNamespace(configure=lambda **options: changes.append(("sequence", options)))
            app.play_button, app.test_button = object(), object()
            app._ready = lambda: True
            app.steps = [self.step]

            app._update_buttons()

            self.assertTrue(all(options["state"] == "disabled" for _, options in changes))
        finally:
            timeline_window_module.configure_changed = original

    def test_roundtrip_stores_references_and_uses_updated_snapshot(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "timeline.json"
            steps = self.reference_steps(root, self.document["steps"])
            resolver = self.resolver(root)
            save_timeline(path, steps, snapshot_resolver=resolver)
            document = json.loads(path.read_text())
            self.assertEqual(document["steps"][0]["snapshot"], "pose.json")
            self.assertNotIn("positions", document["steps"][0]["snapshot"])
            updated = copy.deepcopy(self.step["snapshot"])
            updated["positions"]["servo_11"] = 0.8
            (root / "pose.json").write_text(json.dumps(updated))
            stored_steps = validate_timeline(document, self.joints, self.bounds, resolver)
            result = resolve_timeline_steps(stored_steps, resolver)
            self.assertEqual(result[0]["snapshot"]["positions"]["servo_11"], 0.8)
            result[0]["motion"]["speed"] = 10
            self.assertEqual(result[1]["motion"]["speed"], 200)

    def test_add_snapshot_stores_only_its_file_reference(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "pose.json"
            path.write_text(json.dumps(self.step["snapshot"]))
            app = TimelineWindow.__new__(TimelineWindow)
            app.directory = directory
            app.joints = []
            app.bounds = {}
            app.steps = []
            app.dirty = False
            app.apply = lambda: True
            app.redraw = lambda _index: None
            app.add(path)
            self.assertEqual(app.steps[0]["snapshot"], "pose.json")
            self.assertNotIn("positions", app.steps[0]["snapshot"])

    def test_legacy_migration_matches_pose_not_name(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            other = copy.deepcopy(self.step["snapshot"])
            other["positions"]["servo_11"] = 0.7
            (root / "a-different-pose.json").write_text(json.dumps(other))
            (root / "z-same-pose.json").write_text(json.dumps(self.step["snapshot"]))
            (root / "broken.json").write_text("not json")
            self.assertEqual(
                reference_for_legacy_snapshot(self.step["snapshot"], root, self.joints, self.bounds),
                "z-same-pose.json",
            )

    def test_legacy_migration_accepts_identical_duplicate_poses(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for filename in ("second.json", "first.json"):
                (root / filename).write_text(json.dumps(self.step["snapshot"]))
            self.assertEqual(
                reference_for_legacy_snapshot(self.step["snapshot"], root, self.joints, self.bounds),
                "first.json",
            )

    def test_failed_load_restores_editor_state_after_discard_applies_form(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "broken.json"
            path.write_text("not json")
            app = TimelineWindow.__new__(TimelineWindow)
            app.steps = [copy.deepcopy(self.step)]
            original = copy.deepcopy(app.steps)
            app.dirty = False
            app.directory = directory
            app.root = object()
            app.joints = self.joints
            app.bounds = self.bounds
            app.discard = lambda: (app.steps[0].update(motion={"speed": 99, "torque": 99}) or True)
            app.dependencies = SimpleNamespace(
                filedialog=SimpleNamespace(askopenfilename=lambda **_kwargs: str(path))
            )
            app.status = SimpleNamespace(set=lambda _value: None)

            app.load()

            self.assertEqual(app.steps, original)
            self.assertFalse(app.dirty)

    def test_goal_response_timeout_keeps_lock_and_accepts_late_identity(self):
        app = self.action_window()
        app._bound_timeline_wait("goal", 1)
        app.root.fire(app.timeline_timeout_callbacks["goal"])
        self.assertEqual(app.timeline_state, "goal_unknown")
        self.assertIsNotNone(app.run_steps)

        result = FakeFuture()
        handle = SimpleNamespace(accepted=True, get_result_async=lambda: result)
        app._timeline_goal_response(SimpleNamespace(result=lambda: handle), 1)
        self.assertEqual(app.timeline_state, "active")
        self.assertIsNotNone(app.run_steps)
        self.assertIn("result", app.timeline_timeout_callbacks)

    def test_synchronous_goal_submission_failure_unlocks_without_backend_goal(self):
        app = self.action_window()
        app.run_steps = None
        app._ready = lambda: True
        app.apply = lambda: True
        app.steps = [copy.deepcopy(self.step)]
        app.joints = self.joints
        app.bounds = self.bounds
        app.loop = SimpleNamespace(get=lambda: False)
        app.on_end = SimpleNamespace(get=lambda: "Torque on")
        app.selected_index = 0
        app.dependencies = SimpleNamespace(PlayTimeline=SimpleNamespace(Goal=type("Goal", (), {})))
        app.timeline_client = SimpleNamespace(
            send_goal_async=lambda *_args, **_kwargs: (_ for _ in ()).throw(RuntimeError("submit failed"))
        )

        app.play_timeline()

        self.assertIsNone(app.run_steps)
        self.assertEqual(app.timeline_state, "idle")
        self.assertIn("submit failed", app.status.messages[-1])

    def test_goal_response_error_and_rejection_are_distinct(self):
        app = self.action_window()
        app._timeline_goal_response(SimpleNamespace(result=lambda: (_ for _ in ()).throw(RuntimeError("lost"))), 1)
        self.assertEqual(app.timeline_state, "goal_unknown")
        self.assertIsNotNone(app.run_steps)

        app = self.action_window()
        app._timeline_goal_response(SimpleNamespace(result=lambda: SimpleNamespace(accepted=False)), 1)
        self.assertEqual(app.timeline_state, "idle")
        self.assertIsNone(app.run_steps)
        self.assertIn("rejected", app.status.messages[-1])

    def test_stop_before_goal_acceptance_cancels_only_after_acceptance(self):
        app = self.action_window()
        app.timeline_client = SimpleNamespace(server_is_ready=lambda: True)
        app.cancel_when_accepted = False
        app._request_timeline_cancel()
        self.assertTrue(app.cancel_when_accepted)
        cancel_future = FakeFuture()
        result_future = FakeFuture()
        handle = SimpleNamespace(
            accepted=True,
            get_result_async=lambda: result_future,
            cancel_goal_async=lambda: cancel_future,
        )
        app._timeline_goal_response(SimpleNamespace(result=lambda: handle), 1)
        self.assertEqual(app.timeline_state, "cancel_pending")
        self.assertIsNotNone(app.run_steps)
        cancel_future.complete(SimpleNamespace(goals_canceling=[object()]))
        self.assertEqual(app.timeline_state, "cancel_acknowledged")
        self.assertIsNotNone(app.run_steps)

    def test_cancel_rejection_timeout_and_terminal_result(self):
        app = self.action_window()
        result_future = FakeFuture()
        cancel_future = FakeFuture()
        app.timeline_goal_handle = SimpleNamespace(cancel_goal_async=lambda: cancel_future)
        app.timeline_state = "active"
        app._request_timeline_cancel()
        cancel_future.complete(SimpleNamespace(goals_canceling=[]))
        self.assertEqual(app.timeline_state, "active")
        self.assertIsNotNone(app.run_steps)

        app._request_timeline_cancel()
        app.root.fire(app.timeline_timeout_callbacks["cancel"])
        self.assertEqual(app.timeline_state, "cancel_unknown")
        self.assertIsNotNone(app.run_steps)

        app.timeline_state = "active"
        app.timeline_result_future = result_future
        app._timeline_result(
            SimpleNamespace(result=lambda: SimpleNamespace(
                status=5, result=SimpleNamespace(success=False, message="Timeline canceled")
            )),
            1,
        )
        self.assertEqual(app.timeline_state, "idle")
        self.assertIn("canceled", app.status.messages[-1].lower())

    def test_result_timeout_does_not_unlock_long_or_looped_goal(self):
        app = self.action_window()
        app.run_steps = [copy.deepcopy(self.step)]
        app.looping = True
        app.timeline_state = "active"
        app._bound_timeline_wait("result", 1)
        app.root.fire(app.timeline_timeout_callbacks["result"])
        self.assertEqual(app.timeline_state, "result_unknown")
        self.assertIsNotNone(app.run_steps)
        self.assertTrue(app.looping)
        self.assertIn("No execution deadline", app.status.messages[-1])
        app._timeline_result(
            SimpleNamespace(result=lambda: SimpleNamespace(
                status=4, result=SimpleNamespace(success=True, message="Loop stopped")
            )),
            1,
        )
        self.assertIsNone(app.run_steps)

    def test_result_wins_over_late_cancel_and_old_session_callbacks(self):
        app = self.action_window()
        app.timeline_state = "cancel_pending"
        app._timeline_result(
            SimpleNamespace(result=lambda: SimpleNamespace(
                status=4, result=SimpleNamespace(success=True, message="Timeline completed")
            )),
            1,
        )
        self.assertIsNone(app.run_steps)
        app.run_steps = [copy.deepcopy(self.step)]
        app.timeline_session_id = 3
        app.timeline_state = "active"
        app._timeline_cancel_response(SimpleNamespace(result=lambda: SimpleNamespace(goals_canceling=[])), 1)
        app._timeline_feedback(
            SimpleNamespace(feedback=SimpleNamespace(current_step=1, step_count=1, status="stale", seconds_remaining=0)),
            1,
        )
        self.assertIsNotNone(app.run_steps)
        self.assertEqual(app.timeline_state, "active")
        self.assertFalse(any("stale" in message for message in app.status.messages))

    def test_close_waits_for_cancel_result(self):
        app = self.action_window()
        app.root = FakeRoot()
        app.dirty = False
        app.discard = lambda: True
        app.dependencies = SimpleNamespace(messagebox=SimpleNamespace(askyesno=lambda *args, **kwargs: True))
        close_calls = []
        app.configuration = SimpleNamespace(close=lambda: close_calls.append("configuration"))
        app.configuration_after_id = app.refresh_after_id = None
        app.on_close = lambda: close_calls.append("view")
        app.view_closed = False
        app._close_view = TimelineWindow._close_view.__get__(app)
        cancel_future = FakeFuture()
        app.timeline_goal_handle = SimpleNamespace(cancel_goal_async=lambda: cancel_future)
        app.timeline_state = "active"
        app.close()
        self.assertEqual(app.timeline_close_state, "pending")
        self.assertFalse(app.closed)
        cancel_future.complete(SimpleNamespace(goals_canceling=[object()]))
        self.assertFalse(app.closed)
        app._timeline_result(
            SimpleNamespace(result=lambda: SimpleNamespace(
                status=5, result=SimpleNamespace(success=False, message="Timeline canceled")
            )),
            1,
        )
        self.assertTrue(app.view_closed)
        app._close_view()
        app.close()
        self.assertEqual(close_calls, ["configuration", "view"])


    def test_legacy_migration_rejects_missing_or_invalid_snapshots(self):
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(ValueError, "No matching snapshot"):
                reference_for_legacy_snapshot(self.step["snapshot"], directory, self.joints, self.bounds)
            invalid = copy.deepcopy(self.step["snapshot"])
            invalid["positions"]["servo_11"] = 4
            with self.assertRaises(ValueError):
                reference_for_legacy_snapshot(invalid, directory, self.joints, self.bounds)

    def test_legacy_normalized_migration_requires_measured_limits(self):
        legacy = {
            "schema_version": 2,
            "units": "normalized",
            "name": "pose",
            "positions": {"servo_11": 0, "servo_12": 0},
        }
        with tempfile.TemporaryDirectory() as directory:
            Path(directory, "pose.json").write_text(json.dumps(legacy))
            with self.assertRaisesRegex(ValueError, "hardware joint limits"):
                reference_for_legacy_snapshot(legacy, directory)
            self.assertEqual(
                reference_for_legacy_snapshot(legacy, directory, self.joints, self.bounds),
                "pose.json",
            )

    def test_global_and_override_resolution(self):
        self.assertEqual(
            motion_settings(self.step, self.joints),
            {"servo_11": {"speed": 200, "torque": 500}, "servo_12": {"speed": 100, "torque": 300}},
        )

    def test_global_to_custom_copies_latest_values(self):
        self.step["overrides"] = {}
        self.step["motion"] = {"speed": 123, "torque": 456}
        custom = regroup_motion(self.step, "Custom Control")
        self.assertEqual(custom["control_mode"], "Custom Control")
        self.assertEqual(
            motion_settings(custom, self.joints),
            {j: {"speed": 123, "torque": 456} for j in self.joints},
        )
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
                    label: {
                        "speed": SimpleNamespace(get=lambda: 321),
                        "torque": SimpleNamespace(get=lambda: 654),
                    }
                    for label in app.groups
                }
                app._clear_editor = lambda: None
                shown = []
                app._show_transition = shown.append
                app._change_mode()
                self.assertEqual(app.steps[0]["control_mode"], "Custom Control")
                self.assertEqual(
                    motion_settings(app.steps[0], self.joints),
                    {j: {"speed": 321, "torque": 654} for j in self.joints},
                )
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
            root = Path(directory)
            path = root / "timeline.json"
            steps = self.reference_steps(root, [legs, custom])
            resolver = self.resolver(root)
            save_timeline(path, steps, snapshot_resolver=resolver)
            loaded = validate_timeline(json.loads(path.read_text()), snapshot_resolver=resolver)
        self.assertEqual(loaded, steps)

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
        app = self.action_window()
        app.timeline_state = "active"
        app.timeline_goal_handle = SimpleNamespace(
            cancel_goal_async=lambda: FakeFuture()
        )
        app.stop()
        self.assertIsNotNone(app.run_steps)
        self.assertIn("Cancellation requested", app.status.messages[-1])

    def test_editor_construction_and_reordering(self):
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk

        try:
            root = tk.Tk()
        except tk.TclError as exc:
            self.skipTest(str(exc))
        root.withdraw()
        try:
            app = TimelineWindow.__new__(TimelineWindow)
            app.root = root
            app.container = root
            app.directory = str(Path.cwd())
            app.dependencies = SimpleNamespace(tk=tk, ttk=ttk, filedialog=filedialog, messagebox=messagebox)
            app.joints = self.joints
            app.bounds = self.bounds
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
