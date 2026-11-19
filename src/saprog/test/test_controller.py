import tempfile
import unittest
from pathlib import Path

from saprog_desktop.views.controller import ControllerTab


class ControllerTests(unittest.TestCase):
    def test_pressed_inputs_highlight_rows_and_dispatch_only_on_press(self):
        class FakeLabel:
            def __init__(self):
                self.options = {}

            def configure(self, **options):
                self.options.update(options)

        app = ControllerTab.__new__(ControllerTab)
        first_label = FakeLabel()
        second_label = FakeLabel()
        app.input_labels = {
            "button:0": first_label,
            "button:1": second_label,
        }
        app.previous_pressed = set()
        app.settings = {"button_actions": {"button:0": "torque"}}
        calls = []
        app.on_torque = lambda: calls.append("torque")

        app._set_input_pressed("button:0", True)
        app._set_input_pressed("button:0", True)
        app._set_input_pressed("button:1", True)
        app._set_input_pressed("button:0", False)

        self.assertEqual(first_label.options["style"], "TLabel")
        self.assertEqual(
            second_label.options["style"],
            "Controller.Pressed.TLabel",
        )
        self.assertEqual(calls, ["torque"])

    def test_action_dropdown_values_and_keys(self):
        app = ControllerTab.__new__(ControllerTab)
        app.snapshot_paths = [Path("/tmp/standing.json"), Path("/tmp/sitting.json")]
        app.sequence_paths = [Path("/tmp/wave.json")]

        values = app._action_values()

        self.assertIn("Toggle torque", values)
        self.assertIn("Release all joints", values)
        self.assertIn("Set snapshot: standing", values)
        self.assertIn("Play sequence: wave", values)
        self.assertEqual(
            app._action_key_from_label("Set snapshot: sitting"),
            "snapshot:sitting.json",
        )
        self.assertEqual(
            app._action_key_from_label("Play sequence: wave"),
            "sequence:wave.json",
        )
        self.assertEqual(
            app._action_label("snapshot:standing.json"),
            "Set snapshot: standing",
        )
        self.assertEqual(app._action_key_from_label("Release all joints"), "release")
        self.assertEqual(app._action_label("release"), "Release all joints")

    def test_refresh_files_reloads_snapshots_and_sequences(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            snapshots = root / "snapshots"
            sequences = root / "sequences"
            snapshots.mkdir()
            sequences.mkdir()
            (snapshots / "standing.json").touch()
            (sequences / "wave.json").touch()

            app = ControllerTab.__new__(ControllerTab)
            app.closed = False
            app.directory = snapshots
            app.sequence_directory = sequences
            app.snapshot_paths = []
            app.sequence_paths = []
            app._build_mappings = lambda: None

            app.refresh_files()

        self.assertEqual(app.snapshot_paths, [snapshots / "standing.json"])
        self.assertEqual(app.sequence_paths, [sequences / "wave.json"])

    def test_dispatches_snapshot_sequence_and_torque_by_physical_input(self):
        app = ControllerTab.__new__(ControllerTab)
        app.snapshot_paths = [Path("/tmp/standing.json")]
        app.sequence_paths = [Path("/tmp/wave.json")]
        app.settings = {
            "button_actions": {
                "button:0": "snapshot:standing.json",
                "button:1": "sequence:wave.json",
                "button:2": "release",
                "hat:0:up": "torque",
            }
        }
        calls = []
        app.on_snapshot = lambda path: calls.append(("snapshot", path))
        app.on_sequence = lambda path: calls.append(("sequence", path))
        app.on_torque = lambda: calls.append(("torque", None))
        app.on_release = lambda: calls.append(("release", None))

        app._dispatch(0)
        app._dispatch(1)
        app._dispatch(2)
        app._dispatch("hat:0:up")

        self.assertEqual(
            calls,
            [
                ("snapshot", str(app.snapshot_paths[0])),
                ("sequence", str(app.sequence_paths[0])),
                ("release", None),
                ("torque", None),
            ],
        )

    def test_settings_migrate_old_mapping_format(self):
        with tempfile.TemporaryDirectory() as directory:
            settings_path = Path(directory) / "controller.json"
            settings_path.write_text(
                '{"mappings":{"snapshot:standing.json":2},"torque_button":8}',
                encoding="utf-8",
            )
            app = ControllerTab.__new__(ControllerTab)
            app.settings_path = settings_path
            settings = app._load_settings()

        self.assertEqual(
            settings["button_actions"],
            {"button:2": "snapshot:standing.json", "button:8": "torque"},
        )
