import json
import math
from pathlib import Path
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from snapshot_units import joint_bounds, to_radians, from_radians, validate_snapshot
from snapshot_window import SnapshotWindow


class Value:
    def __init__(self, value=""):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class SnapshotUnitsTest(unittest.TestCase):
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
            app.hold_after_load, app.status = Value(True), Value()
            app.moving_speed, app.torque_strength = Value(250), Value(600)
            app.fixed, app.torque_state = set(), {}
            app._set_playback_controls = lambda locked: None
            app.target_values = {joint: Value() for joint in targets}
            app.target_edited = set()
            app.node = NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: None, nanoseconds=1)))
            messages = []
            app.command_pub = NS(publish=messages.append)
            app.load()
            with patch("snapshot_window.JointState", side_effect=lambda: NS(header=NS())):
                app._playback_tick()
            self.assertEqual(len(messages), 1)
            self.assertEqual(dict(zip(messages[0].name, messages[0].position)), targets)
            self.assertEqual(messages[0].effort, [600.0] * len(targets))
            self.assertAlmostEqual(messages[0].velocity[0], 250 * 0.111 * 2 * math.pi / 60)
            self.assertEqual(app.playback_state["phase"], "waiting")
            self.assertEqual(app.target_edited, set(targets))

    def test_asymmetric_limits_and_reversed_direction(self):
        bounds = joint_bounds([13, 33], [512, 512], [1, -1], [350, 570], [440, 660])
        for limits in bounds.values():
            for normalized in (-1, -.3, 0, .7, 1):
                radians = to_radians(normalized, limits, "normalized")
                self.assertAlmostEqual(from_radians(radians, limits, "normalized"), normalized)
            self.assertEqual(to_radians(-1, limits, "normalized"), limits[0])
            self.assertEqual(to_radians(1, limits, "normalized"), limits[1])
            self.assertAlmostEqual(to_radians(0, limits, "normalized"), sum(limits) / 2)

    def test_legacy_and_new_snapshots(self):
        bounds = {"servo_13": (-.8, -.3)}
        for version, units, value in ((1, None, -.55), (2, "normalized", 0), (2, "radians", -.55)):
            doc = dict(schema_version=version, units=units, name="pose", positions={"servo_13": value})
            self.assertAlmostEqual(validate_snapshot(doc, bounds, bounds)[1]["servo_13"], -.55)
        for value in (-1.01, 1.01, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                validate_snapshot(dict(schema_version=2, units="normalized", name="pose",
                                       positions={"servo_13": value}), bounds, bounds)
        with self.assertRaises(ValueError):
            validate_snapshot(dict(schema_version=2, name="pose", positions={"servo_13": 0}), bounds, bounds)

    def test_save_uses_typed_targets_without_telemetry(self):
        with tempfile.TemporaryDirectory() as directory:
            app = SnapshotWindow.__new__(SnapshotWindow)
            app.snapshot_name, app.status = Value("pose"), Value()
            app.joints, app.bounds = ["servo_13"], {"servo_13": (-.8, -.3)}
            app.pending, app.playback_state = {}, None
            app.target_values = {"servo_13": Value("-.37")}
            app.positions = {"servo_13": -.8}
            app.directory, app.refresh_files = directory, lambda: None
            app.save()
            saved = json.loads((Path(directory) / "pose.json").read_text())
            self.assertEqual(saved["schema_version"], 1)
            self.assertNotIn("units", saved)
            self.assertEqual(saved["positions"]["servo_13"], -.37)
            app.snapshot_name.set("radian_pose")
            app.target_values["servo_13"].set("-.55")
            app.save()
            saved = json.loads((Path(directory) / "radian_pose.json").read_text())
            self.assertAlmostEqual(saved["positions"]["servo_13"], -.55)


if __name__ == "__main__":
    unittest.main()
