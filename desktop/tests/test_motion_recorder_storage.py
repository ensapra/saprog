"""Exercise recorder serialization with plain data, without creating Qt widgets."""

import copy
import json
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from documents.timeline_domain import mirrored_positions
from tabs.motion_recorder.storage import write_steps


class RecorderStorageTest(unittest.TestCase):
    def test_capture_timing_and_manual_settings_survive_serialization(self):
        entries = [
            {"positions": {"servo_11": 0.1}, "captured_at": 10.0, "recorded": True},
            {"positions": {"servo_11": 0.2}, "captured_at": 10.4, "recorded": True},
            {"kind": "wait", "seconds": 1.5},
            {"positions": {"servo_11": 0.3}, "captured_at": 20.0, "recorded": False},
        ]
        original = copy.deepcopy(entries)
        with tempfile.TemporaryDirectory() as directory:
            steps = write_steps(
                entries, Path(directory), sample_interval=0.25, speed=123, strength=456,
            )
            self.assertAlmostEqual(steps[0]["duration"]["seconds"], 0.4)
            self.assertEqual(steps[1]["duration"]["seconds"], 0.25)
            self.assertEqual(steps[2], entries[2])
            self.assertIsNot(steps[2], entries[2])
            self.assertEqual(steps[3]["duration"]["seconds"], 0.25)
            for step, entry in zip(steps, entries):
                if "positions" in entry:
                    self.assertEqual(step["motion"], {"speed": 123, "torque": 456})
                    snapshot = json.loads(Path(step["snapshot"]).read_text(encoding="utf-8"))
                    self.assertEqual(snapshot["positions"], entry["positions"])
        self.assertEqual(entries, original)

    def test_edited_mirrored_pose_preserves_settings_and_drops_removed_overrides(self):
        source = {
            "snapshot": "original.json",
            "mirror": "Horizontal",
            "motion": {"speed": 123, "torque": 456},
            "duration": {"mode": "until_reached", "seconds": 1.5},
            "overrides": {"servo_11": {"speed": 100}, "servo_21": {"speed": 200}},
        }
        positions = mirrored_positions({"servo_11": 0.2}, source["mirror"])
        entries = [{
            "positions": positions,
            "positions_changed": True,
            "timeline_step": source,
        }]
        original = copy.deepcopy(entries)
        with tempfile.TemporaryDirectory() as directory:
            steps = write_steps(
                entries, Path(directory), sample_interval=0.25, speed=999, strength=999,
            )
            saved = json.loads(Path(steps[0]["snapshot"]).read_text(encoding="utf-8"))
            self.assertEqual(mirrored_positions(saved["positions"], source["mirror"]), positions)
            self.assertEqual(steps[0]["motion"], source["motion"])
            self.assertEqual(steps[0]["duration"], source["duration"])
            self.assertEqual(steps[0]["overrides"], {"servo_11": {"speed": 100}})
        self.assertEqual(entries, original)


if __name__ == "__main__":
    unittest.main()
