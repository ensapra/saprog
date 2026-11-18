import json
import unittest
from pathlib import Path

from saprog_desktop.snapshot_units import validate_snapshot
from saprog_desktop.timeline_storage import load_timeline

WORKSPACE = Path(__file__).resolve().parents[3]


class ProjectDataValidationTest(unittest.TestCase):
    def test_checked_in_snapshots_pass_domain_validation(self):
        snapshots = sorted((WORKSPACE / "snapshots").rglob("*.json"))
        self.assertTrue(snapshots, "No checked-in snapshots found")
        for path in snapshots:
            with self.subTest(path=path.relative_to(WORKSPACE)):
                document = json.loads(path.read_text(encoding="utf-8"))
                validate_snapshot(document)

    def test_checked_in_sequences_resolve_and_validate_all_steps(self):
        snapshot_directory = WORKSPACE / "snapshots"
        sequences = sorted((WORKSPACE / "sequences").glob("*.json"))
        self.assertTrue(sequences, "No checked-in sequences found")
        for path in sequences:
            with self.subTest(path=path.relative_to(WORKSPACE)):
                document = load_timeline(path, snapshot_directory)
                self.assertTrue(document["steps"])


if __name__ == "__main__":
    unittest.main()
