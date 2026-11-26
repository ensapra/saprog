"""Offline profile validation and import contract; no Gazebo installation needed."""

import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE))

from saprog_simulation import session, world  # noqa: E402
from saprog_simulation.profile import validate_bounds_profile  # noqa: E402

ROOT = PACKAGE.parents[1]


def baseline():
    return json.loads((PACKAGE / "profiles/offline-baseline.json").read_text(encoding="utf-8"))


class BoundsProfileTests(unittest.TestCase):
    def test_valid_profile_is_an_owned_copy(self):
        source = baseline()
        result = validate_bounds_profile(source)
        result["servo_bounds"]["servo_11"][0] = -1
        self.assertNotEqual(source, result)

    def test_invalid_profiles(self):
        for key, value in (
            ("schema_version", True), ("schema_version", 2), ("units", "degrees"),
            ("name", " "), ("provenance", None), ("servo_bounds", {}),
        ):
            with self.subTest(key=key, value=value):
                document = baseline()
                document[key] = value
                with self.assertRaises(ValueError):
                    validate_bounds_profile(document)
        for pair in ([0, 0], [1, -1], [False, 1], [float("nan"), 1],
                     [-1, float("inf")], [1, 2], [-1], "bad"):
            with self.subTest(pair=pair):
                document = baseline()
                document["servo_bounds"]["servo_11"] = pair
                with self.assertRaisesRegex(ValueError, "servo_11"):
                    validate_bounds_profile(document)

    def test_incomplete_extra_and_unknown_fields_are_rejected(self):
        document = baseline()
        del document["servo_bounds"]["servo_11"]
        with self.assertRaises(ValueError):
            validate_bounds_profile(document)
        document = baseline()
        document["servo_bounds"]["servo_99"] = [-1, 1]
        with self.assertRaises(ValueError):
            validate_bounds_profile(document)
        document = baseline()
        document["initial_pose"] = [1, 2, 3]
        with self.assertRaises(ValueError):
            validate_bounds_profile(document)

    def test_invalid_profile_does_not_stop_existing_session(self):
        simulation = session.GazeboSession()
        with patch.object(simulation, "stop") as stop:
            with self.assertRaises(ValueError):
                simulation.start({"bounds_profile": {}})
            stop.assert_not_called()

    def test_ambiguous_sources_rejected_before_conversion(self):
        with patch.object(world.subprocess, "run") as convert:
            with self.assertRaisesRegex(ValueError, "not both"):
                world.prepare_world("missing.urdf", ".", bounds_profile=baseline(), servo_bounds={})
            convert.assert_not_called()

    def test_profile_import_preserves_source_and_resolves_all_servos(self):
        source = ROOT / "urdf/urdf/URDF.urdf"
        original = source.read_bytes()
        model = ET.Element("model", name="robot")
        for joint in ET.fromstring(original).findall("joint"):
            mimic = joint.find("mimic")
            if mimic is not None:
                axis = ET.SubElement(ET.SubElement(model, "joint", name=joint.get("name")), "axis")
                ET.SubElement(axis, "mimic", joint=mimic.get("joint"))
        converted = f"<sdf>{ET.tostring(model, encoding='unicode')}</sdf>"
        with tempfile.TemporaryDirectory() as directory, patch.object(
            world.subprocess, "run", return_value=SimpleNamespace(returncode=0, stdout=converted),
        ):
            _, joints = world.prepare_world(source, directory, bounds_profile=baseline())
            self.assertEqual(set(joints), set(baseline()["servo_bounds"]))
            prepared = ET.parse(Path(directory) / "robot.urdf")
            self.assertAlmostEqual(float(prepared.find(
                "joint[@name='front_right_11']/limit",
            ).get("lower")), baseline()["servo_bounds"]["servo_11"][0])
        self.assertEqual(source.read_bytes(), original)

    def test_incomplete_urdf_mapping_is_rejected_before_conversion(self):
        source = ET.parse(ROOT / "urdf/urdf/URDF.urdf")
        joint = source.find("joint[@name='front_right_11']")
        joint.set("type", "fixed")
        with tempfile.TemporaryDirectory() as directory, patch.object(world.subprocess, "run") as convert:
            path = Path(directory) / "incomplete.urdf"
            source.write(path)
            with self.assertRaisesRegex(ValueError, "exactly once"):
                world.prepare_world(path, directory, bounds_profile=baseline())
            convert.assert_not_called()

    def test_start_records_owned_inputs_and_reset_reuses_them(self):
        simulation = session.GazeboSession()
        process = Mock()
        process.poll.return_value = None
        options = {"urdf_path": "robot.urdf", "bounds_profile": baseline(), "paused": True}
        with patch.object(session, "prepare_world", return_value=("world.sdf", {})) as prepare, \
                patch.object(simulation, "_launch", return_value=process), \
                patch.object(simulation, "_stop_process"), \
                patch.object(simulation, "_load_transport", side_effect=lambda: setattr(
                    simulation, "stats_at", 1,
                )), patch.object(simulation, "pause"):
            try:
                simulation.start(options)
                recorded = json.loads((Path(simulation.directory.name) / "run-inputs.json").read_text())
                self.assertEqual(recorded["bounds_profile"], baseline())
                options["bounds_profile"]["servo_bounds"]["servo_11"][0] = -1
                simulation.reset()
                self.assertEqual(prepare.call_args.kwargs["bounds_profile"], baseline())
                state = simulation.status()
                self.assertEqual(state["bounds_profile"], baseline())
                state["bounds_profile"]["servo_bounds"]["servo_11"][0] = -1
                self.assertEqual(simulation.options["bounds_profile"], baseline())
            finally:
                simulation.stop()
        self.assertIsNone(simulation.status()["bounds_profile"])


if __name__ == "__main__":
    unittest.main()
