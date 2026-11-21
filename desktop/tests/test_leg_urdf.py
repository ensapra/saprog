"""URDF transforms, actual servo naming, vertical reachability and local previews."""

# ruff: noqa: E402 -- configure Qt and the standalone desktop import path first.

import copy
import json
import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "desktop"))

from documents.timeline_storage import load_snapshot_reference, load_timeline
from limit_fixtures import fixture_bounds, prime_limits
from PySide6.QtCore import QEvent, QObject, Signal
from PySide6.QtWidgets import QApplication
from tabs.leg_test import create_tab
from tabs.leg_test.urdf_model import (
    DESCRIPTION,
    LEGS,
    SingleLegModel,
    origin,
    rotation,
    sequence_frames,
    vector,
)
from tabs.leg_test.urdf_tab import STATE_TOPIC, SingleLegTestTab

_APP = QApplication.instance() or QApplication([])


class FakeManager(QObject):
    message_received = Signal(str, object)
    status_changed = Signal(str, str)
    command_result = Signal(str, bool, str)

    def __init__(self):
        super().__init__()
        self.request_topic = Mock(return_value="leg-token")
        self.release = Mock()
        self.publish_joint_command = Mock()
        self.refresh_servo_limits = Mock(return_value="limits-read")
        self.cancel_commands = Mock()
        prime_limits(self)


class KinematicsTests(unittest.TestCase):
    def test_cached_joint_frames_match_direct_urdf_evaluation_for_all_legs(self):
        for leg in LEGS:
            model = SingleLegModel(DESCRIPTION / "urdf/URDF.urdf", leg)
            for crank in (.3, .55, .8):
                values = {m.servo: crank if i == 2 else .15 for i, m in enumerate(model.mappings)}
                actual, angles = model.transforms(values)
                expected = {model.root_link: np.eye(4)}
                for joint in model.joints:
                    local = origin(joint.find("origin"))
                    local[:3, :3] = local[:3, :3] @ rotation(
                        vector(joint.find("axis").get("xyz")), angles[joint.get("name")])
                    child, parent = joint.find("child").get("link"), joint.find("parent").get("link")
                    expected[child] = expected[parent] @ local
                    np.testing.assert_allclose(actual[child], expected[child], atol=1e-12)
            with patch("tabs.leg_test.urdf_model.origin", side_effect=AssertionError("reparsed URDF")):
                model.transforms(values)

    def test_analytic_jacobian_matches_finite_differences_with_calibration(self):
        for leg in LEGS:
            model = SingleLegModel(DESCRIPTION / "urdf/URDF.urdf", leg)
            model.mappings[1].gain, model.mappings[1].offset = -.8, .3
            model.mappings[2].gain, model.mappings[2].offset = .7, .15
            model.multiplier = 2.1
            values = {m.servo: .55 if i else -.12 for i, m in enumerate(model.mappings)}
            point, jacobian = model.foot_kinematics(values)
            np.testing.assert_allclose(point, model.foot_position(values), atol=1e-12)
            for index, mapping in enumerate(model.mappings):
                before, after = dict(values), dict(values)
                before[mapping.servo] -= 1e-6
                after[mapping.servo] += 1e-6
                numerical = (model.foot_position(after)-model.foot_position(before))/2e-6
                np.testing.assert_allclose(jacobian[:, index], numerical, atol=1e-9)

    def test_single_leg_has_same_meshes_and_transforms_as_full_robot_branch(self):
        full = SingleLegModel(DESCRIPTION / "urdf/URDF.urdf")
        single = SingleLegModel(DESCRIPTION / "urdf/front_right.urdf")
        pose = {"servo_11": -.12, "servo_12": .36, "servo_13": .76}
        np.testing.assert_allclose(full.foot_position(pose), single.foot_position(pose))
        self.assertEqual(len(single.joints), 4)
        self.assertEqual(len(single.links), 4)
        self.assertEqual(len(single.visuals), 4)
        self.assertNotIn("base", single.links)

    def test_logical_radians_map_once_and_knee_follows_crank(self):
        model = SingleLegModel(DESCRIPTION / "urdf/front_right.urdf")
        pose = {"servo_11": -.12, "servo_12": .36, "servo_13": .76}
        _, angles = model.transforms(pose)
        self.assertEqual(angles["front_right_11"], -.12)
        self.assertEqual(angles["front_right_joint_13"], .76)
        self.assertAlmostEqual(angles["front_right_leg"], 2.4*.76-2.112820865757358)
        model.mappings[2].gain, model.mappings[2].offset = -1, .9
        _, angles = model.transforms(pose)
        self.assertAlmostEqual(angles["front_right_joint_13"], .14)
        self.assertAlmostEqual(angles["front_right_leg"], 2.4*.14-2.112820865757358)

    def test_vertical_cycle_for_every_leg_holds_xy_within_point_one_mm(self):
        for leg in LEGS:
            with self.subTest(leg=leg):
                model = SingleLegModel(DESCRIPTION / "urdf/URDF.urdf", leg, fixture_bounds())
                seed = {m.servo: .55 if i else 0 for i, m in enumerate(model.mappings)}
                center = model.foot_position(seed)
                poses, targets = model.vertical_cycle(center, .01, seed)
                actual = np.array([model.foot_position(p) for p in poses])
                self.assertLess(np.max(np.linalg.norm(actual-targets, axis=1)), .0001)
                self.assertTrue(all(model.in_limits(p) for p in poses))
                self.assertGreater(np.ptp(actual[:, 2]), .0098)
                self.assertLess(np.max(np.ptp(actual[:, :2], axis=0)), .0002)
                self.assertTrue(any(abs(p[model.mappings[1].servo]-seed[model.mappings[1].servo]) > .005 for p in poses))

    def test_unreachable_motion_rejected(self):
        model = SingleLegModel(DESCRIPTION / "urdf/front_right.urdf", bounds=fixture_bounds())
        seed = {m.servo: .55 if i else 0 for i, m in enumerate(model.mappings)}
        with self.assertRaisesRegex(ValueError, "unreachable"):
            model.vertical_cycle([2, 0, 0], .1, seed)

    def test_referenced_sequence_uses_snapshot_positions_and_saved_durations(self):
        seed = {"servo_21": 0, "servo_22": .55, "servo_23": .55}
        frames, loop = sequence_frames(ROOT / "sequences/BackSteps_working.json", seed, seed)
        self.assertTrue(loop)
        self.assertEqual(len(frames), 4)
        self.assertAlmostEqual(frames[0][0], .2528139000060037)
        self.assertAlmostEqual(frames[0][1]["servo_23"], .4504075489017625)
        self.assertEqual(set(frames[0][1]), set(seed))


class UrdfTabTests(unittest.TestCase):
    def setUp(self):
        self.manager = FakeManager()
        self.tab = SingleLegTestTab(self.manager)

    def tearDown(self):
        self.tab.close()
        self.tab.deleteLater()
        _APP.sendPostedEvents(None, QEvent.Type.DeferredDelete)

    def test_opening_loads_cad_without_starting_ros_or_sending_commands(self):
        self.assertIsNotNone(self.tab.model)
        self.manager.request_topic.assert_not_called()
        self.manager.publish_joint_command.assert_not_called()

    def test_manual_servo_edit_updates_model_and_captured_center(self):
        initial = self.tab.model.foot_position(self.tab.values)
        self.tab.mapping_fields[1]["pose"].setValue(.75)
        self.assertEqual(self.tab.values["servo_12"], .75)
        current = self.tab.model.foot_position(self.tab.values)
        self.assertGreater(np.linalg.norm(current-initial), .001)
        self.tab.capture_center()
        np.testing.assert_allclose([f.value()/1000 for f in self.tab.center], current, atol=1e-8)

    def test_feedback_matches_names_handles_partial_frames_and_preserves_out_of_range(self):
        self.tab.connect_live()
        self.tab.receive(STATE_TOPIC, {"name": ["servo_13", "servo_11", "servo_23"],
                                      "position": [.6, -.13, 2.]})
        self.assertIn("waiting", self.tab.mode)
        self.tab.receive(STATE_TOPIC, {"name": ["servo_12"], "position": [3.]})
        self.tab.tick()
        self.assertEqual(self.tab.values, {"servo_11": -.13, "servo_12": 3., "servo_13": .6})
        self.assertIn("OUTSIDE", self.tab.metrics.text())
        self.tab.receive(STATE_TOPIC, {"name": ["servo_11"], "position": [float("nan")]})
        self.assertEqual(self.tab.values["servo_11"], -.13)
        self.tab.receive(STATE_TOPIC, {"name": None, "position": []})
        with patch("tabs.leg_test.urdf_tab.time.monotonic", return_value=1e20):
            self.tab.tick()
        self.assertIn("stale", self.tab.mode)
        self.tab.stop()
        self.manager.release.assert_called_once_with("leg-token")
        self.manager.publish_joint_command.assert_not_called()

    def test_feedback_bursts_are_coalesced_and_repeated_readings_do_not_refresh(self):
        self.tab.connect_live()
        with patch.object(self.tab, "refresh", wraps=self.tab.refresh) as refresh:
            for index in range(100):
                self.tab.receive(STATE_TOPIC, {"name": ["servo_11", "servo_12", "servo_13"],
                                              "position": [index*.001, .55, .6]})
            refresh.assert_not_called()
            self.tab.tick()
            self.assertEqual(refresh.call_count, 1)
            self.assertAlmostEqual(self.tab.values["servo_11"], .099)
            self.tab.tick()
            self.tab.receive(STATE_TOPIC, {"name": list(self.tab.values),
                                          "position": list(self.tab.values.values())})
            self.tab.tick()
            self.assertEqual(refresh.call_count, 1)

    def test_mesh_image_reused_for_repaints_and_invalidated_by_pose_or_camera(self):
        view = self.tab.view
        with patch.object(view, "draw_meshes", wraps=view.draw_meshes) as draw:
            view.grab()
            view.grab()
            self.assertEqual(draw.call_count, 1)
            view.set_pose(self.tab.model, dict(self.tab.values, servo_12=.7))
            view.grab()
            self.assertEqual(draw.call_count, 2)
            view.azimuth += 15
            view.grab()
            self.assertEqual(draw.call_count, 3)
            view.resize(view.width()+50, view.height()+50)
            view.grab()
            self.assertEqual(draw.call_count, 4)

    def test_calibration_changes_reuse_geometry_and_recompute_foot(self):
        view = self.tab.view
        meshes = view.meshes
        modified = copy.copy(self.tab.model)
        modified.foot = modified.foot + [.001, 0, 0]
        view.set_pose(modified, self.tab.values)
        self.assertIs(view.meshes, meshes)
        np.testing.assert_allclose(view._foot, modified.foot_position(self.tab.values))

    def test_preview_disconnects_feedback_and_moves_the_mesh(self):
        self.tab.connect_live()
        self.tab.preview_sequence()
        self.assertEqual(self.tab.mode, "Sequence preview")
        self.assertIsNone(self.tab.token)
        self.tab.started -= .2
        self.tab.tick()
        self.assertNotEqual(self.tab.values["servo_13"], .55)
        self.tab.resize(1250, 950)
        self.tab.show()
        _APP.processEvents()
        self.assertFalse(self.tab.grab().isNull())

    def test_vertical_preflight_rejects_invalid_path_without_starting_timer(self):
        self.tab.center[0].setValue(1000)
        self.tab.preview_vertical()
        self.assertFalse(self.tab.timer.isActive())
        self.assertIn("unreachable", self.tab.status.text())

    def test_export_roundtrips_through_existing_timeline_loader_with_only_selected_servos(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            path = root / "vertical.json"
            with patch("tabs.leg_test.urdf_tab.ROOT", root), patch(
                "tabs.leg_test.urdf_tab.QFileDialog.getSaveFileName", return_value=(str(path), "")
            ):
                self.tab.export_vertical()
            document = load_timeline(path, root / "snapshots")
            self.assertEqual(len(document["steps"]), 80)
            self.assertAlmostEqual(sum(s["duration"]["seconds"] for s in document["steps"]), 4.)
            for step in document["steps"]:
                pose = load_snapshot_reference(step["snapshot"], root / "snapshots")["positions"]
                self.assertEqual(set(pose), {"servo_11", "servo_12", "servo_13"})
                self.assertTrue(self.tab.model.in_limits(pose))
            self.manager.publish_joint_command.assert_not_called()

    def test_tuning_roundtrips_and_invalid_load_is_transactional(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.json"
            self.tab.stroke.setValue(8)
            with patch("tabs.leg_test.urdf_tab.QFileDialog.getSaveFileName", return_value=(str(path), "")):
                self.tab.save_tuning()
            self.tab.stroke.setValue(12)
            with patch("tabs.leg_test.urdf_tab.QFileDialog.getOpenFileName", return_value=(str(path), "")):
                self.tab.load_tuning()
                self.assertEqual(self.tab.stroke.value(), 8)
                data = json.loads(path.read_text())
                data["mappings"][0]["gain"] = 0
                path.write_text(json.dumps(data))
                original = self.tab.model
                self.tab.load_tuning()
                self.assertIs(self.tab.model, original)
                self.assertIn("nonzero", self.tab.status.text())

    def test_saved_tuning_has_no_limits_and_legacy_limits_are_ignored(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "tuning.json"
            with patch("tabs.leg_test.urdf_tab.QFileDialog.getSaveFileName", return_value=(str(path), "")):
                self.tab.save_tuning()
            saved = json.loads(path.read_text())
            self.assertEqual(saved["version"], 2)
            self.assertTrue(all("lower" not in row and "upper" not in row for row in saved["mappings"]))
            saved["version"] = 1
            for row in saved["mappings"]:
                row.update(lower=-100, upper=100)
            path.write_text(json.dumps(saved))
            current = dict(self.tab.limits.bounds)
            with patch("tabs.leg_test.urdf_tab.QFileDialog.getOpenFileName", return_value=(str(path), "")):
                self.tab.load_tuning()
            self.assertEqual(dict(self.tab.model.bounds), current)
            self.assertIs(self.tab.model.bounds, self.tab.limits.bounds)

    def test_factory_tab_close_releases_feedback(self):
        tab = create_tab(self.manager)
        self.assertIsInstance(tab, SingleLegTestTab)
        tab.connect_live()
        tab.close()
        self.manager.release.assert_called_with("leg-token")
        tab.deleteLater()
        _APP.sendPostedEvents(None, QEvent.Type.DeferredDelete)


if __name__ == "__main__":
    unittest.main()
