"""Single-leg closure, loaded dynamics, UI lifecycle and isolated ROS routing."""

import importlib.util
import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "desktop"))
sys.path.insert(0, str(ROOT / "ros2/saprog_leg_test"))

from PySide6.QtCore import QEvent, QObject, Signal
from PySide6.QtWidgets import QApplication
from ros_manager import RosDataManager
from ros_manager.config import LEG_TEST_STATUS
from ros_manager.wsl_bridge.runtime import dispatch_command
from ros_manager.wsl_bridge.simulation import SimulationCommands
from saprog_leg_test.controller import LegTestController
from saprog_leg_test.geometry import LinkageConfig, reference_pose
from saprog_leg_test.physics import LegPhysics
from tabs.leg_test import SingleLegTestTab

_APP = QApplication.instance() or QApplication([])


class GeometryTests(unittest.TestCase):
    def test_both_assembly_branches_close_at_every_servo_angle(self):
        for branch in (-1, 1):
            cfg = LinkageConfig.from_dict({"branch": branch})
            for index in range(101):
                q = cfg.servo_lower + (cfg.servo_upper - cfg.servo_lower) * index / 100
                pose = reference_pose(cfg, q)
                self.assertAlmostEqual(math.dist(pose["O"], pose["A"]), cfg.crank_radius)
                self.assertAlmostEqual(math.dist(pose["A"], pose["B"]), cfg.rod_length)
                self.assertAlmostEqual(math.dist(pose["D"], pose["B"]), cfg.knee_radius)

    def test_invalid_geometry_and_nonfinite_parameters_are_rejected(self):
        for settings in ({"rod_length": 0.001}, {"pivot_z": float("nan")},
                         {"kp": True}, {"servo_lower": 1},
                         {"branch": 0}, {"unknown": 1}, {"foot_load": -1}):
            with self.subTest(settings=settings), self.assertRaises(ValueError):
                LinkageConfig.from_dict(settings)

    def test_initial_servo_is_clamped_to_configured_travel(self):
        config = LinkageConfig.from_dict({"initial_servo": 3})
        self.assertEqual(config.initial_servo, config.servo_upper)

    def test_interior_toggle_is_rejected_even_when_endpoints_close(self):
        # The crank points away from D at pi/2, inside this servo interval.
        # Endpoint pin distances are 77 mm, but the interior distance is 92 mm.
        with self.assertRaisesRegex(ValueError, "full servo range"):
            LinkageConfig.from_dict({
                "servo_lower": -1, "servo_upper": 1, "initial_servo": 0,
                "crank_zero": math.pi / 2, "rod_length": 0.06,
                "knee_radius": 0.027,
            })


@unittest.skipUnless(importlib.util.find_spec("mujoco"), "Install the optional MuJoCo test dependency")
class PhysicsTests(unittest.TestCase):
    def test_only_crank_is_actuated_and_reference_pose_is_closed(self):
        physics = LegPhysics(LinkageConfig())
        self.assertEqual(physics.model.nu, 1)
        self.assertEqual(physics.model.njnt, 3)
        self.assertEqual(physics.model.neq, 1)
        self.assertLess(physics.closure_error(), 1e-8)

    def test_loaded_sweep_tracks_nonlinear_geometry_without_mimic(self):
        rig = LegTestController()
        rig.command({"action": "start", "config": {"foot_load": 1.0, "real_time_factor": 2}})
        cfg = rig.physics.config
        rig.command({"action": "sweep", "lower": cfg.servo_lower,
                     "upper": cfg.servo_upper, "period": 4})
        samples = []
        for _ in range(80):
            rig.advance(0.05)
            state = rig.state()
            self.assertFalse(state["error"], state["error"])
            self.assertLess(state["closure_error"], 1e-5)
            self.assertLessEqual(abs(state["torque"]), cfg.torque_limit)
            q = state["servo_angle"]
            expected = reference_pose(cfg, q)["beta"]
            self.assertLess(abs(state["knee_angle"] - expected), 0.001)
            samples.append(q)
        self.assertGreater(max(samples) - min(samples), 0.5)

    def test_pause_reset_and_stop_keep_lifecycle_consistent(self):
        rig = LegTestController()
        rig.command({"action": "start"})
        rig.advance(0.05)
        rig.command({"action": "pause"})
        stamp = rig.state()["sim_time"]
        rig.advance(0.05)
        self.assertEqual(rig.state()["sim_time"], stamp)
        rig.command({"action": "reset"})
        self.assertTrue(rig.state()["paused"])
        self.assertEqual(rig.state()["sim_time"], 0)
        rig.command({"action": "resume"})
        rig.advance(0.05)
        self.assertGreater(rig.state()["sim_time"], 0)
        rig.command({"action": "stop"})
        self.assertFalse(rig.state()["running"])

    def test_out_of_range_target_is_clamped_but_invalid_sweep_is_rejected(self):
        rig = LegTestController()
        rig.command({"action": "start"})
        rig.command({"action": "target", "angle": 0.6})
        rig.command({"action": "target", "angle": 2})
        self.assertEqual(rig.physics.target, rig.physics.config.servo_upper)
        for request in ({"action": "target", "angle": float("nan")},
                        {"action": "sweep", "lower": 0.8, "upper": 0.3, "period": 5}):
            with self.assertRaises(ValueError):
                rig.command(request)
            self.assertEqual(rig.physics.target, rig.physics.config.servo_upper)
            self.assertIsNone(rig.sweep)

    def test_out_of_range_sweep_endpoints_are_clamped(self):
        rig = LegTestController()
        rig.command({"action": "start"})
        rig.command({"action": "sweep", "lower": -2, "upper": 2, "period": 5})
        self.assertEqual(rig.sweep[:2], (
            rig.physics.config.servo_lower, rig.physics.config.servo_upper,
        ))

    def test_missing_physics_dependency_is_an_actionable_error(self):
        with patch.dict(sys.modules, {"mujoco": None}), self.assertRaisesRegex(RuntimeError, "ROS Python environment"):
            LegPhysics(LinkageConfig())


class FakeManager(QObject):
    message_received = Signal(str, object)
    status_changed = Signal(str, str)
    command_result = Signal(str, bool, str)

    def __init__(self):
        super().__init__()
        self.request_topic = Mock(return_value="leg-token")
        self.leg_test_command = Mock(return_value="leg-command")
        self.release = Mock()


class TabTests(unittest.TestCase):
    def setUp(self):
        self.manager = FakeManager()
        self.tab = SingleLegTestTab(self.manager)

    def tearDown(self):
        self.tab.close()
        self.tab.deleteLater()
        _APP.sendPostedEvents(None, QEvent.Type.DeferredDelete)

    def test_tab_is_lazy_and_start_passes_valid_si_geometry_to_its_own_provider(self):
        self.manager.request_topic.assert_not_called()
        self.manager.leg_test_command.assert_not_called()
        cfg = LinkageConfig.from_dict(self.tab.configuration())
        self.assertAlmostEqual(cfg.crank_radius, 0.017)
        self.assertAlmostEqual(cfg.foot_offset, math.radians(-100))
        self.tab.start()
        self.assertEqual(self.manager.request_topic.call_args.args[0], LEG_TEST_STATUS.topic)
        self.assertEqual(self.manager.request_topic.call_args.kwargs["executable"], "leg_test_node")
        self.assertEqual(self.manager.leg_test_command.call_args.args, ("start",))
        self.assertFalse(self.tab.start_button.isEnabled())
        self.manager.command_result.emit("leg-command", True, "Started")
        self.assertTrue(self.tab.pause_button.isEnabled())
        self.assertFalse(self.tab.settings_groups[0].isEnabled())

    def test_failed_start_recovers_and_status_topics_are_isolated(self):
        self.tab.start()
        self.manager.command_result.emit("unrelated", True, "ignore")
        self.assertIsNotNone(self.tab.pending)
        self.manager.command_result.emit("leg-command", False, "Linkage cannot close")
        self.assertTrue(self.tab.start_button.isEnabled())
        self.tab.receive("/saprog/simulation/status", {"data": '{"running":true}'})
        self.assertFalse(self.tab.state["running"])
        self.tab.receive(LEG_TEST_STATUS.topic, {"data": "[]"})
        self.assertFalse(self.tab.state["running"])

    def test_ui_clamps_out_of_range_motion_before_sending(self):
        self.tab.start()
        self.manager.command_result.emit("leg-command", True, "Started")
        self.manager.leg_test_command.reset_mock()
        self.tab.send("target", angle=2)
        self.manager.leg_test_command.assert_called_once_with(
            "target", angle=self.tab.active_limits[1],
        )
        self.assertAlmostEqual(self.tab.target.value(), self.tab.active_limits[1], places=4)
        self.manager.command_result.emit("leg-command", True, "accepted")
        self.manager.leg_test_command.reset_mock()
        self.tab.send("target", angle=float("nan"))
        self.manager.leg_test_command.assert_not_called()
        self.assertIn("finite number", self.tab.status.text())

    def test_start_clamps_initial_angle_to_configured_travel(self):
        self.tab.fields["initial_servo"].setValue(2.0)
        self.tab.start()
        self.assertAlmostEqual(
            self.tab.fields["initial_servo"].value(), self.tab.active_limits[1], places=4,
        )
        self.assertAlmostEqual(
            self.manager.leg_test_command.call_args.kwargs["config"]["initial_servo"],
            self.tab.active_limits[1],
        )

    def test_closing_an_active_test_requests_stop_and_releases_subscription(self):
        self.tab.start()
        self.manager.command_result.emit("leg-command", True, "Started")
        self.tab.close()
        self.manager.leg_test_command.assert_called_with("stop")
        self.manager.release.assert_called_once_with("leg-token")

    def test_profile_values_are_clamped_to_form_ranges(self):
        profile = {"version": 1, "config": self.tab.configuration()}
        profile["config"].update(crank_radius=0.05, foot_load=-1)
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.json"
            path.write_text(json.dumps(profile), encoding="utf-8")
            with patch("tabs.leg_test.QFileDialog.getOpenFileName", return_value=(str(path), "")):
                self.tab.load_profile()
        self.assertEqual(self.tab.fields["crank_radius"].value(), 50)
        self.assertEqual(self.tab.fields["foot_load"].value(), 0)

    def test_running_state_draws_real_points_and_reset_clears_history(self):
        cfg = LinkageConfig()
        points = reference_pose(cfg, cfg.initial_servo)
        state = {"running": True, "sim_time": 1, "servo_angle": 0.4,
                 "servo_target": 0.4, "knee_angle": points["beta"], "torque": 0,
                 "servo_speed": 0, "closure_error": 0,
                 "points": {name: point for name, point in points.items() if len(name) == 1}}
        state["points"]["C"] = state["points"]["B"]
        self.tab.receive(LEG_TEST_STATUS.topic, {"data": json.dumps(state)})
        self.assertEqual(len(self.tab.history), 1)
        self.tab.resize(1100, 800)
        self.tab.show()
        _APP.processEvents()
        self.assertFalse(self.tab.grab().isNull())
        self.tab.send("reset")
        self.manager.command_result.emit("leg-command", True, "Reset")
        self.assertEqual(len(self.tab.history), 0)


class RoutingTests(unittest.TestCase):
    def test_manager_queues_only_leg_test_operation(self):
        manager = RosDataManager()
        with patch.object(manager, "_ensure_worker_locked"):
            request_id = manager.leg_test_command("target", angle=0.5)
        self.assertEqual(manager._drain_commands(), [{
            "op": "leg_test", "id": request_id, "payload": {"action": "target", "angle": 0.5},
        }])
        with self.assertRaises(ValueError):
            manager.leg_test_command("publish_position")

    def test_leg_and_full_robot_use_separate_channels_and_result_tracking(self):
        node = Mock()
        bridge = SimulationCommands(node, SimpleNamespace, operation="leg_test", namespace="/saprog/leg_test")
        self.assertFalse(bridge.handle({"op": "simulation"}))
        bridge.handle({"op": "leg_test", "id": "leg", "payload": {"action": "start"}})
        self.assertEqual(node.create_publisher.call_args.args[1], "/saprog/leg_test/command")
        self.assertEqual(node.create_subscription.call_args.args[1], "/saprog/leg_test/result")
        node.create_publisher.return_value.get_subscription_count.return_value = 1
        with patch("ros_manager.wsl_bridge.simulation.emit") as emit:
            bridge.process_pending()
            emit.assert_not_called()
            bridge._result(SimpleNamespace(data='{"id":"leg","success":true,"message":"started"}'))
            emit.assert_called_once_with("command_result", id="leg", success=True, message="started")

    def test_dispatch_intercepts_leg_requests_before_hardware_handler(self):
        hardware, topics, simulation, leg = Mock(), Mock(), Mock(), Mock()
        leg.handle.return_value = True
        dispatch_command('{"op":"leg_test"}', hardware, topics, simulation, leg)
        hardware.handle.assert_not_called()
        simulation.handle.assert_not_called()
        topics.handle.assert_not_called()


if __name__ == "__main__":
    unittest.main()
