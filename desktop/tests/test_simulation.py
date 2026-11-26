"""Simulation routing and world generation without ROS, WSL or Gazebo."""

import json
import os
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "desktop"))
sys.path.insert(0, str(ROOT / "ros2/saprog_simulation"))

from limit_fixtures import prime_limits  # noqa: E402
from PySide6.QtCore import QEvent, QObject, Signal  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402
from ros_manager import RosDataManager  # noqa: E402
from ros_manager.config import SIMULATION_STATUS  # noqa: E402
from ros_manager.wsl_bridge.simulation import SimulationCommands  # noqa: E402
from saprog_simulation import session, world  # noqa: E402
from tabs.simulation import SimulationTab  # noqa: E402

_APP = QApplication.instance() or QApplication([])


class FakeManager(QObject):
    message_received = Signal(str, object)
    status_changed = Signal(str, str)
    command_result = Signal(str, bool, str)

    def __init__(self):
        super().__init__()
        self.request_topic = Mock(return_value="simulation-token")
        self.simulation_command = Mock(return_value="simulation-command")
        self.release = Mock()
        self.refresh_servo_limits = Mock(return_value="limits-read")
        self.cancel_commands = Mock()
        prime_limits(self)


class SimulationTabTests(unittest.TestCase):
    def setUp(self):
        self.manager = FakeManager()
        self.tab = SimulationTab(self.manager)

    def tearDown(self):
        self.tab.close()
        self.tab.deleteLater()
        _APP.sendPostedEvents(None, QEvent.Type.DeferredDelete)

    def test_opening_tab_starts_nothing_and_start_uses_lightweight_defaults(self):
        self.manager.request_topic.assert_not_called()
        self.manager.simulation_command.assert_not_called()
        self.assertFalse(self.tab.viewer_on_start.isChecked())
        self.tab.start()
        self.assertEqual(self.manager.request_topic.call_args.args[0], SIMULATION_STATUS.topic)
        args, options = self.manager.simulation_command.call_args
        self.assertEqual(args, ("start",))
        self.assertEqual(options["real_time_factor"], 0.5)
        self.assertEqual(options["step_size"], 0.004)
        self.assertFalse(options["viewer"])
        self.assertFalse(self.tab.start_button.isEnabled())

    def test_results_and_status_enable_pause_and_reset(self):
        self.tab.start()
        self.manager.command_result.emit("simulation-command", True, "loaded")
        self.manager.message_received.emit(SIMULATION_STATUS.topic, {"data": json.dumps({
            "running": True, "busy": False, "paused": False, "sim_time": 1.5,
            "real_time_factor": 0.49, "height": 0.12, "tilt_degrees": 4,
        })})
        self.assertTrue(self.tab.pause_button.isEnabled())
        self.assertTrue(self.tab.reset_button.isEnabled())
        self.assertIn("0.49×", self.tab.metrics.text())
        self.tab.pause()
        self.assertEqual(self.manager.simulation_command.call_args.args, ("pause",))

    def test_show_and_start_need_no_hardware_report(self):
        from servo_limits import get_servo_limits

        get_servo_limits(self.manager).invalidate("No hardware connected")
        self.tab.show()
        _APP.processEvents()
        self.tab.start()
        self.manager.refresh_servo_limits.assert_not_called()
        self.manager.request_topic.assert_called_once()
        self.assertEqual(self.manager.request_topic.call_args.args[0], SIMULATION_STATUS.topic)
        options = self.manager.simulation_command.call_args.kwargs
        self.assertNotIn("servo_bounds", options)
        self.assertEqual(len(options["bounds_profile"]["servo_bounds"]), 12)
        self.assertIn("not measured EEPROM", self.tab.bounds_source.text())

    def test_profile_is_resolved_on_desktop_and_missing_file_starts_nothing(self):
        self.tab.profile_path.setText(str(ROOT / "missing-profile.json"))
        self.tab.start()
        self.manager.request_topic.assert_not_called()
        self.manager.simulation_command.assert_not_called()

    def test_hardware_report_changes_do_not_stop_offline_simulation(self):
        from servo_limits import get_servo_limits

        self.tab.state["running"] = True
        get_servo_limits(self.manager).invalidate("Hardware disconnected")
        self.manager.simulation_command.assert_not_called()

    def test_manager_never_routes_simulation_actions_to_hardware(self):
        manager = RosDataManager()
        with patch.object(manager, "_ensure_worker_locked"):
            request_id = manager.simulation_command("pause")
        command, = manager._drain_commands()
        self.assertEqual(command, {
            "op": "simulation", "id": request_id,
            "payload": {"action": "pause"},
        })


class SimulationBridgeTests(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.bridge = SimulationCommands(self.node, SimpleNamespace)
        self.node.create_publisher.return_value.get_subscription_count.return_value = 1

    def test_waits_for_node_and_acknowledges_actual_result(self):
        publisher = self.node.create_publisher.return_value
        publisher.get_subscription_count.return_value = 0
        self.bridge.handle({"op": "simulation", "id": "a", "payload": {"action": "pause"}})
        with patch("ros_manager.wsl_bridge.simulation.emit") as emit:
            self.bridge.process_pending()
            publisher.publish.assert_not_called()
            publisher.get_subscription_count.return_value = 1
            self.bridge.process_pending()
            self.bridge.process_pending()
            publisher.publish.assert_called_once()
            self.assertEqual(json.loads(publisher.publish.call_args.args[0].data), {"id": "a", "action": "pause"})
            emit.assert_not_called()
            self.bridge._result(SimpleNamespace(data='{"id":"a","success":true,"message":"paused"}'))
            emit.assert_called_once_with("command_result", id="a", success=True, message="paused")

    def test_windows_urdf_path_is_converted_before_publication(self):
        self.bridge.handle({"op": "simulation", "id": "a", "payload": {
            "action": "start", "urdf_path": "C:/robot/robot.urdf",
        }})
        with patch("ros_manager.wsl_bridge.simulation.subprocess.run", return_value=SimpleNamespace(
            stdout="/mnt/c/robot/robot.urdf\n",
        )) as convert:
            self.bridge.process_pending()
        self.assertEqual(convert.call_args.args[0], ["wslpath", "-u", "C:/robot/robot.urdf"])
        message = self.node.create_publisher.return_value.publish.call_args.args[0]
        self.assertEqual(json.loads(message.data)["urdf_path"], "/mnt/c/robot/robot.urdf")


class WorldTests(unittest.TestCase):
    def test_world_control_retries_during_slow_physics_startup(self):
        simulation = session.GazeboSession()
        simulation.server = Mock()
        simulation.server.poll.return_value = None
        simulation.transport = Mock()
        simulation.transport.request.side_effect = [
            (False, SimpleNamespace(data=False)), (False, SimpleNamespace(data=False)),
            (True, SimpleNamespace(data=True)),
        ]
        simulation.types = {"WorldControl": SimpleNamespace, "Boolean": object}
        self.assertEqual(simulation.pause(True), "Simulation paused")
        self.assertEqual(simulation.transport.request.call_count, 3)
        self.assertTrue(simulation.options["paused"])

    def test_world_control_stops_retrying_when_shutdown_begins(self):
        simulation = session.GazeboSession()
        simulation.server = Mock()
        simulation.server.poll.return_value = None
        simulation.transport = Mock()
        simulation.types = {"WorldControl": SimpleNamespace, "Boolean": object}
        simulation.closing.set()
        with self.assertRaisesRegex(RuntimeError, "shutdown"):
            simulation.pause(False)
        simulation.transport.request.assert_not_called()

    def test_import_preserves_model_and_enables_gravity_without_sensors(self):
        source = ROOT / "urdf/urdf/URDF.urdf"
        original = source.read_bytes()
        # Simulate SDFormat's native conversion; geometry is its responsibility.
        model = ET.Element("model", name="URDF")
        ET.SubElement(model, "link", name="base")
        for joint in ET.fromstring(original).findall("joint"):
            mimic = joint.find("mimic")
            if mimic is not None:
                axis = ET.SubElement(ET.SubElement(model, "joint", name=joint.attrib["name"]), "axis")
                ET.SubElement(axis, "mimic", joint=mimic.attrib["joint"])
        converted = f"<sdf version='1.11'>{ET.tostring(model, encoding='unicode')}</sdf>"
        with tempfile.TemporaryDirectory() as directory, patch.object(
            world.subprocess, "run", return_value=SimpleNamespace(returncode=0, stdout=converted),
        ):
            from limit_fixtures import fixture_bounds

            path, joints = world.prepare_world(source, directory, servo_bounds=fixture_bounds())
            tree = ET.parse(path)
            self.assertEqual(tree.findtext("world/gravity"), "0 0 -9.81")
            self.assertEqual(tree.findtext("world/physics/max_step_size"), "0.004")
            self.assertEqual(tree.findtext("world/model[@name='saprog']/static"), "false")
            self.assertEqual(tree.findtext("world/model[@name='saprog']/link/gravity"), "true")
            self.assertEqual(len(joints), 12)
            self.assertEqual(len(tree.findall(".//mimic")), 4)
            self.assertEqual(tree.findall(".//sensor"), [])
            self.assertEqual(joints["servo_13"]["name"], "front_right_joint_13")
            prepared = ET.parse(Path(directory) / "robot.urdf")
            motor = prepared.find("joint[@name='front_right_joint_13']/limit")
            knee = prepared.find("joint[@name='front_right_leg']/limit")
            low, high = fixture_bounds()["servo_13"]
            self.assertAlmostEqual(float(motor.get("lower")), low)
            self.assertAlmostEqual(float(motor.get("upper")), high)
            self.assertAlmostEqual(float(knee.get("lower")), -2.4*high+.527179134242642)
            self.assertAlmostEqual(float(knee.get("upper")), -2.4*low+.527179134242642)
            self.assertIn("bullet-featherstone", path.read_text())
        self.assertEqual(source.read_bytes(), original)

    def test_no_saved_limits_fallback_before_gazebo_conversion(self):
        with tempfile.TemporaryDirectory() as directory, patch.object(world.subprocess, "run") as convert:
            with self.assertRaisesRegex(ValueError, "Explicit servo bounds"):
                world.prepare_world(ROOT / "urdf/urdf/URDF.urdf", directory)
            convert.assert_not_called()

    def test_invalid_performance_settings_are_rejected_before_conversion(self):
        for options in ({"step_size": float("nan")}, {"real_time_factor": 0}, {"spawn_height": -1}):
            with self.subTest(options=options), patch.object(world.subprocess, "run") as convert:
                with self.assertRaises(ValueError):
                    world.prepare_world("missing.urdf", ".", **options)
                convert.assert_not_called()

if __name__ == "__main__":
    unittest.main()
