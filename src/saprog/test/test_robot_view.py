import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
from xml.etree import ElementTree

from saprog_desktop.robot_view_model import (
    IDENTITY_QUATERNION,
    SERVO_JOINT_NAMES,
    OrientationReference,
)
from saprog_desktop.robot_view_session import RendererSession  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]

try:
    import rclpy
except ImportError:
    rclpy = None
else:
    from saprog_desktop.ros.robot_view_bridge import RobotViewBridge
    from sensor_msgs.msg import JointState


@unittest.skipIf(rclpy is None, "bridge integration requires ROS")
class RobotViewBridgeTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = RobotViewBridge()
        self.node.joint_publisher = Mock()
        self.node.tf_broadcaster = Mock()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_startup_publishes_complete_pose_and_matching_base_stamp(self):
        self.node.publish_base_transform()
        pose = self.node.joint_publisher.publish.call_args.args[0]
        base = self.node.tf_broadcaster.sendTransform.call_args.args[0]
        self.assertEqual(pose.name, list(SERVO_JOINT_NAMES.values()))
        self.assertEqual(list(pose.position), [0.0] * 12)
        self.assertEqual(pose.header.stamp, base.header.stamp)
        self.assertAlmostEqual(base.transform.translation.z, 0.183)
        self.assertEqual(base.transform.rotation.w, 1.0)

    def test_logical_angles_pass_through_for_every_servo(self):
        for angle in (0.35, -0.35, 0.0):
            self.node.on_joints(JointState(
                name=[f"servo_{servo_id}" for servo_id in SERVO_JOINT_NAMES],
                position=[angle] * 12,
            ))
            self.node.publish_base_transform()
            pose = self.node.joint_publisher.publish.call_args.args[0]
            values = dict(zip(pose.name, pose.position))
            for servo_id, joint in SERVO_JOINT_NAMES.items():
                with self.subTest(servo_id=servo_id, angle=angle):
                    self.assertEqual(values[joint], angle)

    @unittest.skipUnless(os.name == "posix", "ROS executable entry point requires POSIX")
    def test_bridge_entry_point_executes_without_explicit_python(self):
        # Match install(PROGRAMS): preserve bytes and add executable permission.
        # Import-only tests miss absent shebangs and CRLF interpreter failures.
        with tempfile.TemporaryDirectory() as directory:
            entrypoint = Path(directory) / "robot_view_bridge"
            entrypoint.write_bytes((ROOT / "src/saprog/scripts/robot_view_bridge.py").read_bytes())
            entrypoint.chmod(0o755)
            environment = os.environ.copy()
            environment["PYTHONPATH"] = os.pathsep.join((
                str(ROOT / "src/saprog/scripts"), environment.get("PYTHONPATH", ""),
            ))
            topic = f"/test_robot_view_{os.getpid()}/joint_states"
            received = []
            subscription = self.node.create_subscription(JointState, topic, received.append, 10)
            process = subprocess.Popen(
                [str(entrypoint), "--ros-args", "-r", f"/joint_states:={topic}"],
                env=environment, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            )
            try:
                deadline = time.monotonic() + 10
                while not received and process.poll() is None and time.monotonic() < deadline:
                    rclpy.spin_once(self.node, timeout_sec=0.1)
            finally:
                process.send_signal(signal.SIGINT)
                try:
                    stdout, stderr = process.communicate(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    stdout, stderr = process.communicate(timeout=5)
                self.node.destroy_subscription(subscription)
            self.assertTrue(received, stdout + stderr)
            self.assertEqual(received[0].name, list(SERVO_JOINT_NAMES.values()))
            self.assertEqual(process.returncode, 0, stdout + stderr)

    def test_partial_readback_preserves_other_joints_and_rejects_nonfinite(self):
        self.node.on_joints(JointState(name=["servo_13"], position=[-0.2]))
        self.node.on_joints(JointState(
            name=["servo_11", "servo_13", "servo_12", "servo_99", "servo_bad", "servo_21"],
            position=[0.3, float("nan"), float("inf"), 1.0, 1.0],
        ))
        self.node.publish_base_transform()
        pose = self.node.joint_publisher.publish.call_args.args[0]
        values = dict(zip(pose.name, pose.position))
        self.assertEqual(len(values), 12)
        self.assertEqual(values["front_right_joint_13"], -0.2)
        self.assertEqual(values["front_right_11"], 0.3)
        self.assertEqual(values["front_right_12"], 0.0)
        self.assertEqual(values["back_right_21"], 0.0)


class FakeProcess:
    pid = 4321

    def __init__(self):
        self.returncode = None
        self.polls = 0

    def poll(self):
        self.polls += 1
        return self.returncode

    def wait(self, timeout=None):
        return self.returncode


class RobotViewSessionTest(unittest.TestCase):
    def test_map_resolves_every_servo_name_to_a_urdf_joint(self):
        urdf = ElementTree.parse(ROOT / "urdf/urdf/URDF.urdf").getroot()
        joints = {item.attrib["name"] for item in urdf.findall("joint")}
        self.assertEqual(len(SERVO_JOINT_NAMES), len(set(SERVO_JOINT_NAMES.values())))
        self.assertTrue(set(SERVO_JOINT_NAMES.values()) <= joints)
        self.assertEqual(SERVO_JOINT_NAMES[13], "front_right_joint_13")

    def test_urdf_owns_servo_directions_and_knee_coupling(self):
        urdf = ElementTree.parse(ROOT / "urdf/urdf/URDF.urdf").getroot()
        joints = {item.attrib["name"]: item for item in urdf.findall("joint")}
        reversed_ids = {11, 12, 13, 22, 23, 31, 33, 43}
        for servo_id, name in SERVO_JOINT_NAMES.items():
            with self.subTest(servo_id=servo_id):
                axis = tuple(map(float, joints[name].find("axis").attrib["xyz"].split()))
                self.assertEqual(axis, (1.0 if servo_id in reversed_ids else -1.0, 0.0, 0.0))
        for joint in joints.values():
            mimic = joint.find("mimic")
            if mimic is not None:
                # Reversed crank coordinates must still drive the same knee motion.
                self.assertLess(float(mimic.attrib["multiplier"]), 0.0)

    def test_orientation_resets_to_identity_after_stale_timeout_and_rejects_invalid(self):
        now = [1.0]
        reference = OrientationReference(max_age=0.5, clock=lambda: now[0])
        self.assertEqual(reference.current(), IDENTITY_QUATERNION)
        self.assertFalse(reference.update((0, 0, 0, 0)))
        self.assertTrue(reference.update((0, 0.1, 0, 0.995)))
        self.assertEqual(reference.current(), (0.0, 0.1, 0.0, 0.995))
        now[0] += 0.51
        self.assertEqual(reference.current(), IDENTITY_QUATERNION)

    @unittest.skipUnless(hasattr(os, "killpg"), "renderer process groups require POSIX")
    def test_launch_startup_readiness_stop_and_repeat_stop(self):
        with tempfile.TemporaryDirectory() as directory:
            process = FakeProcess()
            session = RendererSession(directory)
            with patch("saprog_desktop.robot_view_session.subprocess.Popen", return_value=process):
                self.assertTrue(session.start(12, 640, 480))
            marker = Path(session.ready_file)
            self.assertTrue(marker.parent.exists())
            self.assertEqual(session.poll(), "starting")
            marker.touch()
            self.assertEqual(session.poll(), "running")
            with patch("saprog_desktop.robot_view_session.os.killpg") as killpg, patch(
                "saprog_desktop.robot_view_session.threading.Timer"
            ) as timer:
                session.stop()
                session.stop()
                self.assertEqual(killpg.call_count, 1)
                self.assertEqual(timer.call_count, 1)
                process.returncode = 0
                self.assertEqual(session.poll(), "stopped")
            self.assertFalse(marker.exists())
            self.assertIsNone(session.ready_file)

    @unittest.skipUnless(hasattr(os, "killpg"), "renderer process groups require POSIX")
    def test_startup_failure_and_root_close_cleanup(self):
        session = RendererSession(tempfile.gettempdir())
        with patch("saprog_desktop.robot_view_session.subprocess.Popen", side_effect=OSError("missing ros2")):
            self.assertFalse(session.start(1, 2, 3))
        self.assertEqual(session.state, "error")
        self.assertIsNone(session.ready_file)

        process = FakeProcess()
        with patch("saprog_desktop.robot_view_session.subprocess.Popen", return_value=process):
            self.assertTrue(session.start(1, 2, 3))
        marker = session.ready_file
        with patch("saprog_desktop.robot_view_session.os.killpg") as killpg, patch(
            "saprog_desktop.robot_view_session.threading.Timer"
        ) as timer:
            session.close()
            session.close()
        self.assertTrue(session.closing)
        self.assertEqual(killpg.call_count, 1)
        self.assertEqual(timer.call_count, 1)
        self.assertFalse(os.path.exists(marker))

    @unittest.skipUnless(hasattr(os, "killpg"), "renderer process groups require POSIX")
    def test_early_child_exit_is_reported_as_startup_failure(self):
        process = FakeProcess()
        session = RendererSession(tempfile.gettempdir())
        with patch("saprog_desktop.robot_view_session.subprocess.Popen", return_value=process):
            session.start(1, 2, 3)
        process.returncode = 17
        with patch("saprog_desktop.robot_view_session.os.killpg"), patch(
            "saprog_desktop.robot_view_session.threading.Timer"
        ):
            self.assertEqual(session.poll(), "error")
        self.assertIn("failed during startup", session.message)

    @unittest.skipUnless(hasattr(os, "killpg"), "renderer process groups require POSIX")
    def test_exit_after_readiness_is_reported_as_unexpected(self):
        process = FakeProcess()
        session = RendererSession(tempfile.gettempdir())
        with patch("saprog_desktop.robot_view_session.subprocess.Popen", return_value=process):
            session.start(1, 2, 3)
        Path(session.ready_file).touch()
        self.assertEqual(session.poll(), "running")
        process.returncode = 9
        with patch("saprog_desktop.robot_view_session.os.killpg"), patch(
            "saprog_desktop.robot_view_session.threading.Timer"
        ):
            self.assertEqual(session.poll(), "error")
        self.assertIn("exited unexpectedly (code 9)", session.message)


if __name__ == "__main__":
    unittest.main()
