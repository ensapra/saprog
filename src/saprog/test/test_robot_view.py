import os
import json
import math
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
    from sensor_msgs.msg import Imu, JointState
    from visualization_msgs.msg import Marker


@unittest.skipIf(rclpy is None, "bridge integration requires ROS")
class RobotViewBridgeTest(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = RobotViewBridge()
        self.node.joint_publisher = Mock()
        self.node.tf_broadcaster = Mock()
        self.node.compass_publisher = Mock()

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
        self.assertAlmostEqual(base.transform.rotation.x, math.sqrt(0.5))
        self.assertAlmostEqual(base.transform.rotation.y, math.sqrt(0.5))
        self.assertEqual(base.transform.rotation.z, 0.0)
        self.assertEqual(base.transform.rotation.w, 0.0)

    def test_absolute_magnetic_yaw_rotates_world_to_base(self):
        message = Imu()
        message.orientation.z = math.sin(math.pi / 4)
        message.orientation.w = math.cos(math.pi / 4)
        message.orientation_covariance[0] = 0.0
        self.node.on_orientation(message)
        self.node.publish_base_transform()
        rotation = self.node.tf_broadcaster.sendTransform.call_args.args[0].transform.rotation
        self.assertAlmostEqual(rotation.x, 0.0)
        self.assertAlmostEqual(rotation.y, 1.0)
        self.assertAlmostEqual(rotation.z, 0.0)
        self.assertAlmostEqual(rotation.w, 0.0)
        markers = self.node.compass_publisher.publish.call_args.args[0].markers
        north = next(marker for marker in markers if marker.id == 1)
        self.assertEqual(north.pose.orientation.w, 1.0)
        self.assertEqual(north.type, Marker.ARROW)
        self.assertEqual(north.scale.z, 0.008)
        self.assertEqual(next(marker for marker in markers if marker.id == 0).action,
                         Marker.DELETE)
        self.assertEqual(next(marker for marker in markers if marker.id == 2).action,
                         Marker.DELETE)

    def test_crank_endpoints_and_other_joint_angles(self):
        for angle in (0.8803420273988994, 0.2661499152601324, 0.5):
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
            name=["servo_11", "servo_13", "servo_12", "servo_99", "servo_bad", "servo_21", "servo_42", "servo_33"],
            position=[0.3, float("nan"), float("inf"), 1.0, 1.0, 0.0, 0.4, 0.5],
        ))
        self.node.on_joints(JointState(
            name=["servo_12", "servo_21", "servo_22", "servo_23", "servo_31", "servo_43"],
            position=[0.2, 0.6, 0.4, -0.7, 0.8, -0.9],
        ))
        self.node.publish_base_transform()
        pose = self.node.joint_publisher.publish.call_args.args[0]
        values = dict(zip(pose.name, pose.position))
        self.assertEqual(len(values), 12)
        self.assertEqual(values["front_right_joint_13"], -0.2)
        self.assertEqual(values["front_right_11"], 0.3)
        self.assertEqual(values["front_right_12"], 0.2)
        self.assertEqual(values["back_right_21"], 0.6)
        self.assertEqual(values["back_right_22"], 0.4)
        self.assertEqual(values["back_right_joint_23"], -0.7)
        self.assertEqual(values["front_left_31"], 0.8)
        self.assertEqual(values["back_left_42"], 0.4)
        self.assertEqual(values["front_left_joint_33"], 0.5)
        self.assertEqual(values["back_left_joint_43"], -0.9)


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
        reversed_ids = {11, 12, 22, 31}
        for servo_id, name in SERVO_JOINT_NAMES.items():
            with self.subTest(servo_id=servo_id):
                axis = tuple(map(float, joints[name].find("axis").attrib["xyz"].split()))
                self.assertEqual(axis, (1.0 if servo_id in reversed_ids else -1.0, 0.0, 0.0))
        for joint in joints.values():
            mimic = joint.find("mimic")
            if mimic is not None:
                self.assertAlmostEqual(float(mimic.attrib["multiplier"]), 2.4)
                self.assertAlmostEqual(float(mimic.attrib["offset"]), -2.1128208657573584)

    def test_urdf_alone_preserves_crank_and_knee_poses_in_servo_coordinates(self):
        urdf = ElementTree.parse(ROOT / "urdf/urdf/URDF.urdf").getroot()
        joints = {joint.get("name"): joint for joint in urdf.findall("joint")}
        report = json.loads((ROOT / "urdf/config/measured_servo_limits.json").read_text())
        # CAD joint frames before the application-side conversion was removed.
        cad_frames = {
            "front_right_joint_13": (3.1415853072, 0.0, 0.0),
            "back_right_joint_23": (0.0, 0.0, 0.0),
            "front_left_joint_33": (3.1415853072, 0.0, 3.1415853072),
            "back_left_joint_43": (0.0, 0.0, -3.1415853072),
        }

        def rotate(vector, rpy):
            x, y, z = vector
            roll, pitch, yaw = rpy
            y, z = math.cos(roll) * y - math.sin(roll) * z, math.sin(roll) * y + math.cos(roll) * z
            x, z = math.cos(pitch) * x + math.sin(pitch) * z, -math.sin(pitch) * x + math.cos(pitch) * z
            return (math.cos(yaw) * x - math.sin(yaw) * y,
                    math.sin(yaw) * x + math.cos(yaw) * y, z)

        for servo in report["servos"]:
            name = servo["urdf_joint"]
            if name not in cad_frames:
                continue
            joint = joints[name]
            lower, upper = servo["lower_radians"], servo["upper_radians"]
            bounds = joint.find("limit")
            self.assertAlmostEqual(float(bounds.get("lower")), lower)
            self.assertAlmostEqual(float(bounds.get("upper")), upper)
            origin = tuple(map(float, joint.find("origin").get("rpy").split()))
            axis = tuple(map(float, joint.find("axis").get("xyz").split()))
            knee = next(j for j in joints.values() if j.find("mimic") is not None
                        and j.find("mimic").get("joint") == name)
            mimic = knee.find("mimic")
            for angle in (lower, (lower + upper) / 2, upper, 0.0):
                with self.subTest(joint=name, angle=angle):
                    # Compare actual orientations, not just signs or RPY values.
                    for basis in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
                        expected = rotate(rotate(basis, (upper - angle, 0, 0)), cad_frames[name])
                        actual = rotate(rotate(basis, tuple(v * angle for v in axis)), origin)
                        for a, b in zip(actual, expected):
                            self.assertAlmostEqual(a, b, places=12)
                    knee_angle = float(mimic.get("multiplier")) * angle + float(mimic.get("offset"))
                    self.assertAlmostEqual(knee_angle, -2.4 * (upper - angle))
            self.assertAlmostEqual(float(knee.find("limit").get("lower")), -2.4 * (upper - lower))
            self.assertEqual(float(knee.find("limit").get("upper")), 0.0)

    def test_orientation_holds_last_pose_after_stale_timeout_and_rejects_invalid(self):
        now = [1.0]
        reference = OrientationReference(max_age=0.5, clock=lambda: now[0])
        self.assertEqual(reference.current(), IDENTITY_QUATERNION)
        self.assertFalse(reference.update((0, 0, 0, 0)))
        self.assertTrue(reference.update((0, 0.1, 0, 0.995)))
        self.assertEqual(reference.current(), (0.0, 0.1, 0.0, 0.995))
        now[0] += 0.51
        self.assertFalse(reference.is_fresh())
        self.assertEqual(reference.current(), (0.0, 0.1, 0.0, 0.995))
        self.assertFalse(reference.update((float("nan"), 0, 0, 1)))
        self.assertEqual(reference.current(), (0.0, 0.1, 0.0, 0.995))
        self.assertTrue(reference.update((0, 0, 0, 1)))
        self.assertTrue(reference.is_fresh())
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
