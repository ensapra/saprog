import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch
from xml.etree import ElementTree

from saprog_desktop.robot_view_model import (
    IDENTITY_QUATERNION,
    OrientationReference,
    SERVO_JOINT_NAMES,
)
from saprog_desktop.robot_view_session import RendererSession  # noqa: E402


ROOT = Path(__file__).resolve().parents[3]


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
        self.assertEqual(SERVO_JOINT_NAMES[13], "froint_right_joint_13")

    def test_orientation_resets_to_identity_after_stale_timeout_and_rejects_invalid(self):
        now = [1.0]
        reference = OrientationReference(max_age=0.5, clock=lambda: now[0])
        self.assertEqual(reference.current(), IDENTITY_QUATERNION)
        self.assertFalse(reference.update((0, 0, 0, 0)))
        self.assertTrue(reference.update((0, 0.1, 0, 0.995)))
        self.assertEqual(reference.current(), (0.0, 0.1, 0.0, 0.995))
        now[0] += 0.51
        self.assertEqual(reference.current(), IDENTITY_QUATERNION)

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
