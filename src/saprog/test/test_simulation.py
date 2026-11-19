"""Hardware-independent simulation lifecycle and Tk checks."""

import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch
from xml.etree import ElementTree as ET

from saprog_desktop.simulation_playback import SimulationPlayback, SimulationRobot
from saprog_desktop.simulation_session import SimulationSession

ROOT = Path(__file__).resolve().parents[3]


class RobotDescriptionTest(unittest.TestCase):
    def test_source_names_are_sdf_compatible(self):
        robot = ET.parse(ROOT / "urdf" / "urdf" / "URDF.urdf").getroot()
        links = {link.attrib["name"] for link in robot.findall("link")}
        joints = {joint.attrib["name"] for joint in robot.findall("joint")}
        self.assertFalse(links & joints)
        self.assertIn("front_right_11", joints)
        self.assertIn("front_right_11_link", links)
        for joint in robot.findall("joint"):
            self.assertIn(joint.find("parent").attrib["link"], links)
            self.assertIn(joint.find("child").attrib["link"], links)
            mimic = joint.find("mimic")
            if mimic is not None:
                self.assertIn(mimic.attrib["joint"], joints)


class SimulationSessionTest(unittest.TestCase):
    def test_failed_start_cleans_marker_and_allows_retry(self):
        session = SimulationSession(ROOT)
        with patch("subprocess.Popen", side_effect=OSError("missing bash")):
            self.assertFalse(session.start())
        self.assertEqual(session.state, "error")
        self.assertIsNone(session._directory)
        self.assertIn("missing bash", session.message)

    def test_only_spawn_marker_marks_ready_and_start_is_exclusive(self):
        process = Mock()
        process.poll.return_value = None
        session = SimulationSession(ROOT)
        with patch("subprocess.Popen", return_value=process) as popen:
            self.assertTrue(session.start())
            self.assertFalse(session.start())
        self.assertEqual(session.poll(), "starting")
        command = popen.call_args.args[0]
        marker = Path(command[-1].split(":=", 1)[1])
        self.assertEqual(command[2], "--launch")
        self.assertTrue(popen.call_args.kwargs["start_new_session"])
        self.assertTrue(popen.call_args.kwargs["env"]["GZ_PARTITION"].startswith("saprog-"))
        marker.touch()
        self.assertEqual(session.poll(), "running")
        with patch.object(session, "_retire_group"):
            session.close()
        self.assertFalse(marker.exists())
        self.assertFalse(session.start())

    def test_import_timeout_stops_owned_group(self):
        session = SimulationSession(ROOT)
        process = Mock()
        process.poll.return_value = None
        with patch("subprocess.Popen", return_value=process):
            session.start()
        session._deadline = 0
        with patch.object(session, "_retire_group") as retire:
            self.assertEqual(session.poll(), "stopping")
            retire.assert_called_once_with(process)
        session._clear_directory()

    def test_commands_require_ready_session_and_use_private_directory(self):
        session = SimulationSession(ROOT)
        with self.assertRaises(ValueError):
            session.command("release")
        with tempfile.TemporaryDirectory() as directory:
            session._directory = SimpleNamespace(name=directory)
            session.state, session.playback = "running", {"ready": True}
            request = session.command("release")
            import json
            document = json.loads((Path(directory) / "command.json").read_text())
            self.assertEqual(document, {"id": request, "action": "release"})


class SimulationPlaybackTest(unittest.TestCase):
    def setUp(self):
        self.send = Mock()
        self.player = SimulationPlayback(SimpleNamespace(joints={"a": (-1, 1, 3, 2)}), self.send)
        self.player.observe(0, {"a": 0})

    def plan(self, *, mode="until_reached", seconds=1, loop=False, on_end="on"):
        return {"name": "test", "steps": [{"targets": {"a": (0.5, 1, 2)}, "mode": mode, "seconds": seconds}],
                "loop": loop, "on_end": on_end}

    def test_paused_feedback_cannot_advance_fixed_duration(self):
        self.player.start(self.plan(mode="seconds"))
        for _ in range(10):
            self.player.observe(0, {"a": 0.5})
        self.assertEqual(self.player.state, "playing")
        self.player.observe(1, {"a": 0.5})
        self.assertEqual(self.player.state, "complete")

    def test_arrival_requires_stable_measured_position(self):
        self.player.start(self.plan())
        self.player.observe(1, {"a": 0.5})
        self.player.observe(1.1, {"a": 0})
        self.player.observe(2, {"a": 0.5})
        self.player.observe(2.3, {"a": 0.5})
        self.assertEqual(self.player.state, "complete")

    def test_end_release_and_loop_stop_hold_current_pose(self):
        self.player.start(self.plan(mode="seconds", on_end="off"))
        self.player.observe(1, {"a": 0.5})
        self.send.assert_called_with(None)
        self.player.start(self.plan(mode="seconds", loop=True))
        self.player.observe(2, {"a": 0.2})
        self.assertEqual(self.player.state, "playing")
        self.player.stop()
        self.send.assert_called_with({"a": (0.2, 3, 2)})
        self.assertIsNone(self.player.plan)

    def test_clock_reset_and_arrival_timeout_release(self):
        self.player.start(self.plan())
        self.player.observe(21, {"a": 0})
        self.assertEqual(self.player.state, "error")
        self.send.assert_called_with(None)
        self.player.start(self.plan())
        self.player.observe(0, {"a": 0})
        self.assertEqual(self.player.state, "error")
        self.send.assert_called_with(None)

    def test_wait_step_retains_pose_and_waits_for_simulated_duration(self):
        plan = self.plan(mode="seconds", seconds=0.2)
        plan["steps"].append({"targets": None, "mode": "seconds", "seconds": 1})
        self.player.start(plan)
        self.player.observe(0.2, {"a": 0.5})
        self.send.assert_called_once()
        self.player.observe(1, {"a": 0.5})
        self.assertEqual(self.player.state, "playing")
        self.player.observe(1.3, {"a": 0.5})
        self.assertEqual(self.player.state, "complete")

    def test_bad_late_sequence_step_is_rejected_before_sending(self):
        import json
        robot = SimulationRobot(ROOT / "urdf/urdf/URDF.urdf")
        with tempfile.TemporaryDirectory() as directory:
            snapshot = {"schema_version": 1, "name": "test", "positions": {
                name: (low + high) / 2 for name, (low, high) in robot.bounds.items()
            }}
            Path(directory, "pose.json").write_text(json.dumps(snapshot))
            step = {"snapshot": "pose.json", "motion": {"speed": 300, "torque": 500}}
            document = {"kind": "saprog_timeline", "schema_version": 2, "steps": [step, {**step, "snapshot": "missing.json"}]}
            path = Path(directory, "sequence.json")
            path.write_text(json.dumps(document))
            with self.assertRaises(OSError):
                self.player.start(robot.load(path, "sequence", directory))
            self.send.assert_not_called()

    def test_source_mimic_and_motion_settings_are_applied(self):
        robot = SimulationRobot(ROOT / "urdf/urdf/URDF.urdf")
        positions = {name: (low + high) / 2 for name, (low, high) in robot.bounds.items()}
        settings = {name: {"speed": 100, "torque": 512} for name in robot.mapping}
        targets = robot.targets(positions, settings)
        leader, ratio, offset = robot.mimics["front_right_leg"]
        self.assertAlmostEqual(targets["front_right_leg"][0], ratio * targets[leader][0] + offset)
        self.assertAlmostEqual(targets[leader][2], robot.joints[leader][3] * 512 / 1023)

    @unittest.skipUnless(hasattr(os, "killpg"), "requires POSIX process groups")
    def test_close_reaps_term_resistant_descendants(self):
        session = SimulationSession(ROOT)
        with tempfile.TemporaryDirectory() as directory:
            ready = Path(directory) / "child-ready"
            child = subprocess.Popen([
                "bash", "-c", 'trap "" TERM; touch "$1"; exec sleep 60', "bash", str(ready),
            ], start_new_session=True)
            session.process = child
            try:
                deadline = time.monotonic() + 5
                while not ready.exists() and time.monotonic() < deadline:
                    time.sleep(0.02)
                self.assertTrue(ready.exists())
                session.close()
                self.assertEqual(child.wait(timeout=6), -signal.SIGKILL)
            finally:
                if child.poll() is None:
                    os.killpg(child.pid, signal.SIGKILL)
                    child.wait()


@unittest.skipUnless(os.environ.get("DISPLAY"), "requires a Tk display")
class SimulationViewTest(unittest.TestCase):
    def test_embedded_and_standalone_close_ownership(self):
        import tkinter as tk

        from saprog_desktop.views.simulation import SimulationView

        root = tk.Tk()
        try:
            root.title("owner")
            view = SimulationView(root, ROOT)
            self.assertEqual(root.title(), "owner")
            view.poll()
            self.assertEqual(str(view.start_button["state"]), "normal")
            self.assertEqual(str(view.stop_button["state"]), "disabled")
            view.session.state = "running"
            view.session.playback = {"ready": True, "state": "idle", "message": "ready"}
            view.poll()
            self.assertEqual(str(view.play_buttons[0]["state"]), "normal")
            selected = view.selection["snapshot"].get()
            with patch.object(view.session, "command", return_value="request") as command:
                view.play("snapshot")
                self.assertEqual(command.call_args.kwargs["path"].name, selected)
                self.assertEqual(command.call_args.kwargs["kind"], "snapshot")
            view.close()
            self.assertTrue(view.session.closing)
        finally:
            root.destroy()


if __name__ == "__main__":
    unittest.main()
