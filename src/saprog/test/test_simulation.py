"""Hardware-independent simulation lifecycle and Tk checks."""

import os
import signal
import subprocess
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch
from xml.etree import ElementTree as ET

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
            view.close()
            self.assertTrue(view.session.closing)
        finally:
            root.destroy()


if __name__ == "__main__":
    unittest.main()
