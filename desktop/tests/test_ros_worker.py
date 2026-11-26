"""Windows-side WSL startup regressions without starting WSL or ROS."""

import subprocess
import sys
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ros_manager import ros_worker  # noqa: E402


class WslStartupTests(unittest.TestCase):
    def setUp(self):
        self.status = Mock()
        self.worker = ros_worker.RosWorker(
            Mock(return_value={}), Mock(return_value=[]), Mock(), self.status, Mock(),
            Mock(workspace=Path("C:/robot workspace/ros2")),
        )
        self.process = Mock(returncode=0)
        self.process.poll.return_value = 0
        self.process.communicate.return_value = ("/mnt/c/robot workspace/ros2\n", "")
        popen = patch.object(ros_worker.subprocess, "Popen", return_value=self.process)
        self.popen = popen.start()
        self.addCleanup(popen.stop)

    def convert(self):
        return self.worker._wsl_path(
            "wsl.exe", "Ubuntu-custom", self.worker._launcher.workspace,
        )

    def test_cold_boot_can_take_longer_than_ten_seconds(self):
        self.process.communicate.side_effect = [
            subprocess.TimeoutExpired("wslpath", 0.2),
            self.process.communicate.return_value,
        ]
        with patch.object(ros_worker.time, "monotonic", side_effect=[0, 11]):
            self.assertEqual(self.convert(), "/mnt/c/robot workspace/ros2")
        self.assertEqual(self.process.communicate.call_count, 2)
        self.process.kill.assert_not_called()
        self.assertEqual(self.popen.call_args.args[0], [
            "wsl.exe", "--distribution", "Ubuntu-custom", "--exec", "wslpath", "-u",
            str(self.worker._launcher.workspace),
        ])

    def test_hung_distribution_times_out_with_guidance_and_reaps_process(self):
        self.process.poll.return_value = None
        self.process.communicate.side_effect = [
            subprocess.TimeoutExpired("wslpath", 0.2), ("", ""),
        ]
        with patch.object(ros_worker.time, "monotonic", side_effect=[0, 61]):
            with self.assertRaisesRegex(RuntimeError, "Ubuntu-custom.*60 seconds.*wsl"):
                self.convert()
        self.process.kill.assert_called_once()
        self.process.communicate.assert_called_with(timeout=2.0)

    def test_stop_during_startup_cleans_up_without_launching_bridge_or_reporting_failure(self):
        self.process.poll.return_value = None

        def communicate(*, timeout):
            if timeout == 0.2:
                self.worker.stop()
                raise subprocess.TimeoutExpired("wslpath", timeout)
            return "", ""

        self.process.communicate.side_effect = communicate
        with patch.object(ros_worker.shutil, "which", return_value="wsl.exe"), patch.object(
            ros_worker._LOGGER, "exception",
        ) as log_error:
            self.worker._run_wsl_bridge()
        self.popen.assert_called_once()
        self.process.kill.assert_called_once()
        self.process.communicate.assert_called_with(timeout=2.0)
        log_error.assert_not_called()
        self.assertEqual(self.status.call_count, 1)
        self.assertIn("Starting WSL", self.status.call_args.args[1])

    def test_already_stopped_worker_does_not_start_path_conversion(self):
        self.worker.stop()
        self.assertIsNone(self.convert())
        self.popen.assert_not_called()

    def test_conversion_failure_preserves_wsl_diagnostic(self):
        self.process.returncode = 1
        for stdout, stderr, expected in (
            ("", "distribution missing\n", "distribution missing"),
            ("WSL startup failed\n", "", "WSL startup failed"),
            ("", "", "wslpath failed"),
        ):
            with self.subTest(expected=expected):
                self.process.communicate.return_value = stdout, stderr
                with self.assertRaisesRegex(RuntimeError, expected):
                    self.convert()

    def test_empty_conversion_is_rejected(self):
        self.process.communicate.return_value = "\n", ""
        with self.assertRaisesRegex(RuntimeError, "empty path"):
            self.convert()


if __name__ == "__main__":
    unittest.main()
