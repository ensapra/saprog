"""Exercise the generated launch commands in a real shell without ROS or USB."""

import os
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from ros_manager.models import NodeRequirement  # noqa: E402
from ros_manager.node_launcher import RosNodeLauncher  # noqa: E402
from ros_manager.wsl_bridge import processes as wsl_bridge  # noqa: E402


def shell_path(path):
    text = Path(path).as_posix()
    return f"/{text[0].lower()}{text[2:]}" if os.name == "nt" else text


_BASH = (
    "C:/Program Files/Git/bin/bash.exe" if os.name == "nt" else shutil.which("bash")
)


@unittest.skipUnless(_BASH and Path(_BASH).is_file(), "Requires Bash (Git Bash on Windows)")
class RosLauncherTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="saprog launch test ")
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.workspace = self.root / "ros2"
        self.workspace.mkdir()
        self.bin = self.root / "bin"
        self.bin.mkdir()
        self.output = self.root / "arguments.txt"
        self.helper_calls = self.root / "helper.txt"
        fake_ros = self.bin / "ros2"
        fake_ros.write_text(
            '#!/bin/bash\nif [ "$1" = pkg ]; then printf "%s\\n" "$TEST_SHARE"; '
            'else printf "%s\\n" "$@" "${SAPROG_IMU_DEVICE:-unset}" > "$TEST_OUTPUT"; fi\n',
            encoding="utf-8", newline="\n",
        )
        fake_ros.chmod(0o755)
        helper = self.root / "scripts" / "ensure_imu_usb.bash"
        helper.parent.mkdir()
        helper.write_text(
            'printf called > "$TEST_HELPER_CALLS"\n'
            '[ "${TEST_HELPER_FAIL:-0}" = 0 ] || exit 1\n'
            'printf "/dev/ttyACM7\\n"\n', encoding="utf-8", newline="\n",
        )
        self.share = self.root / "share"
        self.share.mkdir()
        env = {
            "PATH": f"{shell_path(self.bin)}:/usr/bin:/bin",
            "TEST_SHARE": shell_path(self.share),
            "TEST_OUTPUT": shell_path(self.output),
            "TEST_HELPER_CALLS": shell_path(self.helper_calls),
            "SAPROG_IMU_DEVICE": "",
            "TEST_HELPER_FAIL": "0",
        }
        environment = patch.dict(os.environ, env)
        environment.start()
        self.addCleanup(environment.stop)

    def run_launcher(self, bridge=True):
        if bridge:
            with patch.object(wsl_bridge, "ros_shell", side_effect=lambda _, cmd: [_BASH, "-c", cmd]):
                process = wsl_bridge.launch_node(str(self.workspace), "saprog_imu", "imu_publisher_node")
        else:
            launcher = RosNodeLauncher(self.workspace)
            with patch.object(launcher, "_ros_shell", side_effect=lambda cmd: [_BASH, "-c", cmd]):
                process = launcher.launch(NodeRequirement("saprog_imu", "imu_publisher_node"))
        self.addCleanup(lambda: process.kill() if process.poll() is None else None)
        return process.wait(timeout=10)

    def test_local_launch_runs_executable(self):
        self.assertEqual(self.run_launcher(bridge=False), 0)
        self.assertEqual(self.output.read_text().splitlines()[:3], ["run", "saprog_imu", "imu_publisher_node"])

    def test_bridge_attaches_imu_and_passes_detected_port(self):
        self.assertEqual(self.run_launcher(), 0)
        self.assertTrue(self.helper_calls.exists())
        self.assertEqual(self.output.read_text().splitlines()[-1], "/dev/ttyACM7")

    def test_explicit_port_skips_attachment(self):
        os.environ["SAPROG_IMU_DEVICE"] = "/dev/pts/9"
        self.assertEqual(self.run_launcher(), 0)
        self.assertFalse(self.helper_calls.exists())
        self.assertEqual(self.output.read_text().splitlines()[-1], "/dev/pts/9")

    def test_attachment_failure_does_not_launch_node(self):
        os.environ["TEST_HELPER_FAIL"] = "1"
        self.assertNotEqual(self.run_launcher(), 0)
        self.assertFalse(self.output.exists())

    def test_configured_launch_keeps_parameter_file(self):
        config = self.share / "config" / "saprog.yaml"
        config.parent.mkdir()
        config.write_text("{}", encoding="utf-8")
        self.assertEqual(self.run_launcher(), 0)
        self.assertEqual(self.output.read_text().splitlines()[3:6], [
            "--ros-args", "--params-file", f"{shell_path(self.share)}/config/saprog.yaml",
        ])


if __name__ == "__main__":
    unittest.main()
