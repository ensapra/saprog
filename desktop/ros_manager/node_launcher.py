"""Build and launch ROS 2 package executables on the local ROS environment."""

import os
import logging
import shlex
import shutil
import subprocess
from pathlib import Path
from typing import Callable

from .models import NodeRequirement

_LOGGER = logging.getLogger("saprog.ros_manager")


class RosNodeLauncher:
    """Resolve ROS setup, build missing packages, and own launched processes."""

    def __init__(self, workspace: Path):
        self.workspace = Path(workspace)

    def ensure_executable(
        self,
        requirement: NodeRequirement,
        status: Callable[[str], None],
    ) -> None:
        check = subprocess.run(
            self._ros_shell(
                f"ros2 pkg executables {shlex.quote(requirement.package)}"
            ),
            cwd=self.workspace,
            capture_output=True,
            text=True,
            timeout=15,
            check=False,
        )
        expected = f"{requirement.package} {requirement.executable}"
        if check.returncode == 0 and expected in check.stdout.splitlines():
            return

        _LOGGER.info("Building ROS component %s with colcon", requirement.package)
        status(f"Building {requirement.package}")
        build = subprocess.run(
            self._ros_shell(
                f"cd {shlex.quote(str(self.workspace))} && "
                f"colcon build --packages-up-to {shlex.quote(requirement.package)}"
            ),
            cwd=self.workspace,
            capture_output=True,
            text=True,
            timeout=600,
            check=False,
        )
        if build.returncode != 0:
            details = (build.stderr or build.stdout).strip()
            raise RuntimeError(details[-1500:] or "colcon build failed")

    def launch(self, requirement: NodeRequirement) -> subprocess.Popen:
        _LOGGER.info(
            "Launching ROS component %s (%s)", requirement.package, requirement.executable
        )
        command = (
            f"ros2 run {shlex.quote(requirement.package)} "
            f"{shlex.quote(requirement.executable)}"
        )
        return subprocess.Popen(
            self._ros_shell(f"exec {command}"),
            cwd=self.workspace,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

    @staticmethod
    def stop_processes(processes: dict[tuple[str, str], subprocess.Popen]) -> None:
        for process in processes.values():
            if process.poll() is None:
                process.terminate()
        for process in processes.values():
            if process.poll() is None:
                try:
                    process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    process.kill()

    def _ros_shell(self, command: str) -> list[str]:
        if not shutil.which("bash"):
            raise RuntimeError("bash is required to source the ROS environment")
        distro = os.environ.get("ROS_DISTRO", "jazzy")
        ros_setup = Path("/opt/ros") / distro / "setup.bash"
        source_parts = []
        if ros_setup.is_file():
            source_parts.append(f"source {shlex.quote(str(ros_setup))}")
        elif not shutil.which("ros2"):
            raise RuntimeError(f"ROS 2 {distro} was not found under /opt/ros")

        overlay_setup = self.workspace / "install" / "setup.bash"
        if overlay_setup.is_file():
            source_parts.append(f"source {shlex.quote(str(overlay_setup))}")
        source_parts.append(command)
        return ["bash", "-lc", " && ".join(source_parts)]
