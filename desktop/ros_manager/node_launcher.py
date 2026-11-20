"""Build and launch ROS 2 package executables on the local ROS environment."""

import logging
import os
import shlex
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import Callable

from .models import NodeRequirement

_LOGGER = logging.getLogger("saprog.ros_manager")


class RosNodeLauncher:
    """Build requested packages and own the processes it launches."""

    def __init__(self, workspace: Path):
        self.workspace = Path(workspace)

    def ensure_executable(
        self,
        requirement: NodeRequirement,
        status: Callable[[str], None],
        cancel_event=None,
    ) -> None:
        _LOGGER.info("Checking whether ROS component %s needs building", requirement.package)
        status(f"Checking build for {requirement.package}")
        build = subprocess.Popen(
            self._ros_shell(
                f"cd {shlex.quote(str(self.workspace))} && "
                f"colcon build --event-handlers console_direct+ --packages-up-to {shlex.quote(requirement.package)}"
            ),
            cwd=self.workspace,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        deadline = time.monotonic() + 600
        while True:
            try:
                stdout, stderr = build.communicate(timeout=0.2)
                break
            except subprocess.TimeoutExpired:
                cancelled = cancel_event is not None and cancel_event.is_set()
                timed_out = time.monotonic() >= deadline
                if not cancelled and not timed_out:
                    continue
                try:
                    os.killpg(build.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    stdout, stderr = build.communicate(timeout=2)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(build.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    stdout, stderr = build.communicate()
                if cancelled:
                    raise RuntimeError("ROS package build cancelled")
                raise RuntimeError("ROS package build timed out after 600 seconds")
        if build.returncode != 0:
            details = (stderr or stdout).strip()
            raise RuntimeError(details[-1500:] or "colcon build failed")
        build_output = f"{stdout}\n{stderr}"
        if f"Starting >>> {requirement.package}" in build_output:
            _LOGGER.info("Built ROS component %s", requirement.package)
            status(f"Built {requirement.package}")
        else:
            _LOGGER.info("ROS component %s is already up to date", requirement.package)
            status(f"{requirement.package} is already up to date")

    def launch(self, requirement: NodeRequirement) -> subprocess.Popen:
        _LOGGER.info(
            "Launching ROS component %s (%s)", requirement.package, requirement.executable
        )
        package = shlex.quote(requirement.package)
        executable = shlex.quote(requirement.executable)
        run_command = f"ros2 run {package} {executable}"
        command = (
            f"share_dir=$(ros2 pkg prefix --share {package} 2>/dev/null || true); "
            f"if [ -n \"$share_dir\" ] && [ -f \"$share_dir/config/saprog.yaml\" ]; then "
            f"exec {run_command} --ros-args --params-file \"$share_dir/config/saprog.yaml\"; "
            f"else exec {run_command}; fi"
        )
        return subprocess.Popen(
            self._ros_shell(command),
            cwd=self.workspace,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            start_new_session=True,
        )

    @staticmethod
    def stop_process(process: subprocess.Popen) -> None:
        if process.poll() is None:
            try:
                os.killpg(process.pid, 15)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, 9)
                except ProcessLookupError:
                    pass

    @staticmethod
    def stop_processes(processes: dict[tuple[str, str], subprocess.Popen]) -> None:
        for process in processes.values():
            RosNodeLauncher.stop_process(process)

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
