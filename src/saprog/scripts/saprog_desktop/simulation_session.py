"""Own a Gazebo launch group independently of all hardware processes."""

import os
import signal
import subprocess
import tempfile
import threading
import time
import uuid
from pathlib import Path


class SimulationSession:
    def __init__(self, workspace):
        self.workspace = Path(workspace)
        self.process = None
        self.state = "stopped"
        self.message = "Start Gazebo to import the robot into a paused physics world."
        self.closing = False
        self._directory = None
        self._deadline = 0.0
        self._retired = set()

    def start(self):
        if self.closing or self.process is not None:
            return False
        self._directory = tempfile.TemporaryDirectory(prefix="saprog-gazebo-")
        marker = Path(self._directory.name) / "ready"
        env = os.environ.copy()
        # Each launch owns its Gazebo transport world, even alongside another GUI.
        env["GZ_PARTITION"] = f"saprog-{uuid.uuid4().hex}"
        try:
            self.process = subprocess.Popen(
                ["bash", str(self.workspace / "tabs" / "launch_simulation.sh"),
                 "--launch", f"ready_file:={marker}"],
                cwd=self.workspace, env=env, start_new_session=True,
            )
        except OSError as error:
            self._clear_directory()
            self.state = "error"
            self.message = f"Could not start Gazebo: {error}"
            return False
        self._deadline = time.monotonic() + 90
        self.state = "starting"
        self.message = "Starting Gazebo and importing the URDF; see terminal output for details."
        return True

    def poll(self):
        if self.process is None:
            return self.state
        result = self.process.poll()
        if result is not None:
            expected = self.state == "stopping"
            self._retire_group(self.process)
            self.process = None
            self._clear_directory()
            self.state = "stopped" if expected or result == 0 else "error"
            self.message = "Gazebo stopped." if self.state == "stopped" else (
                f"Gazebo exited (code {result}). Check terminal output. "
                "Install ros-jazzy-ros-gz-sim and build saprog_simulation; see docs/simulation.md."
            )
        elif self.state == "starting":
            if (Path(self._directory.name) / "ready").exists():
                self.state = "running"
                self.message = (
                    "Robot imported. Gazebo starts paused: use Play to apply gravity and contacts. "
                    "The joints are passive, so the robot may fold or fall."
                )
            elif time.monotonic() > self._deadline:
                self.stop()
                self.message = "Robot import timed out; stopping Gazebo. Check terminal output."
        return self.state

    @staticmethod
    def _signal(process, sig):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass

    def _retire_group(self, process):
        if process in self._retired:
            return
        self._retired.add(process)
        self._signal(process, signal.SIGTERM)

        def finish():
            self._signal(process, signal.SIGKILL)
            process.wait()

        # Keep the interpreter alive until owned descendants have been cleaned up.
        timer = threading.Timer(3.0, finish)
        timer.daemon = False
        timer.start()

    def stop(self):
        if self.process is None or self.state == "stopping":
            return
        self.state = "stopping"
        self.message = "Stopping Gazebo..."
        self._retire_group(self.process)

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.stop()
        self._clear_directory()

    def _clear_directory(self):
        if self._directory is not None:
            self._directory.cleanup()
            self._directory = None
