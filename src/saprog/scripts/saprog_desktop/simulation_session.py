"""Own a Gazebo launch group independently of all hardware processes."""

import json
import os
import signal
import subprocess
import tempfile
import threading
import time
import uuid
from pathlib import Path

from saprog_desktop.snapshot_files import write_json_atomic


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
        self.playback = {}

    def start(self, *, gui=True):
        if self.closing or self.process is not None:
            return False
        self._directory = tempfile.TemporaryDirectory(prefix="saprog-gazebo-")
        self.playback = {}
        marker = Path(self._directory.name) / "ready"
        env = os.environ.copy()
        # Each launch owns its Gazebo transport world, even alongside another GUI.
        self.partition = env["GZ_PARTITION"] = f"saprog-{uuid.uuid4().hex}"
        try:
            self.process = subprocess.Popen(
                ["bash", str(self.workspace / "tabs" / "launch_simulation.sh"),
                 "--launch", f"gui:={str(gui).lower()}",
                 f"control_dir:={self._directory.name}", f"ready_file:={marker}"],
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
        if self._directory is not None:
            status = Path(self._directory.name) / "playback.json"
            try:
                self.playback = json.loads(status.read_text(encoding="utf-8"))
            except (OSError, ValueError):
                pass
        if self.process is None:
            return self.state
        result = self.process.poll()
        if result is not None:
            expected = self.state == "stopping"
            self._retire_group(self.process)
            self.process = None
            self._clear_directory()
            failed = self.playback.get("state") == "error"
            self.state = "stopped" if expected or (result == 0 and not failed) else "error"
            self.message = "Gazebo stopped." if self.state == "stopped" else (
                f"Gazebo exited (code {result}). Check terminal output. "
                "Install ros-jazzy-ros-gz-sim and build saprog_simulation; see docs/simulation.md."
            )
            if failed and not expected:
                self.message = self.playback.get("message", self.message)
        elif self.state == "starting":
            if (Path(self._directory.name) / "ready").exists():
                self.state = "running"
                self.message = (
                    "Robot imported. Gazebo starts paused: use Play to apply gravity and contacts. "
                    "Select a snapshot or sequence below to enable simulated actuators."
                )
            elif time.monotonic() > self._deadline:
                self.stop()
                self.message = "Robot import timed out; stopping Gazebo. Check terminal output."
        return self.state

    def command(self, action, *, path=None, kind=None, snapshots=None):
        if self.state != "running" or self._directory is None or not self.playback.get("ready"):
            raise ValueError("Start Gazebo and wait for simulated joint feedback first")
        command = {"id": uuid.uuid4().hex, "action": action}
        if action == "play":
            command.update(path=str(Path(path).resolve()), kind=kind, snapshots=str(Path(snapshots).resolve()))
        write_json_atomic(Path(self._directory.name) / "command.json", command)
        return command["id"]

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
        self.playback = {}
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
