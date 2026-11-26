"""Windows-side process and JSON protocol for the ROS 2 WSL bridge."""

import json
import logging
import os
import queue
import shlex
import shutil
import subprocess
import threading
import time
from collections.abc import Callable
from pathlib import Path

from .models import TopicRequest
from .node_launcher import RosNodeLauncher

_LOGGER = logging.getLogger("saprog.ros_manager")
_WSL_STARTUP_TIMEOUT = 60.0


class RosWorker(threading.Thread):
    """Synchronize desktop requests and commands with the WSL ROS bridge."""

    def __init__(
        self,
        get_requests: Callable[[], dict[str, TopicRequest]],
        get_commands: Callable[[], list[dict]],
        on_message: Callable[[str, object], None],
        on_status: Callable[[str, str], None],
        on_command_result: Callable[[str, bool, str], None],
        launcher: RosNodeLauncher,
        on_position_published: Callable[[str, int], None] | None = None,
    ):
        super().__init__(name="saprog-ros-data-manager", daemon=True)
        self._get_requests = get_requests
        self._get_commands = get_commands
        self._on_message = on_message
        self._on_status = on_status
        self._on_command_result = on_command_result
        self._launcher = launcher
        self._on_position_published = on_position_published
        self._stopping = threading.Event()

    def stop(self) -> None:
        self._stopping.set()

    def run(self) -> None:
        if os.name != "nt":
            self._on_status("", "The desktop ROS worker currently requires Windows and WSL")
            return
        self._run_wsl_bridge()

    def _run_wsl_bridge(self) -> None:
        process = None
        output_queue: queue.Queue[str | None] = queue.Queue()
        reader = None
        try:
            wsl = shutil.which("wsl.exe") or shutil.which("wsl")
            if not wsl:
                raise RuntimeError("wsl.exe was not found on PATH")

            distro = os.environ.get("SAPROG_WSL_DISTRO", "Ubuntu-24.04")
            bridge = Path(__file__).with_name("wsl_bridge")
            self._on_status("", f"Starting WSL ({distro})")
            workspace = self._wsl_path(wsl, distro, self._launcher.workspace)
            if workspace is None:
                return
            bridge_path = self._wsl_path(wsl, distro, bridge)
            if bridge_path is None or self._stopping.is_set():
                return
            ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
            command = (
                f"source /opt/ros/{shlex.quote(ros_distro)}/setup.bash && "
                f"exec python3 -u {shlex.quote(bridge_path)} "
                f"--workspace {shlex.quote(workspace)}"
            )
            process = subprocess.Popen(
                [wsl, "--distribution", distro, "--exec", "bash", "-lc", command],
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                encoding="utf-8",
                bufsize=1,
            )

            def read_events():
                assert process is not None and process.stdout is not None
                for line in process.stdout:
                    output_queue.put(line)
                output_queue.put(None)

            reader = threading.Thread(
                target=read_events, name="saprog-wsl-bridge-output", daemon=True
            )
            reader.start()
            sent: dict[str, TopicRequest] = {}

            while not self._stopping.is_set() and process.poll() is None:
                self._sync_requests(process, sent)
                for command in self._get_commands():
                    self._send_bridge_command(process, command)

                try:
                    # Check outgoing commands more often than the 50 ms jog
                    # tick, even when no telemetry or acknowledgement arrives.
                    line = output_queue.get(timeout=0.01)
                except queue.Empty:
                    continue
                if line is None:
                    break
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    _LOGGER.debug("Ignoring non-JSON WSL bridge output: %s", line.rstrip())
                    continue
                self._handle_event(event)

            if process.poll() is not None and not self._stopping.is_set():
                self._on_status("", f"WSL ROS bridge exited with code {process.returncode}")
        except Exception as error:
            _LOGGER.exception("WSL ROS manager failed")
            self._on_status("", f"WSL ROS manager unavailable: {error}")
        finally:
            if process is not None:
                if process.stdin:
                    process.stdin.close()
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    process.terminate()
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
            if reader is not None:
                reader.join(timeout=1.0)

    def _sync_requests(self, process, sent: dict[str, TopicRequest]) -> None:
        current = self._get_requests()
        for token in set(sent) - set(current):
            self._send_bridge_command(process, {"op": "release", "token": token})
            sent.pop(token, None)
        for token, request in current.items():
            if sent.get(token) == request:
                continue
            self._send_bridge_command(
                process,
                {
                    "op": "subscribe",
                    "token": token,
                    "topic": request.topic,
                    "message_type": request.message_type,
                    "package": request.requirement.package,
                    "executable": request.requirement.executable,
                },
            )
            sent[token] = request

    def _handle_event(self, event: dict) -> None:
        kind = event.get("event")
        if kind == "message":
            data = event.get("data", {})
            if isinstance(data, dict) and "received_at_ns" in event:
                data = {**data, "_ros_received_at_ns": event["received_at_ns"]}
            self._on_message(event.get("topic", ""), data)
        elif kind == "status":
            topic = event.get("topic", "")
            status = event.get("status", "")
            if (
                status.startswith(("Checking build", "Built ", "Launching "))
                or status.endswith("is already up to date")
                or "exited with code" in status
                or status.startswith("Could not start ")
            ):
                _LOGGER.info("ROS manager: %s", status)
            self._on_status(topic, status)
        elif kind == "command_result":
            if event.get("success") and self._on_position_published is not None:
                stamp = event.get("published_at_ns")
                if type(stamp) is int:
                    self._on_position_published(event.get("id", ""), stamp)
            self._on_command_result(
                event.get("id", ""),
                bool(event.get("success")),
                event.get("message", ""),
            )

    @staticmethod
    def _send_bridge_command(process: subprocess.Popen, command: dict) -> None:
        if process.stdin is None:
            raise RuntimeError("WSL bridge input is unavailable")
        process.stdin.write(json.dumps(command, separators=(",", ":")) + "\n")
        process.stdin.flush()

    def _wsl_path(self, wsl: str, distro: str, windows_path: Path) -> str | None:
        """Allow WSL to cold-boot, but stop promptly when the desktop closes."""
        if self._stopping.is_set():
            return None
        process = subprocess.Popen(
            [wsl, "--distribution", distro, "--exec", "wslpath", "-u", str(windows_path)],
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        deadline = time.monotonic() + _WSL_STARTUP_TIMEOUT
        try:
            while not self._stopping.is_set():
                try:
                    stdout, stderr = process.communicate(timeout=0.2)
                    break
                except subprocess.TimeoutExpired:
                    if time.monotonic() >= deadline:
                        raise RuntimeError(
                            f"WSL distribution {distro!r} did not respond within "
                            f"{_WSL_STARTUP_TIMEOUT:g} seconds. Check that it starts with: "
                            f'wsl --distribution "{distro}" --exec /bin/true'
                        ) from None
            else:
                return None
        finally:
            if process.poll() is None:
                process.kill()
                try:
                    process.communicate(timeout=2.0)
                except subprocess.TimeoutExpired:
                    _LOGGER.warning("Timed out cleaning up WSL path conversion")
        if process.returncode != 0:
            details = stderr.strip() or stdout.strip() or "wslpath failed"
            raise RuntimeError(details)
        path = stdout.strip()
        if not path:
            raise RuntimeError(f"wslpath returned an empty path for {windows_path}")
        return path
