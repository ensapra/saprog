"""Background ROS subscription loop used by :mod:`desktop.ros_manager`."""

import subprocess
import json
import logging
import os
import queue
import shlex
import shutil
import threading
import time
from collections.abc import Callable
from pathlib import Path

from .models import TopicRequest
from .node_launcher import RosNodeLauncher

_LOGGER = logging.getLogger("saprog.ros_manager")


class RosWorker(threading.Thread):
    """Own rclpy subscriptions and manager-launched node processes."""

    DISCOVERY_SECONDS = 1.5
    RETRY_SECONDS = 5.0

    def __init__(
        self,
        get_requests: Callable[[], dict[str, TopicRequest]],
        on_message: Callable[[str, object], None],
        on_status: Callable[[str, str], None],
        launcher: RosNodeLauncher,
    ):
        super().__init__(name="saprog-ros-data-manager", daemon=True)
        self._get_requests = get_requests
        self._on_message = on_message
        self._on_status = on_status
        self._launcher = launcher
        self._stopping = threading.Event()

    def stop(self) -> None:
        self._stopping.set()

    def run(self) -> None:
        if os.name == "nt":
            self._run_wsl_bridge()
            return

        launched: dict[tuple[str, str], subprocess.Popen] = {}
        build_results: queue.Queue = queue.Queue()
        build_jobs: dict[tuple[str, str], threading.Thread] = {}
        node = None
        rclpy = None
        try:
            if os.name != "posix":
                raise RuntimeError(
                    "ROS subscriptions and automatic build currently require running the desktop in Linux/WSL"
                )

            import rclpy as ros_client
            from rosidl_runtime_py.convert import message_to_ordereddict
            from rosidl_runtime_py.utilities import get_message
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data

            rclpy = ros_client
            rclpy.init(args=None)
            node = rclpy.create_node("saprog_desktop_ros_manager")
            executor = SingleThreadedExecutor()
            executor.add_node(node)
            subscriptions = {}
            last_attempt: dict[tuple[str, str], float] = {}
            last_message_at: dict[str, float] = {}
            last_status: dict[str, str] = {}
            initialized_at = time.monotonic()

            while not self._stopping.is_set():
                while True:
                    try:
                        key, topic, process, error = build_results.get_nowait()
                    except queue.Empty:
                        break
                    build_jobs.pop(key, None)
                    if error is not None:
                        _LOGGER.warning("Could not start ROS component %s: %s", key[0], error)
                        self._set_status(last_status, topic, f"Could not start {key[0]}: {error}")
                    elif process is not None:
                        launched[key] = process
                        self._set_status(last_status, topic, f"Started {key[1]}")

                requests = {
                    request.topic: request
                    for request in self._get_requests().values()
                }

                for stale_topic in set(subscriptions) - set(requests):
                    node.destroy_subscription(subscriptions.pop(stale_topic))
                    last_message_at.pop(stale_topic, None)

                for topic, request in requests.items():
                    if topic not in subscriptions:
                        try:
                            message_class = get_message(request.message_type)
                        except (ImportError, AttributeError, ValueError) as error:
                            self._set_status(last_status, topic, f"Unknown ROS message type: {error}")
                            continue

                        def receive(message, topic_name=topic):
                            last_message_at[topic_name] = time.monotonic()
                            self._on_message(
                                topic_name, dict(message_to_ordereddict(message))
                            )

                        subscriptions[topic] = node.create_subscription(
                            message_class, topic, receive, qos_profile_sensor_data
                        )
                        self._set_status(last_status, topic, "Subscribed; waiting for publisher")

                    requirement = request.requirement
                    key = (requirement.package, requirement.executable)
                    publishers = node.count_publishers(topic)
                    process = launched.get(key)

                    last_message = last_message_at.get(topic)
                    if last_message is not None:
                        topic_status = (
                            "Stale IMU data" if time.monotonic() - last_message > 2.0 else "Connected"
                        )
                    elif publishers > 0:
                        topic_status = "Waiting for IMU data"
                    else:
                        topic_status = "Subscribed; waiting for publisher"

                    if process is not None and process.poll() is None:
                        self._set_status(
                            last_status,
                            topic,
                            topic_status if publishers > 0 else f"Starting {requirement.executable}",
                        )
                        continue
                    if publishers > 0:
                        self._set_status(last_status, topic, topic_status)
                        continue

                    if time.monotonic() - initialized_at < self.DISCOVERY_SECONDS:
                        continue
                    if time.monotonic() - last_attempt.get(key, 0.0) < self.RETRY_SECONDS:
                        continue

                    last_attempt[key] = time.monotonic()
                    self._set_status(last_status, topic, f"Checking {requirement.package}")
                    if key not in build_jobs:
                        def build_and_launch(
                            build_key=key, build_topic=topic, build_requirement=requirement
                        ):
                            try:
                                self._launcher.ensure_executable(
                                    build_requirement,
                                    lambda status: self._on_status(build_topic, status),
                                    cancel_event=self._stopping,
                                )
                                process = None if self._stopping.is_set() else self._launcher.launch(build_requirement)
                                error = None
                            except Exception as build_error:
                                process = None
                                error = build_error
                            build_results.put((build_key, build_topic, process, error))

                        build_thread = threading.Thread(
                            target=build_and_launch,
                            name=f"saprog-build-{requirement.package}",
                            daemon=True,
                        )
                        build_jobs[key] = build_thread
                        build_thread.start()

                executor.spin_once(timeout_sec=0.1)
        except Exception as error:  # ROS remains optional for the desktop app.
            _LOGGER.exception("ROS manager worker failed")
            self._on_status("", f"ROS manager unavailable: {error}")
        finally:
            self._stopping.set()
            for build_thread in build_jobs.values():
                build_thread.join()
            while True:
                try:
                    _key, _topic, process, _error = build_results.get_nowait()
                except queue.Empty:
                    break
                if process is not None:
                    self._launcher.stop_process(process)
            self._launcher.stop_processes(launched)
            if node is not None:
                node.destroy_node()
            if rclpy is not None and rclpy.ok():
                rclpy.shutdown()

    def _set_status(self, previous: dict[str, str], topic: str, status: str) -> None:
        if previous.get(topic) == status:
            return
        previous[topic] = status
        self._on_status(topic, status)

    def _run_wsl_bridge(self) -> None:
        """Run the ROS subscriber/build launcher inside the configured WSL distro."""
        process = None
        output_queue: queue.Queue[str | None] = queue.Queue()
        reader = None
        try:
            wsl = shutil.which("wsl.exe") or shutil.which("wsl")
            if not wsl:
                raise RuntimeError("wsl.exe was not found on PATH")

            distro = os.environ.get("SAPROG_WSL_DISTRO", "Ubuntu-24.04")
            workspace = self._launcher.workspace
            bridge = Path(__file__).with_name("wsl_bridge.py")
            wsl_workspace = self._wsl_path(wsl, distro, workspace)
            wsl_bridge = self._wsl_path(wsl, distro, bridge)
            ros_distro = os.environ.get("ROS_DISTRO", "jazzy")
            command = (
                f"source /opt/ros/{shlex.quote(ros_distro)}/setup.bash && "
                f"exec python3 -u {shlex.quote(wsl_bridge)} "
                f"--workspace {shlex.quote(wsl_workspace)}"
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

            reader = threading.Thread(target=read_events, name="saprog-wsl-bridge-output", daemon=True)
            reader.start()
            sent: dict[str, TopicRequest] = {}

            while not self._stopping.is_set() and process.poll() is None:
                current = self._get_requests()
                for token in set(sent) - set(current):
                    self._send_bridge_command(process, {"op": "release", "token": token})
                    sent.pop(token, None)
                for token, request in current.items():
                    if sent.get(token) == request:
                        continue
                    self._send_bridge_command(process, {
                        "op": "subscribe",
                        "token": token,
                        "topic": request.topic,
                        "message_type": request.message_type,
                        "package": request.requirement.package,
                        "executable": request.requirement.executable,
                    })
                    sent[token] = request

                try:
                    line = output_queue.get(timeout=0.1)
                except queue.Empty:
                    continue
                if line is None:
                    break
                try:
                    event = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if event.get("event") == "message":
                    self._on_message(event.get("topic", ""), event.get("data", {}))
                elif event.get("event") == "status":
                    topic = event.get("topic", "")
                    status = event.get("status", "")
                    if (status.startswith("Checking build") or status.startswith("Built ")
                            or status.endswith("is already up to date") or status.startswith("Launching ")):
                        _LOGGER.info("ROS manager: %s", status)
                    self._on_status(topic, status)

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

    @staticmethod
    def _send_bridge_command(process: subprocess.Popen, command: dict) -> None:
        if process.stdin is None:
            raise RuntimeError("WSL bridge input is unavailable")
        process.stdin.write(json.dumps(command, separators=(",", ":")) + "\n")
        process.stdin.flush()

    @staticmethod
    def _wsl_path(wsl: str, distro: str, windows_path: Path) -> str:
        result = subprocess.run(
            [wsl, "--distribution", distro, "--exec", "wslpath", "-u", str(windows_path)],
            capture_output=True,
            text=True,
            timeout=10,
            check=False,
        )
        if result.returncode != 0:
            details = result.stderr.strip() or "wslpath failed"
            raise RuntimeError(details)
        return result.stdout.strip()
