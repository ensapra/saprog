"""Public Qt API for optional ROS data requests from desktop widgets."""

import logging
import math
import queue
import re
import threading
import uuid
from pathlib import Path

from PySide6.QtCore import QObject, Signal

from .models import NodeRequirement, TopicRequest
from .node_launcher import RosNodeLauncher
from .ros_worker import RosWorker

_LOGGER = logging.getLogger("saprog.ros_manager")


class RosDataManager(QObject):
    """Share ROS subscriptions and node startup across desktop widgets.

    ``message_received`` emits ``(topic, plain_message_dict)``. ROS message
    types are supplied as strings such as ``sensor_msgs/msg/Imu`` so importing
    this manager does not require ROS to be installed.
    """

    message_received = Signal(str, object)
    status_changed = Signal(str, str)
    command_result = Signal(str, bool, str)
    position_published = Signal(str, object)
    torque_release_requested = Signal()

    def __init__(self, workspace: Path | None = None, parent=None):
        super().__init__(parent)
        _LOGGER.setLevel(logging.INFO)
        if not _LOGGER.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
            _LOGGER.addHandler(handler)
            _LOGGER.propagate = False
        ros_workspace = Path(workspace) if workspace else Path(__file__).resolve().parents[2] / "ros2"
        self._launcher = RosNodeLauncher(ros_workspace)
        self._lock = threading.RLock()
        self._requests: dict[str, TopicRequest] = {}
        self._topic_statuses: dict[str, str] = {}
        self._commands = queue.Queue()
        self._command_lock = threading.Lock()
        self._worker: RosWorker | None = None
        self._pending_positions = set()
        self.command_result.connect(self._position_finished)
        self.servo_limits.changed.connect(self._limits_updated)

    def _position_finished(self, command_id, _success, _message):
        self._pending_positions.discard(command_id)

    def _limits_updated(self, _bounds):
        pending, self._pending_positions = self._pending_positions, set()
        if pending:
            self.cancel_commands(pending)

    def request_topic(
        self,
        topic: str,
        message_type: str,
        *,
        package: str,
        executable: str,
    ) -> str:
        """Request a topic and return a token for :meth:`release`.

        The requested executable is checked and started only if the topic has
        no publisher. If it is not installed, the matching package is built.
        """
        if not topic.startswith("/"):
            raise ValueError("ROS topic names must be absolute")
        if not message_type or "/msg/" not in message_type:
            raise ValueError("message_type must use the ROS form 'package/msg/Type'")
        if not package or not executable:
            raise ValueError("package and executable are required")

        request = TopicRequest(topic, message_type, NodeRequirement(package, executable))
        _LOGGER.info(
            "Node requested: %s/%s (topic: %s, type: %s)",
            package, executable, topic, message_type,
        )
        with self._lock:
            if any(
                existing.topic == topic and existing != request
                for existing in self._requests.values()
            ):
                raise ValueError(f"Topic {topic} was already requested with a different type or provider")
            token = uuid.uuid4().hex
            self._requests[token] = request
            self._ensure_worker_locked()
            self._emit_status(
                topic, self._topic_statuses.get(topic, "Waiting for ROS 2 data")
            )
        return token

    def publish_joint_command(self, joints, positions, speed, torque):
        """Queue a position command with AX speed and torque settings."""
        joints = list(joints)
        positions = [float(value) for value in positions]
        speed, torque = int(speed), int(torque)
        if not joints or len(joints) != len(positions):
            raise ValueError("Position commands need matching joint names and positions")
        if len(set(joints)) != len(joints):
            raise ValueError("Position commands cannot repeat a servo")
        if any(not re.fullmatch(r"servo_\d+", joint) for joint in joints):
            raise ValueError("Position commands need servo_N joint names")
        if not all(math.isfinite(value) for value in positions):
            raise ValueError("Position command values must be finite")
        if not 1 <= speed <= 1023 or not 0 <= torque <= 1023:
            raise ValueError("Speed must be 1–1023 and torque strength 0–1023")
        # Enforce the same live limits even for callers outside the desktop tabs.
        positions = list(self.servo_limits.clamp(dict(zip(joints, positions))).values())
        command_id = self._queue_command({
            "op": "publish_position",
            "joints": joints,
            "positions": positions,
            "speed": speed,
            "torque": torque,
        })
        self._pending_positions.add(command_id)
        return command_id

    def refresh_servo_limits(self):
        """Refresh the shared, validated EEPROM report for every consumer."""
        return self.servo_limits.refresh()

    def _read_servo_limits(self):
        """Transport operation used only by the shared limits process."""
        return self._queue_command({"op": "refresh_servo_limits"})

    @property
    def servo_limits(self):
        from servo_limits import get_servo_limits

        return get_servo_limits(self)

    def simulation_command(self, action, **options):
        """Send only to the Gazebo supervisor, never to hardware endpoints."""
        if action not in {"start", "stop", "pause", "resume", "reset", "show_viewer", "hide_viewer"}:
            raise ValueError("Unknown simulation action")
        return self._queue_command({"op": "simulation", "payload": {**options, "action": action}})

    def set_servo_torque(self, joint, enabled):
        """Queue a per-servo torque enable or disable service request."""
        if not isinstance(joint, str) or not re.fullmatch(r"servo_\d+", joint):
            raise ValueError("Torque commands need a servo_N joint name")
        return self._queue_command({
            "op": "set_torque",
            "joint": joint,
            "enabled": bool(enabled),
        })

    def release_all_torque(self):
        """Cancel queued commands and ask the ROS node to disable every servo."""
        # Stop producers before draining their queues, including controllers
        # dwelling between steps with no currently pending command.
        self.torque_release_requested.emit()
        command = {"op": "release_all_torque"}
        command["id"] = uuid.uuid4().hex
        cancelled = []
        with self._command_lock:
            while True:
                try:
                    pending = self._commands.get_nowait()
                except queue.Empty:
                    break
                if "id" in pending:
                    cancelled.append(pending["id"])
            self._commands.put(command)
        for command_id in cancelled:
            self.command_result.emit(command_id, False, "Cancelled by Release all torque")
        with self._lock:
            self._ensure_worker_locked()
        return command["id"]

    def cancel_commands(self, command_ids):
        """Cancel these commands if they have not yet been sent to hardware.

        Forward cancellation to WSL as well, where commands can be waiting for
        a subscriber or service. Already published positions cannot be undone.
        """
        command_ids = set(command_ids)
        if not command_ids:
            return
        cancelled = []
        with self._command_lock:
            retained = []
            while True:
                try:
                    command = self._commands.get_nowait()
                except queue.Empty:
                    break
                if command.get("id") not in command_ids:
                    retained.append(command)
                else:
                    cancelled.append(command["id"])
            for command in retained:
                self._commands.put(command)
            self._commands.put({"op": "cancel_commands", "ids": list(command_ids)})
        # Receivers may queue or cancel other commands; never emit under the lock.
        for command_id in cancelled:
            self.command_result.emit(command_id, False, "Command cancelled before dispatch")

    def _queue_command(self, command):
        command["id"] = uuid.uuid4().hex
        with self._command_lock:
            self._commands.put(command)
        with self._lock:
            self._ensure_worker_locked()
        return command["id"]

    def _ensure_worker_locked(self):
        if self._worker is None or not self._worker.is_alive():
            self._topic_statuses.clear()
            self._worker = RosWorker(
                self._active_requests,
                self._drain_commands,
                self.message_received.emit,
                self._emit_status,
                self.command_result.emit,
                self._launcher,
                on_position_published=self.position_published.emit,
            )
            self._worker.start()

    def _drain_commands(self):
        commands = []
        with self._command_lock:
            while True:
                try:
                    commands.append(self._commands.get_nowait())
                except queue.Empty:
                    return commands

    def release(self, token: str) -> None:
        """Release a widget's subscription request."""
        with self._lock:
            request = self._requests.pop(token, None)
            still_requested = request is not None and any(
                existing.topic == request.topic for existing in self._requests.values()
            )
        if request:
            if not still_requested:
                with self._lock:
                    self._topic_statuses.pop(request.topic, None)
                self.status_changed.emit(request.topic, "No longer requested")

    def close(self, timeout: float = 5.0) -> None:
        """Stop the ROS worker and the node processes it launched."""
        limits = vars(self).get("_servo_limits_service")
        if limits is not None:
            limits.close()
        with self._lock:
            worker = self._worker
            self._requests.clear()
        if worker is not None and worker.is_alive():
            worker.stop()
            if worker is not threading.current_thread():
                worker.join(timeout)
        with self._lock:
            if self._worker is worker and (worker is None or not worker.is_alive()):
                self._worker = None

    def _active_requests(self) -> dict[str, TopicRequest]:
        with self._lock:
            return dict(self._requests)

    def _emit_status(self, topic: str, status: str) -> None:
        """Cache per-topic state so new subscribers see the current status."""
        if topic:
            with self._lock:
                self._topic_statuses[topic] = status
        self.status_changed.emit(topic, status)
