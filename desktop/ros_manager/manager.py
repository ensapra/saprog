"""Public Qt API for optional ROS data requests from desktop widgets."""

import logging
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

    def __init__(self, workspace: Path | None = None, parent=None):
        super().__init__(parent)
        _LOGGER.setLevel(logging.INFO)
        if not _LOGGER.handlers:
            handler = logging.StreamHandler()
            handler.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
            _LOGGER.addHandler(handler)
            _LOGGER.propagate = False
        _LOGGER.info("ROS data manager ready")
        ros_workspace = Path(workspace) if workspace else Path(__file__).resolve().parents[2] / "ros2"
        self._launcher = RosNodeLauncher(ros_workspace)
        self._lock = threading.RLock()
        self._requests: dict[str, TopicRequest] = {}
        self._worker: RosWorker | None = None

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
        with self._lock:
            if any(
                existing.topic == topic and existing != request
                for existing in self._requests.values()
            ):
                raise ValueError(f"Topic {topic} was already requested with a different type or provider")
            token = uuid.uuid4().hex
            self._requests[token] = request
            self.status_changed.emit(topic, "Waiting for ROS 2 data")
            if self._worker is None or not self._worker.is_alive():
                self._worker = RosWorker(
                    self._active_requests,
                    self.message_received.emit,
                    self.status_changed.emit,
                    self._launcher,
                )
                self._worker.start()
                _LOGGER.info("ROS data requested; starting ROS manager worker")
        return token

    def release(self, token: str) -> None:
        """Release a widget's subscription request."""
        with self._lock:
            request = self._requests.pop(token, None)
        if request:
            self.status_changed.emit(request.topic, "No longer requested")

    def close(self, timeout: float = 5.0) -> None:
        """Stop the ROS worker and the node processes it launched."""
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
