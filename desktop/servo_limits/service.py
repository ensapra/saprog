"""Shared asynchronous EEPROM reads through the existing serial-port owner."""

from types import MappingProxyType

from documents.snapshot_units import clamp_to_bounds
from PySide6.QtCore import QObject, QTimer, Signal
from widgets.servo_layout import SERVO_NAMES

from .validation import hardware_report, report_bounds


class ServoLimits(QObject):
    changed = Signal(object)
    status_changed = Signal(str)

    def __init__(self, manager):
        super().__init__(manager if isinstance(manager, QObject) else None)
        self.manager = manager
        self._bounds = {}
        self._view = MappingProxyType(self._bounds)
        self.report = None
        self.pending = None
        self.token = None
        self.status = "Current EEPROM limits have not been read"
        self.timer = QTimer(self)
        self.timer.setSingleShot(True)
        self.timer.setInterval(35000)
        self.timer.timeout.connect(self.timed_out)
        if manager is not None:
            manager.command_result.connect(self.result)
            status_signal = getattr(manager, "status_changed", None)
            if status_signal is not None:
                status_signal.connect(self.provider_status)

    def provider_status(self, topic, status):
        if self.token is None or topic not in ("", "/saprog/joint_states"):
            return
        if status == "Connected":
            if not self.available and self.pending is None:
                self.refresh()
        elif status:
            command_id, self.pending = self.pending, None
            self.timer.stop()
            if command_id is not None:
                self.manager.cancel_commands([command_id])
            if self.available or command_id is not None:
                self.invalidate(f"EEPROM limits unavailable: {status}")

    @property
    def bounds(self):
        return self._view

    @property
    def available(self):
        return bool(self._bounds)

    def _status(self, message):
        self.status = message
        self.status_changed.emit(message)

    def refresh(self):
        if self.pending is not None:
            return self.pending
        if self.manager is None:
            self.invalidate("ROS manager is unavailable; current EEPROM limits are required")
            return None
        try:
            if self.token is None:
                self.token = self.manager.request_topic(
                    "/saprog/joint_states", "sensor_msgs/msg/JointState",
                    package="saprog_dynamixel", executable="dynamixel_control_node")
            reader = getattr(self.manager, "_read_servo_limits", None)
            self.pending = reader() if reader is not None else self.manager.refresh_servo_limits()
            self.timer.start()
            self._status("Reading current servo limits from hardware...")
            return self.pending
        except (AttributeError, RuntimeError, ValueError) as error:
            self.invalidate(f"Could not read EEPROM limits: {error}")
            return None

    def result(self, command_id, success, message):
        if self.pending is None or command_id != self.pending:
            return
        self.pending = None
        self.timer.stop()
        if not success:
            self.invalidate(f"Could not read EEPROM limits: {message}")
            return
        try:
            report = hardware_report(message, SERVO_NAMES)
        except ValueError as error:
            self.invalidate(f"Invalid EEPROM limit report: {error}")
            return
        previous = self.report
        updated = report_bounds(report)
        changed = updated != self._bounds or previous is None or previous["servos"] != report["servos"]
        self.report = report
        self._bounds.clear()
        self._bounds.update(updated)
        if changed:
            self.changed.emit(self.bounds)
        self._status("Servo limits updated from hardware (shared for this session)")

    def invalidate(self, message):
        self.report = None
        self._bounds.clear()
        self.changed.emit(self.bounds)
        self._status(message)

    def timed_out(self):
        if self.pending is None:
            return
        command_id, self.pending = self.pending, None
        if command_id is not None:
            self.manager.cancel_commands([command_id])
        self.invalidate("EEPROM limit read timed out; re-enter a motion tab to retry")

    def require(self, joints):
        if not self.available:
            self.refresh()
            raise ValueError("Current EEPROM limits are unavailable; wait for a successful hardware read")
        missing = set(joints)-self._bounds.keys()
        if missing:
            raise ValueError("No hardware limits for " + ", ".join(sorted(missing)))
        return self.bounds

    def clamp(self, positions):
        bounds = self.require(positions)
        return {joint: clamp_to_bounds(value, bounds[joint]) for joint, value in positions.items()}

    def close(self):
        self.timer.stop()
        if self.pending is not None:
            self.manager.cancel_commands([self.pending])
            self.pending = None
        if self.token is not None:
            token, self.token = self.token, None
            self.manager.release(token)
        self.invalidate("Limits session closed")


def get_servo_limits(manager):
    if manager is None:
        return ServoLimits(None)
    service = vars(manager).get("_servo_limits_service")
    if service is None:
        service = ServoLimits(manager)
        manager._servo_limits_service = service
    return service
