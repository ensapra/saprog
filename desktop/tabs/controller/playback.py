"""Button-owned timeline playback, with all hardware commands sent via ros_manager."""

import json
import math
import time
from collections import defaultdict
from pathlib import Path

from documents.snapshot_units import clamp_to_bounds, validate_snapshot
from documents.timeline_domain import (
    mirror_mode,
    mirrored_positions,
    motion_settings,
    transition_duration,
)
from documents.timeline_storage import (
    load_snapshot_reference,
    load_timeline,
)
from PySide6.QtCore import QObject, QTimer, Signal
from widgets.servo_layout import SERVO_NAMES


def measured_bounds():
    path = Path(__file__).resolve().parents[3] / "urdf/config/measured_servo_limits.json"
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
        return {
            f"servo_{item['servo_id']}": (item["lower_radians"], item["upper_radians"])
            for item in document["servos"]
        }
    except (OSError, ValueError, KeyError, TypeError):
        return None


class ControllerPlayback(QObject):
    """Run one sequence at a time, either once or repeatedly while held.

    A command must be acknowledged before the next group is queued. Stopping
    cancels unpublished commands; a target already published may finish moving.
    Sequence file loop/end-torque flags do not apply to controller playback.
    """

    status_changed = Signal(str)
    step_ready = Signal(int)
    step_changed = Signal(int)
    STATE_TOPIC = "/saprog/joint_states"
    ARRIVAL_TOLERANCE = 0.1  # Radians; allow for servo settling error under load.
    SPEED_UNIT_RADIANS = 0.111 * 2 * math.pi / 60

    def __init__(self, ros_manager, snapshot_directory, parent=None, *, clock=time.monotonic):
        super().__init__(parent)
        self.manager = ros_manager
        self.snapshot_directory = Path(snapshot_directory)
        self.clock = clock
        self.bounds = measured_bounds()
        self.owner = None
        self.repeat = True
        self.smooth = False
        self.steps = []
        self.index = -1
        self.phase = "idle"
        self.pending = set()
        self.groups = []
        self.targets = {}
        self.deadline = 0
        self.stable_samples = 0
        self.last_stamp = 0
        self.sent_at = None
        self.token = None
        self.timer = QTimer(self)
        self.timer.setInterval(30)
        self.timer.timeout.connect(self.tick)
        if self.manager is not None:
            self.manager.command_result.connect(self.command_result)
            self.manager.position_published.connect(self.position_published)
            self.manager.message_received.connect(self.observe)
            self.manager.torque_release_requested.connect(self._torque_release_requested)

    def _torque_release_requested(self):
        if self.owner is not None or self.pending:
            self.stop("Controller playback stopped: all torque release requested")

    def _require_manager(self):
        if self.manager is None:
            raise ValueError("ROS manager is unavailable")
        if self.token is None:
            self.token = self.manager.request_topic(
                self.STATE_TOPIC, "sensor_msgs/msg/JointState",
                package="saprog_dynamixel", executable="dynamixel_control_node",
            )

    def _snapshot(self, path):
        snapshot = load_snapshot_reference(
            str(Path(path).resolve()), self.snapshot_directory, bounds=self.bounds,
        )
        positions = snapshot["positions"]
        if set(positions) - set(SERVO_NAMES):
            raise ValueError("Snapshot contains unknown servos")
        validate_snapshot(snapshot, bounds=self.bounds)
        return snapshot

    def snapshot(self, path, speed=512, torque=1023):
        snapshot = self._snapshot(path)
        self.stop()
        self._require_manager()
        self.pending.add(self.manager.publish_joint_command(
            list(snapshot["positions"]), list(snapshot["positions"].values()), speed, torque,
        ))
        self.status_changed.emit(f"Sending snapshot: {snapshot['name']}")

    def torque(self, enabled):
        self.stop()
        self._require_manager()
        if enabled:
            for joint in SERVO_NAMES:
                self.pending.add(self.manager.set_servo_torque(joint, True))
        else:
            self.pending.add(self.manager.release_all_torque())
        self.status_changed.emit(f"Requesting torque {'on' if enabled else 'off'} for all servos")

    def start(self, owner, path, *, repeat=True, smooth=None):
        document = load_timeline(path, self.snapshot_directory, bounds=self.bounds)
        if smooth is None:
            smooth = document.get("smooth_playback", False)
        self.start_steps(owner, document["steps"], repeat=repeat, smooth=smooth)

    def start_steps(self, owner, steps, *, repeat=False, smooth=False):
        """Play already edited timeline steps without requiring a saved file."""
        resolved = []
        for step in steps:
            if step.get("kind") == "wait":
                resolved.append(dict(step))
                continue
            reference = step.get("snapshot")
            path = Path(reference)
            if not path.is_absolute():
                path = self.snapshot_directory / path
            candidate = dict(step)
            candidate["snapshot"] = self._snapshot(path)
            candidate["snapshot"]["positions"] = mirrored_positions(
                candidate["snapshot"]["positions"], mirror_mode(step),
            )
            if self.bounds:
                candidate["snapshot"]["positions"] = {
                    joint: clamp_to_bounds(value, self.bounds[joint])
                    if joint in self.bounds else value
                    for joint, value in candidate["snapshot"]["positions"].items()
                }
            resolved.append(candidate)
        if not resolved:
            raise ValueError("Timeline has no steps to play")
        self.stop()
        self._require_manager()
        self.owner, self.steps, self.index = owner, resolved, -1
        self.repeat = repeat
        self.smooth = smooth
        self.status_changed.emit(
            "Repeating timeline" if repeat else "Playing timeline once"
        )
        self.timer.start()
        try:
            self._advance()
        except (OSError, RuntimeError, ValueError, TypeError, KeyError):
            self.stop()
            raise

    def release(self, owner):
        if self.owner == owner and self.repeat:
            self.stop("Sequence stopped: button released")

    def stop(self, message=None):
        pending = self.pending
        self.owner = None
        self.phase = "idle"
        self.pending = set()
        self.groups = []
        self.steps = []
        self.targets = {}
        self.sent_at = None
        self.last_stamp = 0
        self.stable_samples = 0
        self.timer.stop()
        if pending and self.manager is not None:
            self.manager.cancel_commands(pending)
        if message:
            self.status_changed.emit(message)

    def _advance(self):
        if self.owner is None:
            return
        previous = self.steps[self.index] if self.index >= 0 else None
        self.index += 1
        if self.index >= len(self.steps):
            if not self.repeat:
                self.stop("Sequence finished")
                return
            self.index = 0
        step = self.steps[self.index]
        if step.get("kind") == "wait":
            self.phase = "wait"
            self.deadline = self.clock() + step["seconds"]
            self.step_changed.emit(self.index)
            return
        self.targets = step["snapshot"]["positions"]
        groups = defaultdict(dict)
        duration = transition_duration(step)
        previous_positions = (
            previous["snapshot"]["positions"]
            if previous and previous.get("kind") != "wait" else {}
        )
        for joint, pair in motion_settings(step, self.targets).items():
            speed = pair["speed"]
            if (self.smooth and previous_positions and joint in previous_positions
                    and duration["mode"] == "seconds" and duration["seconds"] > 0):
                delta = abs(self.targets[joint] - previous_positions[joint])
                speed = min(speed, max(1, math.ceil(
                    delta / duration["seconds"] / self.SPEED_UNIT_RADIANS
                )))
            groups[(speed, pair["torque"])][joint] = self.targets[joint]
        self.groups = list(groups.items())
        self.phase = "publishing"
        self.sent_at = None
        self.deadline = self.clock() + 30
        self._send_group()

    def _send_group(self):
        (speed, torque), targets = self.groups.pop(0)
        self.pending.add(self.manager.publish_joint_command(
            list(targets), list(targets.values()), speed, torque,
        ))

    def command_result(self, command_id, success, message):
        if command_id not in self.pending:
            return
        self.pending.remove(command_id)
        if not success:
            self.stop(f"Controller command failed: {message}")
            return
        if self.owner is None:
            if not self.pending:
                self.status_changed.emit(message)
            return
        try:
            if self.groups:
                self._send_group()
                return
            duration = transition_duration(self.steps[self.index])
            self.phase = "wait" if duration["mode"] == "seconds" else "arrival"
            self.deadline = self.clock() + (
                duration["seconds"] if self.phase == "wait" else 30
            )
            self.stable_samples = 0
            self.step_changed.emit(self.index)
            self.step_ready.emit(self.index)
        except (RuntimeError, ValueError, TypeError) as error:
            self.stop(f"Sequence failed: {error}")

    def position_published(self, command_id, stamp):
        if self.owner is not None and command_id in self.pending:
            # Both this barrier and telemetry stamps belong to the WSL ROS
            # clock. Windows wall time can drift from it after suspend/resume.
            self.sent_at = stamp

    def observe(self, topic, message):
        if self.phase != "arrival" or topic != self.STATE_TOPIC or not isinstance(message, dict):
            return
        try:
            stamp_data = message["header"]["stamp"]
            stamp = int(stamp_data["sec"]) * 1_000_000_000 + int(stamp_data["nanosec"])
            if self.sent_at is None:
                return
            age = message["_ros_received_at_ns"] - stamp
            if stamp <= max(self.last_stamp, self.sent_at) or not 0 <= age <= 2_500_000_000:
                return
            self.last_stamp = stamp
            positions = dict(zip(message["name"], message["position"]))
            reached = all(
                joint in positions and math.isfinite(positions[joint])
                and abs(positions[joint] - value) <= self.ARRIVAL_TOLERANCE
                for joint, value in self.targets.items()
            )
        except (KeyError, TypeError, ValueError, OverflowError):
            reached = False
        self.stable_samples = self.stable_samples + 1 if reached else 0
        if self.stable_samples >= 3:
            self.phase = "wait"
            self.deadline = self.clock()

    def tick(self):
        if self.owner is None or self.clock() < self.deadline:
            return
        try:
            if self.phase == "wait":
                self._advance()
            else:
                self.stop("Sequence stopped: timed out waiting for ROS or target arrival")
        except (OSError, RuntimeError, ValueError, TypeError) as error:
            self.stop(f"Sequence failed: {error}")

    def close(self):
        self.stop()
        if self.token is not None:
            self.manager.release(self.token)
            self.token = None
