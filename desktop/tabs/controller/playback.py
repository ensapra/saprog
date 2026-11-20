"""Button-owned timeline playback, with all hardware commands sent via ros_manager."""

import json
import math
import time
from collections import defaultdict
from pathlib import Path

from documents.snapshot_units import validate_snapshot
from documents.timeline_domain import motion_settings, transition_duration
from documents.timeline_storage import (
    load_snapshot_reference,
    load_timeline,
    resolve_timeline_steps,
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
    STATE_TOPIC = "/saprog/joint_states"

    def __init__(self, ros_manager, snapshot_directory, parent=None, *, clock=time.monotonic):
        super().__init__(parent)
        self.manager = ros_manager
        self.snapshot_directory = Path(snapshot_directory)
        self.clock = clock
        self.bounds = measured_bounds()
        self.owner = None
        self.repeat = True
        self.steps = []
        self.index = -1
        self.phase = "idle"
        self.pending = set()
        self.groups = []
        self.targets = {}
        self.deadline = 0
        self.stable_samples = 0
        self.last_stamp = 0
        self.token = None
        self.timer = QTimer(self)
        self.timer.setInterval(30)
        self.timer.timeout.connect(self.tick)
        if self.manager is not None:
            self.manager.command_result.connect(self.command_result)
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

    def start(self, owner, path, *, repeat=True):
        document = load_timeline(path, self.snapshot_directory, bounds=self.bounds)
        steps = resolve_timeline_steps(
            document["steps"],
            lambda reference: self._snapshot(
                Path(reference) if Path(reference).is_absolute()
                else self.snapshot_directory / reference
            ),
        )
        self.stop()
        self._require_manager()
        self.owner, self.steps, self.index = owner, steps, -1
        self.repeat = repeat
        self.status_changed.emit(
            f"Repeating {Path(path).stem} while held" if repeat
            else f"Playing {Path(path).stem} once"
        )
        self.timer.start()
        self._advance()

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
        self.timer.stop()
        if pending and self.manager is not None:
            self.manager.cancel_commands(pending)
        if message:
            self.status_changed.emit(message)

    def _advance(self):
        if self.owner is None:
            return
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
            return
        self.targets = step["snapshot"]["positions"]
        groups = defaultdict(dict)
        for joint, pair in motion_settings(step, self.targets).items():
            groups[(pair["speed"], pair["torque"])][joint] = self.targets[joint]
        self.groups = list(groups.items())
        self.phase = "publishing"
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
            # Never let a buffered pre-command telemetry sample complete a step.
            self.sent_at = time.time_ns()
        except (RuntimeError, ValueError, TypeError) as error:
            self.stop(f"Sequence failed: {error}")

    def observe(self, topic, message):
        if self.phase != "arrival" or topic != self.STATE_TOPIC or not isinstance(message, dict):
            return
        try:
            stamp_data = message["header"]["stamp"]
            stamp = int(stamp_data["sec"]) * 1_000_000_000 + int(stamp_data["nanosec"])
            age = time.time_ns() - stamp
            if stamp <= max(self.last_stamp, self.sent_at) or not 0 <= age <= 2_500_000_000:
                return
            self.last_stamp = stamp
            positions = dict(zip(message["name"], message["position"]))
            reached = all(
                joint in positions and math.isfinite(positions[joint])
                and abs(positions[joint] - value) <= 0.025
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
