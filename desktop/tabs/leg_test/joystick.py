"""Proportional Cartesian stick control for one leg, with optional ROS output."""

import math
import time
from collections import deque

import numpy as np
from documents.snapshot_units import AX_TICK_RADIANS
from PySide6.QtCore import QObject, QTimer, Signal
from servo_limits import get_servo_limits

STATE_TOPIC = "/saprog/joint_states"


class CartesianJoystick(QObject):
    pose_changed = Signal(object)
    status_changed = Signal(str)
    active_changed = Signal(bool)

    def __init__(self, manager=None, parent=None, *, clock=time.monotonic, read_input=None):
        super().__init__(parent)
        self.manager, self.clock = manager, clock
        self.read_input = read_input
        self.limits = get_servo_limits(manager)
        self.active = False
        self.hardware = False
        self.pending = None
        self.token = None
        self.axes = ()
        self.pressed = set()
        self.feedback = {}
        self._clear_motion()
        self.last_direction = np.zeros(3)
        self.issued = deque(maxlen=32)
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.tick)
        self.limits.changed.connect(self._limits_changed)
        if manager is not None:
            manager.message_received.connect(self.observe)
            manager.command_result.connect(self.command_result)
            manager.status_changed.connect(self.provider_status)
            manager.position_published.connect(self.position_published)
            manager.torque_release_requested.connect(self.stop)

    def update_input(self, axes, pressed):
        self.axes, self.pressed = tuple(axes), set(pressed)
        if self.active and (not self.axes or max(self.axis_indices) >= len(self.axes) or any(
            not math.isfinite(v) or abs(v) > 1.001 for v in self.axes
        )):
            self.stop("Joystick stopped: controller input unavailable")

    def _sample_input(self):
        if self.read_input is not None:
            axes, pressed = self.read_input()
            self.update_input(axes, pressed)
        return bool(self.axes)

    def _clear_motion(self):
        self.motion_elapsed = 0.
        self.motion_delta = np.zeros(3)

    def _rebase(self, measured):
        self.pose = dict(measured)
        if self.hardware and self.x1_centered:
            self.pose[self.x1] = 0.
        self.target = self.model.foot_position(self.pose)
        self.target[1-self.horizontal] = self.origin[1-self.horizontal]
        self._clear_motion()

    def start(self, model, seed, *, axes=(0, 1), horizontal=0, reverse=(False, False),
              speed=.01, deadzone=.15, hardware=False, enable_button="button:0",
              servo_speed=100, torque=300, enable_button_name=None):
        self.stop()
        if not self._sample_input():
            raise ValueError("Detect and select a connected controller first")
        self.limits.require(m.servo for m in model.mappings)
        if (len(axes) != 2 or axes[0] == axes[1] or
                any(type(a) is not int or not 0 <= a < len(self.axes) for a in axes)):
            raise ValueError("Choose two different connected stick axes")
        if horizontal not in (0, 1):
            raise ValueError("Horizontal motion must use base X or Y")
        if (not math.isfinite(speed) or not 0 < speed <= .05 or
                not math.isfinite(deadzone) or not 0 <= deadzone < 1):
            raise ValueError("Choose a speed up to 50 mm/s and a dead zone below 1")
        if any(not math.isfinite(v) or abs(v) > 1.001 for v in self.axes):
            raise ValueError("Controller returned invalid stick values")
        if hardware and (self.manager is None or not enable_button):
            raise ValueError("Live control needs ROS and a hold-to-move button")
        x1 = model.mappings[0].servo
        if hardware and not model.bounds[x1][0] <= 0 <= model.bounds[x1][1]:
            raise ValueError(f"Live control needs {x1} at 0 rad, outside its current EEPROM limits")
        if not hardware and not model.in_limits(seed):
            raise ValueError("Start from a pose inside current hardware limits")
        self.model, self.pose = model, dict(seed)
        self.axis_indices, self.horizontal, self.reverse = axes, horizontal, reverse
        self.speed, self.deadzone, self.hardware = speed, deadzone, hardware
        self.x1, self.x1_centered, self.x1_commanded = x1, False, False
        self.enable_button = enable_button
        button_name = enable_button_name or (
            f"Button {int(enable_button[7:])+1}"
            if isinstance(enable_button, str) and enable_button.startswith("button:")
            and enable_button[7:].isdigit() else "the selected button"
        )
        self.hold_instruction = f"Hold {button_name} on your controller with the stick centered"
        self.servo_speed, self.torque = servo_speed, torque
        self.feedback.clear()
        self.ready = not hardware
        self.neutral = False
        self._clear_motion()
        self.last_direction = np.zeros(3)
        self.target = model.foot_position(seed)
        self.origin = self.target.copy()
        self.started = self.last_tick = self.clock()
        try:
            if hardware:
                self.token = self.manager.request_topic(
                    STATE_TOPIC, "sensor_msgs/msg/JointState",
                    package="saprog_dynamixel", executable="dynamixel_control_node",
                )
            self.active = True
            self.active_changed.emit(True)
            self.timer.start()
            self.status_changed.emit("Center the stick to begin" if not hardware else
                                     "Waiting for fresh leg feedback. " + self.hold_instruction)
        except (RuntimeError, ValueError):
            self.stop()
            raise

    def stop(self, message="Joystick stopped; a target already sent may finish moving"):
        was_active, self.active = self.active, False
        self.timer.stop()
        self._cancel_pending()
        token, self.token = self.token, None
        if token is not None:
            self.manager.release(token)
        if was_active:
            self.active_changed.emit(False)
            self.status_changed.emit(message)

    def _cancel_pending(self):
        pending, self.pending = self.pending, None
        if pending is not None:
            self.manager.cancel_commands([pending])

    def _publish(self, pose, now, *, duration=None):
        speeds = self.servo_speed
        if duration is not None:
            # Give each motor the rate of its continuous joint trajectory.
            # Sending every tiny waypoint at the ceiling makes fast motors
            # arrive early and stop between successive 50 ms targets.
            unit = .111*2*math.pi/60
            speeds = [max(1, min(self.servo_speed,
                                int(math.ceil(abs(pose[s]-self.pose[s])/duration/unit))))
                      for s in pose]
        self.pending = self.manager.publish_joint_command(
            list(pose), list(pose.values()), speeds, self.torque,
        )
        self.issued.append(self.pending)
        self.sent_at = now

    def _center_x1(self, measured, direction, now):
        """Center x1 under the held button before accepting X/Z input."""
        if self.x1_centered:
            if abs(measured[self.x1]) <= AX_TICK_RADIANS+1e-9:
                return False
            self.x1_centered = self.x1_commanded = False
        self._clear_motion()
        self.neutral = False
        if self.pending is not None:
            if now-self.sent_at > 1:
                self.stop("Joystick stopped: x1 command acknowledgement timed out")
            return True
        if not self.x1_commanded and abs(measured[self.x1]) > 1e-9:
            if np.any(direction):
                self.status_changed.emit(f"Center the stick while holding the controller button to set {self.x1} to 0 rad")
                return True
            try:
                self._publish({self.x1: 0.}, now)
            except (RuntimeError, ValueError) as error:
                self.stop(f"Could not center {self.x1}: {error}")
                return True
            self.x1_commanded = True
            rate = self.servo_speed*(.111*2*math.pi/60)
            self.x1_deadline = now + max(3., abs(measured[self.x1])/rate+2)
            self.status_changed.emit(f"Moving {self.x1} to 0 rad; keep the controller button held")
            return True
        if abs(measured[self.x1]) > AX_TICK_RADIANS+1e-9:
            if now > self.x1_deadline:
                self.stop(f"Joystick stopped: {self.x1} did not reach 0 rad")
            else:
                self.status_changed.emit(f"Waiting for {self.x1} to reach 0 rad")
            return True
        self.x1_centered = True
        self.pose = dict(measured)
        self.pose[self.x1] = 0.
        self.target = self.model.foot_position(self.pose)
        self.origin = self.target.copy()
        return False

    def _limits_changed(self, _bounds):
        if self.active:
            self.stop("Joystick stopped: hardware limits changed; start again")

    def provider_status(self, topic, status):
        if self.active and self.hardware and self.ready and topic in (STATE_TOPIC, "") and status != "Connected":
            self.stop(f"Joystick stopped: {status}")

    def position_published(self, command_id, _stamp):
        if self.active and self.hardware and command_id not in self.issued:
            self.stop("Joystick stopped: another control published a servo target")

    def command_result(self, command_id, success, message):
        if command_id != self.pending:
            return
        self.pending = None
        if not success:
            self.stop(f"Joystick command failed: {message}")

    def observe(self, topic, message):
        if not self.active or not self.hardware or topic != STATE_TOPIC or not isinstance(message, dict):
            return
        names, positions = message.get("name"), message.get("position")
        if not isinstance(names, (tuple, list)) or not isinstance(positions, (tuple, list)):
            return
        servos = {m.servo for m in self.model.mappings}
        for name, value in zip(names, positions):
            if isinstance(name, str) and name in servos and type(value) in (int, float) and math.isfinite(value):
                self.feedback[name] = (float(value), self.clock())

    def _direction(self):
        values = np.array([self.axes[i] for i in self.axis_indices])
        values = np.sign(values) * np.maximum(0, (abs(values)-self.deadzone)/(1-self.deadzone))
        values *= [-1 if self.reverse[0] else 1, 1 if self.reverse[1] else -1]
        values /= max(1, np.linalg.norm(values))
        direction = np.zeros(3)
        direction[self.horizontal], direction[2] = values
        return direction

    def tick(self):
        if not self.active:
            return
        if not self._sample_input():
            self.stop("Joystick stopped: controller disconnected")
            return
        if not self.active:
            return
        now = self.clock()
        dt, self.last_tick = min(.1, max(0, now-self.last_tick)), now
        measured = None
        if self.hardware:
            if any(m.servo not in self.feedback or now-self.feedback[m.servo][1] > 2
                   for m in self.model.mappings):
                if self.ready or now-self.started > 5:
                    self.stop("Joystick stopped: leg feedback is stale or incomplete")
                return
            measured = {m.servo: self.feedback[m.servo][0] for m in self.model.mappings}
            if not self.model.in_limits(measured):
                self.stop("Joystick stopped: measured pose is outside hardware limits")
                return
            self.pose_changed.emit(measured)
            if not self.ready:
                self.pose = measured
                self.target = self.model.foot_position(measured)
                self.origin = self.target.copy()
                self.ready = True
            if self.enable_button not in self.pressed:
                self._cancel_pending()
                self.neutral = False
                if not self.x1_centered:
                    self.x1_commanded = False
                self._rebase(measured)
                self.last_direction = np.zeros(3)
                self.status_changed.emit(self.hold_instruction + " to move the leg")
                return
        direction = self._direction()
        if measured is not None and self._center_x1(measured, direction, now):
            return
        if not np.any(direction):
            self._clear_motion()
            self.last_direction = np.zeros(3)
            if self.pending is not None:
                self._cancel_pending()
                self._rebase(measured)
            self.neutral = True
            self.status_changed.emit("Stick centered: holding foot target")
            return
        if not self.neutral:
            self.status_changed.emit("Center the stick before moving")
            return
        if measured is not None and np.any(direction*self.last_direction < 0):
            # Cancel unpublished motion in the old direction and reverse from
            # the actual pose, instead of making the leg chase an old target.
            self._cancel_pending()
            self._rebase(measured)
        self.last_direction = direction
        # Retain input across publication delays, but never catch up more than
        # a quarter second after a delayed tick or acknowledgement.
        credit = min(dt, max(0., .25-self.motion_elapsed))
        self.motion_elapsed += credit
        self.motion_delta += direction*self.speed*credit
        if self.pending is not None:
            if now-self.sent_at > 1:
                self.stop("Joystick stopped: servo command acknowledgement timed out")
            return
        if self.motion_elapsed <= 0:
            return
        displacement = self.motion_delta.copy()
        seed, reference = self.pose, self.target
        max_joint_step = None
        if measured is not None:
            rate = self.servo_speed*(.111*2*math.pi/60)
            following_limit = max(4*AX_TICK_RADIANS, rate*.1)
            if max(abs(self.pose[s]-measured[s]) for s in self.pose) > following_limit:
                self._clear_motion()
                self.status_changed.emit("Waiting for servos to follow the target; check live torque and load if the leg stays still")
                return
            # Integrate from the last continuous target. Restarting each step
            # from quantized feedback loses sub-tick motion and can stall forever.
            max_joint_step = rate*self.motion_elapsed
        duration = self.motion_elapsed
        self._clear_motion()
        try:
            pose, target = self.model.cartesian_jog(
                displacement, seed, target=reference, max_joint_step=max_joint_step,
                fixed={self.x1: 0.} if self.hardware else None,
            )
        except (ValueError, np.linalg.LinAlgError) as error:
            self.status_changed.emit(f"Foot held at motion boundary: {error}")
            return
        if max(abs(pose[s]-seed[s]) for s in pose) < 1e-10:
            self.target = target
            self.status_changed.emit("Motion blocked by servo limits or linkage reach; move the stick back")
            return
        if self.hardware:
            try:
                self._publish(pose, now, duration=duration)
            except (RuntimeError, ValueError) as error:
                self.stop(f"Joystick command failed: {error}")
                return
        self.target, self.pose = target, pose
        if not self.hardware:
            self.pose_changed.emit(pose)
        self.status_changed.emit("Joystick live control" if self.hardware else "Joystick preview")
