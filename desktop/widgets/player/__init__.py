"""Compose the shared Static Animator controls and its two views."""

import json
import math

from documents.snapshot_units import clamp_to_bounds
from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QSplitter, QVBoxLayout, QWidget
from servo_limits import get_servo_limits

from .creator_view import CreatorView
from .keyframes_view import KeyframesView
from .snapshot_loader import load_keyframe


class StaticAnimatorWidget(QWidget):
    """Place Creator and Keyframes side by side under shared motion controls."""

    load_requested = Signal(str)
    play_requested = Signal(str)

    def __init__(self, parent=None, *, directory=None, ros_manager=None):
        super().__init__(parent)
        self.ros_manager = ros_manager
        self._state_token = None
        self._pending_commands = {}
        self.limits = get_servo_limits(ros_manager)
        self._pose_edit = None
        layout = QVBoxLayout(self)
        self.splitter = QSplitter(Qt.Orientation.Horizontal, self)
        self.creator_view = CreatorView(self.splitter)
        self.creator_view.set_servo_limits(self.limits.bounds)
        self.limits.changed.connect(self._limits_changed)
        self.keyframes_view = KeyframesView(self.splitter, directory=directory)
        self.limits.status_changed.connect(self.keyframes_view.status_label.setText)
        self.splitter.addWidget(self.creator_view)
        self.splitter.addWidget(self.keyframes_view)
        self.splitter.setStretchFactor(0, 2)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setSizes([660, 330])
        layout.addWidget(self.splitter)

        self.keyframes_view.load_requested.connect(self._load_keyframe)
        self.keyframes_view.play_requested.connect(self._play_keyframe)
        self.creator_view.save_requested.connect(self._save_keyframe)
        self.creator_view.finish_edit_button.clicked.connect(self._finish_pose_edit)
        self.creator_view.release_all_torque_requested.connect(self._release_all_torque)
        self.creator_view.move_requested.connect(self._move_servo)
        self.creator_view.puppet_mode_requested.connect(self._puppet_mode_changed)
        if ros_manager is not None:
            self.creator_view.set_ros_available(True)
            try:
                self._state_token = ros_manager.request_topic(
                    "/saprog/joint_states", "sensor_msgs/msg/JointState",
                    package="saprog_dynamixel", executable="dynamixel_control_node",
                )
                ros_manager.message_received.connect(self._on_ros_message)
                ros_manager.command_result.connect(self._on_command_result)
                manager_ref = [ros_manager]
                ros_manager.destroyed.connect(lambda _obj=None: manager_ref.__setitem__(0, None))
                self.destroyed.connect(
                    lambda _obj=None: manager_ref[0].release(self._state_token)
                    if manager_ref[0] is not None and self._state_token else None
                )
            except (AttributeError, RuntimeError, ValueError) as error:
                self.creator_view.set_ros_available(False)
                self.keyframes_view.status_label.setText(f"ROS unavailable: {error}")

    def showEvent(self, event):
        super().showEvent(event)
        self.refresh_servo_limits()

    def refresh_servo_limits(self):
        self.limits.refresh()

    def _limits_changed(self, bounds):
        self.creator_view.set_servo_limits(bounds)
        commands = [key for key, (operation, _joints) in self._pending_commands.items()
                    if operation == "position"]
        for key in commands:
            self._pending_commands.pop(key)
        if commands:
            self.ros_manager.cancel_commands(commands)

    def _load_keyframe(self, path):
        try:
            count = self.creator_view.load_keyframe_fields(path)
        except (OSError, json.JSONDecodeError, ValueError) as error:
            self.keyframes_view.status_label.setText(f"Could not load keyframe: {error}")
            return
        self._finish_pose_edit()
        self.keyframes_view.status_label.setText(f"Loaded {count} servo targets")
        self.load_requested.emit(path)

    def edit_pose(self, name, positions, apply_changes):
        """Edit an in-memory recorder pose using this existing animator."""
        self._pose_edit = (tuple(positions), apply_changes)
        for servo, field in self.creator_view.target_inputs.items():
            self.creator_view.set_puppet_mode(servo, False)
            field.setText(repr(positions[servo]) if servo in positions else "")
        self.creator_view.name_input.setText(name)
        self.creator_view.name_input.setReadOnly(True)
        self.creator_view.save_button.setText("Apply pose to Motion Recorder")
        self.creator_view.finish_edit_button.show()
        self.keyframes_view.status_label.setText(f"Editing {name} from Motion Recorder")

    def _finish_pose_edit(self):
        self._pose_edit = None
        self.creator_view.name_input.setReadOnly(False)
        self.creator_view.save_button.setText("Save targets")
        self.creator_view.finish_edit_button.hide()

    def _move_servo(self, servo):
        if self.creator_view.puppet_buttons[servo].isChecked():
            return
        value = self._target_value(servo)
        if value is None:
            return
        # The hardware position path writes goal, speed and strength before
        # enabling torque, so a released joint cannot jump to its old goal.
        self._send_positions([servo], [value], f"Moving {servo}")

    def _puppet_mode_changed(self, servo, active):
        if not active:
            self.keyframes_view.status_label.setText(
                f"Puppet Mode ended for {servo}; torque remains off and position is unchanged"
            )
            return
        self._cancel_servo_motion(servo)
        self._request_torque(servo)

    def _cancel_servo_motion(self, servo):
        command_ids = [
            command_id for command_id, (operation, joints) in self._pending_commands.items()
            if operation == "position" and servo in joints
        ]
        for command_id in command_ids:
            self._pending_commands.pop(command_id)
        if command_ids:
            self.ros_manager.cancel_commands(command_ids)

    def _release_all_torque(self):
        if self.ros_manager is None:
            self.keyframes_view.status_label.setText("ROS manager is unavailable")
            return
        try:
            command_id = self.ros_manager.release_all_torque()
        except (RuntimeError, TypeError, ValueError) as error:
            self.keyframes_view.status_label.setText(f"Could not release torque: {error}")
            return
        self._pending_commands.clear()
        puppets = tuple(
            servo for servo, button in self.creator_view.puppet_buttons.items()
            if button.isChecked()
        )
        for servo in puppets:
            self.creator_view.set_puppet_pending(servo, True)
        self._pending_commands[command_id] = ("release_all", puppets)
        self.keyframes_view.status_label.setText(
            "Cancelling queued motion and disabling torque for all servos"
        )

    def _request_torque(self, servo):
        if self.ros_manager is None:
            self.creator_view.set_puppet_mode(servo, False)
            self.keyframes_view.status_label.setText("ROS manager is unavailable")
            return
        try:
            command_id = self.ros_manager.set_servo_torque(servo, False)
        except (RuntimeError, TypeError, ValueError) as error:
            self.creator_view.set_puppet_mode(servo, False)
            self.keyframes_view.status_label.setText(f"Could not set torque: {error}")
            return
        self._pending_commands[command_id] = ("torque", servo)
        self.creator_view.set_puppet_pending(servo, True)
        self.keyframes_view.status_label.setText(f"Requesting torque off for {servo}")

    def _target_value(self, servo):
        field = self.creator_view.target_inputs[servo]
        try:
            value = float(field.text().strip())
        except ValueError:
            self.keyframes_view.status_label.setText(f"Enter a numeric target for {servo}")
            return None
        if not math.isfinite(value):
            self.keyframes_view.status_label.setText(f"Target for {servo} must be finite")
            return None
        limits = self.creator_view.servo_limits.get(servo)
        if limits:
            value = clamp_to_bounds(value, limits)
            field.setText(repr(value))
        return value

    def _send_positions(self, joints, positions, status):
        if any(self.creator_view.puppet_buttons[joint].isChecked() for joint in joints):
            self.keyframes_view.status_label.setText("Exit Puppet Mode before moving these servos")
            return False
        if self.ros_manager is None:
            self.keyframes_view.status_label.setText("ROS manager is unavailable")
            return False
        try:
            positions = list(self.limits.clamp(dict(zip(joints, positions))).values())
            command_id = self.ros_manager.publish_joint_command(
                joints, positions, max(1, self.creator_view.speed_slider.value()),
                self.creator_view.torque_slider.value(),
            )
        except (RuntimeError, ValueError, TypeError) as error:
            self.keyframes_view.status_label.setText(f"Could not send position: {error}")
            return False
        self._pending_commands[command_id] = ("position", tuple(joints))
        self.keyframes_view.status_label.setText(status)
        return True

    def _play_keyframe(self, path):
        try:
            name, positions = load_keyframe(
                path, self.creator_view.target_inputs, self.creator_view.servo_limits,
            )
        except (OSError, json.JSONDecodeError, ValueError) as error:
            self.keyframes_view.status_label.setText(f"Could not play keyframe: {error}")
            return
        if self._send_positions(list(positions), list(positions.values()), f"Playing {name}"):
            self.play_requested.emit(path)

    def _on_ros_message(self, topic, message):
        if topic != "/saprog/joint_states" or not isinstance(message, dict):
            return
        names = message.get("name", ())
        positions = message.get("position", ())
        efforts = message.get("effort", ())
        for index, servo in enumerate(names):
            position = positions[index] if index < len(positions) else None
            torque = bool(efforts[index]) if index < len(efforts) else None
            self.creator_view.update_servo_state(servo, position, torque)

    def _on_command_result(self, command_id, success, message):
        pending = self._pending_commands.pop(command_id, None)
        if pending is None:
            return
        operation, servo = pending
        if operation == "torque":
            self.creator_view.set_puppet_pending(servo, False)
            if success:
                self.creator_view.update_servo_state(servo, torque_enabled=False)
            else:
                self.creator_view.set_puppet_mode(servo, False)
        elif operation == "release_all":
            for joint in servo:
                self.creator_view.set_puppet_pending(joint, False)
                if not success:
                    self.creator_view.set_puppet_mode(joint, False)
            if success:
                for joint in self.creator_view.target_inputs:
                    self.creator_view.update_servo_state(joint, torque_enabled=False)
        self.keyframes_view.status_label.setText(
            message if success else f"Command failed: {message}"
        )

    def _save_keyframe(self):
        if self._pose_edit is not None:
            joints, apply_changes = self._pose_edit
            try:
                apply_changes(self.creator_view.target_positions(joints))
            except ValueError as error:
                self.keyframes_view.status_label.setText(f"Could not apply pose: {error}")
                return
            self.keyframes_view.status_label.setText(
                "Pose updated in Motion Recorder. Save the timeline to keep these changes."
            )
            return
        try:
            path = self.creator_view.save_targets(self.keyframes_view.directory)
        except (OSError, ValueError) as error:
            self.keyframes_view.status_label.setText(f"Could not save keyframe: {error}")
            return
        if path is None:
            return
        self.keyframes_view.refresh()
        self.keyframes_view.status_label.setText(f"Saved {path.name}")


__all__ = ["CreatorView", "KeyframesView", "StaticAnimatorWidget"]
