"""Capture manually guided joint motion and save it as a timeline."""

import copy
import math
import re
import tempfile
import time
from datetime import datetime
from pathlib import Path

from documents.snapshot_files import write_json_atomic
from documents.snapshot_units import clamp_to_bounds
from documents.timeline_domain import mirror_mode, mirrored_positions
from documents.timeline_storage import (
    load_snapshot_reference,
    make_snapshot_reference,
    save_timeline,
)
from documents.timeline_storage import (
    load_timeline as load_timeline_document,
)
from PySide6.QtCore import QTimer, Signal
from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QMessageBox,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)
from widgets.servo_layout import LEG_SERVOS, SERVO_NAMES

from tabs.controller.playback import ControllerPlayback

TAB_TITLE = "Motion Recorder"
TAB_ORDER = 36
STATE_TOPIC = "/saprog/joint_states"
STATE_TIMEOUT = 2.0


class MotionRecorderTab(QWidget):
    """Record feedback for one leg and export timeline-compatible snapshots."""

    pose_edit_requested = Signal(object, object)

    def __init__(
        self, ros_manager=None, parent=None, *, snapshot_directory=None,
        sequence_directory=None, clock=time.monotonic,
    ):
        super().__init__(parent)
        workspace = Path(__file__).resolve().parents[3]
        self.manager = ros_manager
        self.snapshot_directory = Path(
            snapshot_directory or workspace / "snapshots"
        ).expanduser().resolve()
        self.sequence_directory = Path(
            sequence_directory or workspace / "sequences"
        ).expanduser().resolve()
        self.clock = clock
        self.state_token = None
        self.positions = {}
        self.position_updated_at = {}
        self.torque_states = {}
        self.pending_torque = {}
        self.torque_failures = set()
        self.recorded_joints = set()
        self.frames = []
        self.timeline_entries = []
        self.timeline_path = None
        self.timeline_options = {"loop": True, "on_end": "on"}
        self.base_loop_locked = False
        self.recording = False
        self.overlay_recording = False
        self.overlay_samples = {}
        self.copied_leg = None
        self.playing_row = None
        self._preview_row = None
        self.overlay_last_index = None
        self.last_capture_at = None
        self.playback = ControllerPlayback(
            self.manager, self.snapshot_directory, self, clock=clock,
        )
        self.playback.status_changed.connect(self._playback_status)
        self.playback.step_ready.connect(self._overlay_step_ready)
        self.playback.step_changed.connect(self._playback_step_changed)
        self._playback_temp_dir = None

        layout = QVBoxLayout(self)
        title = QLabel("Motion Recorder")
        title.setObjectName("title")
        layout.addWidget(title)
        description = QLabel(
            "Record one leg to set the timeline length. Play it in a loop, then record "
            "other legs over one complete loop to add them without extending the timeline."
        )
        description.setWordWrap(True)
        layout.addWidget(description)

        form = QFormLayout()
        self.leg_picker = QComboBox()
        for leg, joints in LEG_SERVOS:
            self.leg_picker.addItem(leg, joints)
        self.leg_picker.currentIndexChanged.connect(self._selection_changed)
        form.addRow("Leg to record", self.leg_picker)
        self.sample_interval = QDoubleSpinBox()
        self.sample_interval.setRange(0.05, 5.0)
        self.sample_interval.setDecimals(2)
        self.sample_interval.setSingleStep(0.05)
        self.sample_interval.setValue(0.25)
        self.sample_interval.setSuffix(" s")
        self.sample_interval.setAccessibleName("Recording sample interval")
        form.addRow("Sample interval", self.sample_interval)
        self.speed = QSpinBox()
        self.speed.setRange(1, 1023)
        self.speed.setValue(512)
        self.speed.setAccessibleName("Timeline speed")
        form.addRow("Timeline speed", self.speed)
        self.strength = QSpinBox()
        self.strength.setRange(1, 1023)
        self.strength.setValue(512)
        self.strength.setAccessibleName("Timeline torque strength")
        form.addRow("Timeline torque strength", self.strength)
        layout.addLayout(form)

        torque_controls = QHBoxLayout()
        self.release_button = QPushButton("Torque off for selected leg")
        self.release_button.clicked.connect(lambda: self.set_leg_torque(False))
        torque_controls.addWidget(self.release_button)
        self.enable_button = QPushButton("Torque on for selected leg")
        self.enable_button.clicked.connect(lambda: self.set_leg_torque(True))
        torque_controls.addWidget(self.enable_button)
        layout.addLayout(torque_controls)

        self.pose_status = QLabel("Waiting for joint feedback…")
        self.pose_status.setWordWrap(True)
        layout.addWidget(self.pose_status)

        pose_source_controls = QHBoxLayout()
        self.new_timeline_button = QPushButton("New timeline")
        self.new_timeline_button.clicked.connect(self.new_timeline)
        pose_source_controls.addWidget(self.new_timeline_button)
        self.load_timeline_button = QPushButton("Load timeline…")
        self.load_timeline_button.clicked.connect(
            lambda _checked=False: self.load_timeline()
        )
        pose_source_controls.addWidget(self.load_timeline_button)
        self.import_pose_button = QPushButton("Add pose from snapshot…")
        self.import_pose_button.clicked.connect(
            lambda _checked=False: self.add_snapshot_pose()
        )
        pose_source_controls.addWidget(self.import_pose_button)
        pose_source_controls.addStretch(1)
        layout.addLayout(pose_source_controls)

        capture_controls = QHBoxLayout()
        self.capture_button = QPushButton("Capture current pose")
        self.capture_button.clicked.connect(self.capture_pose)
        capture_controls.addWidget(self.capture_button)
        self.play_step_button = QPushButton("Play selected step")
        self.play_step_button.clicked.connect(self.play_selected_step)
        capture_controls.addWidget(self.play_step_button)
        self.edit_pose_button = QPushButton("Edit selected pose…")
        self.edit_pose_button.clicked.connect(self.edit_selected_pose)
        capture_controls.addWidget(self.edit_pose_button)
        self.record_button = QPushButton("Record")
        self.record_button.clicked.connect(self.start_recording)
        capture_controls.addWidget(self.record_button)
        self.stop_button = QPushButton("Stop recording")
        self.stop_button.clicked.connect(self.stop_recording)
        self.stop_button.setEnabled(False)
        capture_controls.addWidget(self.stop_button)
        layout.addLayout(capture_controls)

        self.pose_list = QTableWidget(0, len(LEG_SERVOS) + 1)
        self.pose_list.setHorizontalHeaderLabels(["Pose", *(leg for leg, _ in LEG_SERVOS)])
        self.pose_list.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.pose_list.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeMode.ResizeToContents,
        )
        self.pose_list.verticalHeader().setVisible(False)
        self.pose_list.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        self.pose_list.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        self.pose_list.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        self.pose_list.setWordWrap(True)
        self.pose_list.setAccessibleName("Recorded poses")
        self.pose_list.currentCellChanged.connect(lambda *_args: self._update_controls())
        self.pose_list.cellDoubleClicked.connect(self._edit_pose_at)
        layout.addWidget(self.pose_list, 1)

        copy_controls = QHBoxLayout()
        self.copy_leg_button = QPushButton("Copy selected leg")
        self.copy_leg_button.clicked.connect(self.copy_leg)
        copy_controls.addWidget(self.copy_leg_button)
        self.paste_offset = QSpinBox()
        self.paste_offset.setRange(-100000, 100000)
        self.paste_offset.setAccessibleName("Paste offset in poses")
        self.paste_offset.setToolTip("Positive values shift copied poses later, wrapping around the timeline.")
        copy_controls.addWidget(QLabel("Pose offset"))
        copy_controls.addWidget(self.paste_offset)
        self.paste_leg_button = QPushButton("Paste to selected leg")
        self.paste_leg_button.clicked.connect(self.paste_leg)
        copy_controls.addWidget(self.paste_leg_button)
        copy_controls.addStretch(1)
        layout.addLayout(copy_controls)

        pose_controls = QHBoxLayout()
        self.remove_leg_data_button = QPushButton("Remove selected leg data")
        self.remove_leg_data_button.clicked.connect(self.remove_selected_leg_data)
        pose_controls.addWidget(self.remove_leg_data_button)
        self.remove_pose_button = QPushButton("Delete entire selected step")
        self.remove_pose_button.clicked.connect(self.remove_selected_pose)
        pose_controls.addWidget(self.remove_pose_button)
        self.play_button = QPushButton("Play recorded motion (loop)")
        self.play_button.clicked.connect(self.play_recorded_motion)
        pose_controls.addWidget(self.play_button)
        self.stop_playback_button = QPushButton("Stop playback")
        self.stop_playback_button.clicked.connect(self.stop_playback)
        pose_controls.addWidget(self.stop_playback_button)
        self.smooth_playback = QCheckBox("Smooth timed playback")
        self.smooth_playback.setToolTip("Adjust servo speeds to pose durations, up to saved speeds.")
        pose_controls.addWidget(self.smooth_playback)
        layout.addLayout(pose_controls)
        save_controls = QHBoxLayout()
        self.save_button = QPushButton("Save timeline")
        self.save_button.clicked.connect(lambda _checked=False: self.save_timeline())
        save_controls.addWidget(self.save_button)
        self.save_as_button = QPushButton("Save as…")
        self.save_as_button.clicked.connect(
            lambda _checked=False: self.save_timeline_as()
        )
        save_controls.addWidget(self.save_as_button)
        layout.addLayout(save_controls)
        self.status = QLabel(
            "Connect to the robot to begin. Torque remains in its current state after recording."
        )
        self.status.setWordWrap(True)
        layout.addWidget(self.status)

        self.record_timer = QTimer(self)
        self.record_timer.setInterval(50)
        self.record_timer.timeout.connect(self._record_tick)
        self.stop_playback_button.setEnabled(False)
        if self.manager is not None:
            self.manager.message_received.connect(self.receive)
            self.manager.command_result.connect(self.command_result)
            try:
                self.state_token = self.manager.request_topic(
                    STATE_TOPIC, "sensor_msgs/msg/JointState",
                    package="saprog_dynamixel", executable="dynamixel_control_node",
                )
                manager_ref = [self.manager]
                self.manager.destroyed.connect(
                    lambda _obj=None: manager_ref.__setitem__(0, None)
                )
                self.destroyed.connect(
                    lambda _obj=None: manager_ref[0].release(self.state_token)
                    if manager_ref[0] is not None and self.state_token else None
                )
            except (AttributeError, RuntimeError, ValueError) as error:
                self.status.setText(f"Could not subscribe to joint feedback: {error}")
                self.manager = None
        else:
            self.status.setText("ROS manager is unavailable.")
        self._selection_changed()

    def showEvent(self, event):
        super().showEvent(event)
        self.playback.limits.refresh()

    def _selected_joints(self):
        """Return the configured servos for the selected leg."""
        return tuple(self.leg_picker.currentData() or ())

    def _selection_changed(self, _index=None):
        """Refresh pose controls when the selected leg changes."""
        self._update_pose_status()
        self._update_controls()

    def receive(self, topic, message):
        """Cache valid servo positions and torque feedback."""
        if topic != STATE_TOPIC or not isinstance(message, dict):
            return
        names = message.get("name", ())
        positions = message.get("position", ())
        efforts = message.get("effort", ())
        updated = False
        for index, joint in enumerate(names):
            if not isinstance(joint, str) or joint not in SERVO_NAMES:
                continue
            if index < len(positions):
                try:
                    value = float(positions[index])
                except (TypeError, ValueError, OverflowError):
                    continue
                if math.isfinite(value):
                    self.positions[joint] = value
                    self.position_updated_at[joint] = self.clock()
                    updated = True
            if index < len(efforts):
                try:
                    self.torque_states[joint] = bool(float(efforts[index]))
                except (TypeError, ValueError, OverflowError):
                    pass
        if updated:
            self._update_pose_status()
            self._update_controls()

    def _state_is_fresh(self, joints=None):
        """Check that every requested joint has reported a recent position."""
        now = self.clock()
        joints = self._selected_joints() if joints is None else joints
        return all(
            joint in self.positions
            and now - self.position_updated_at.get(joint, float("-inf")) <= STATE_TIMEOUT
            for joint in joints
        )

    def _capture_joints(self):
        """Return recorded joints plus the currently selected leg."""
        return self.recorded_joints | set(self._selected_joints())

    def _playback_joints(self):
        """Return joints currently controlled by this tab's recorded playback."""
        if self.playback.owner is not self:
            return set()
        return {
            joint
            for step in self.playback.steps
            if isinstance(step.get("snapshot"), dict)
            for joint in step["snapshot"]["positions"]
        }

    def _playback_status(self, message):
        """Surface playback results and release its temporary snapshots when stopped."""
        self.status.setText(message)
        if self.playback.owner is not self:
            self._highlight_playback_step(None)
            self._cleanup_playback_files()
            if self.overlay_recording:
                self._discard_overlay(f"Overlay recording stopped with playback: {message}")
        self._update_controls()

    def _cleanup_playback_files(self):
        """Remove temporary preview poses after playback no longer needs them."""
        self._preview_row = None
        if self._playback_temp_dir is not None:
            self._playback_temp_dir.cleanup()
            self._playback_temp_dir = None

    def _update_pose_status(self):
        """Show the latest selected-leg positions and torque states."""
        joints = self._selected_joints()
        values = []
        for joint in joints:
            position = self.positions.get(joint)
            text = f"{joint}: {position:.3f}" if position is not None else f"{joint}: —"
            torque = self.torque_states.get(joint)
            if torque is not None:
                text += f" (torque {'on' if torque else 'off'})"
            values.append(text)
        self.pose_status.setText("    ".join(values) or "No leg selected")

    def _update_controls(self):
        """Enable only actions supported by the current connection and state."""
        connected = self.manager is not None
        pending = bool(self.pending_torque)
        playback_joints = self._playback_joints()
        selected_conflicts = bool(playback_joints & set(self._selected_joints()))
        self.release_button.setEnabled(
            connected and not pending and not self.recording and not selected_conflicts
        )
        self.enable_button.setEnabled(
            connected and not pending and not self.recording and not selected_conflicts
        )
        self.leg_picker.setEnabled(not self.recording)
        self.sample_interval.setEnabled(not self.recording)
        self.capture_button.setEnabled(
            not self.recording and not self.base_loop_locked and self.playback.owner is not self
            and self._state_is_fresh(self._capture_joints())
        )
        self.import_pose_button.setEnabled(
            not self.recording and not self.base_loop_locked and self.playback.owner is not self
        )
        joints_off = all(
            self.torque_states.get(joint) is False for joint in self._selected_joints()
        )
        playing = self.playback.owner is self
        self.copy_leg_button.setEnabled(bool(self.frames) and not self.recording and not playing)
        self.paste_leg_button.setEnabled(
            bool(self.frames) and self.copied_leg is not None and not self.recording and not playing
        )
        self.paste_offset.setEnabled(not self.recording and not playing)
        selected_row = self.pose_list.currentRow()
        selected_entry = (
            self.timeline_entries[selected_row]
            if 0 <= selected_row < len(self.timeline_entries) else None
        )
        selected_pose = isinstance(selected_entry, dict) and "positions" in selected_entry
        selected_leg_data = selected_pose and any(
            joint in selected_entry["positions"] for joint in self._selected_joints()
        )
        self.record_button.setEnabled(
            connected and not self.recording and not pending
            and self._state_is_fresh(self._capture_joints()) and joints_off
            and (playing or not self.base_loop_locked)
            and (not playing or self._preview_row is None)
        )
        self.stop_button.setEnabled(self.recording)
        self.play_step_button.setEnabled(
            connected and selected_entry is not None and not self.recording and not playing
        )
        self.edit_pose_button.setEnabled(
            selected_pose and not self.recording and not playing
        )
        self.remove_pose_button.setEnabled(
            selected_entry is not None and not self.recording and not playing
        )
        self.remove_leg_data_button.setEnabled(
            selected_leg_data and not self.recording and not playing
        )
        self.save_button.setEnabled(bool(self.timeline_entries) and not self.recording)
        self.save_as_button.setEnabled(bool(self.timeline_entries) and not self.recording)
        self.play_button.setEnabled(
            connected and bool(self.timeline_entries) and not self.recording and not playing
        )
        self.stop_playback_button.setEnabled(playing)
        self.new_timeline_button.setEnabled(not self.recording and not playing)
        self.load_timeline_button.setEnabled(not self.recording and not playing)
        self.save_button.setText("Save timeline" if self.timeline_path else "Save as timeline…")
        if self.base_loop_locked:
            self.record_button.setText(
                "Record leg over one loop" if playing else "Play loop to record another leg"
            )
        else:
            self.record_button.setText("Record")

    def set_leg_torque(self, enabled):
        """Request torque state changes for every servo in the selected leg."""
        if self.manager is None:
            self.status.setText("ROS manager is unavailable.")
            return False
        if self.recording or self.pending_torque:
            return False
        joints = self._selected_joints()
        if not joints:
            self.status.setText("Select a leg first.")
            return False
        if self._playback_joints() & set(joints):
            self.status.setText("Stop recorded playback before changing torque on the playing leg.")
            return False
        self.torque_failures.clear()
        try:
            for joint in joints:
                command_id = self.manager.set_servo_torque(joint, enabled)
                self.pending_torque[command_id] = (joint, enabled)
        except (RuntimeError, ValueError, TypeError, AttributeError) as error:
            self.status.setText(f"Could not request torque change for the selected leg: {error}")
            self._update_controls()
            return False
        state = "on" if enabled else "off"
        self.status.setText(f"Requesting torque {state} for {len(joints)} servos…")
        self._update_controls()
        return True

    def command_result(self, command_id, success, message):
        """Apply acknowledged torque states and report any failed servo request."""
        pending = self.pending_torque.pop(command_id, None)
        if pending is None:
            return
        joint, enabled = pending
        if success:
            self.torque_states[joint] = enabled
        else:
            self.torque_failures.add(joint)
        if not self.pending_torque:
            if self.torque_failures:
                failed = ", ".join(sorted(self.torque_failures))
                self.status.setText(f"Torque change failed for {failed}: {message}")
            else:
                state = "on" if enabled else "off"
                self.status.setText(f"Torque {state} acknowledged for the selected leg.")
            self._update_pose_status()
            self._update_controls()

    def _capture_current_pose(self, captured_at=None, *, recorded=False):
        """Append a composed pose from the selected and previously recorded legs."""
        joints = self._capture_joints()
        if not self._state_is_fresh(joints):
            return False
        pose = {joint: self.positions[joint] for joint in sorted(joints)}
        return self._append_pose(pose, captured_at, recorded=recorded)

    def _append_pose(self, pose, captured_at=None, *, recorded=False, label=None):
        """Append a pose and add its summary to the keyframe list."""
        timestamp = self.clock() if captured_at is None else captured_at
        pose = self._clamp_pose(pose)
        joints = tuple(pose)
        self.recorded_joints.update(joints)
        frame = {
            "positions": dict(pose), "captured_at": timestamp, "recorded": recorded,
            "label": label,
        }
        self.frames.append(frame)
        self.timeline_entries.append(frame)
        row = self.pose_list.rowCount()
        self.pose_list.insertRow(row)
        self._set_pose_row(row, frame, len(self.frames))
        self.pose_list.selectRow(row)
        self._update_controls()
        return True

    def _clamp_pose(self, pose):
        bounds = self.playback.bounds or {}
        return {
            joint: clamp_to_bounds(value, bounds[joint]) if joint in bounds else value
            for joint, value in pose.items()
        }

    def _set_pose_row(self, row, frame, pose_number):
        label = f" — {frame['label']}" if frame.get("label") else ""
        self.pose_list.setItem(row, 0, QTableWidgetItem(f"Pose {pose_number}{label}"))
        positions = frame["positions"]
        for column, (_leg, joints) in enumerate(LEG_SERVOS, start=1):
            values = [
                f"{joint.removeprefix('servo_')}: {positions[joint]:.3f}"
                for joint in joints if joint in positions
            ]
            self.pose_list.setItem(
                row, column, QTableWidgetItem("\n".join(values) if values else "—"),
            )
        self.pose_list.setRowHeight(row, 58)

    def _set_wait_row(self, row, step):
        self.pose_list.setItem(
            row, 0, QTableWidgetItem(f"Wait — {step['seconds']:g} s"),
        )
        for column in range(1, len(LEG_SERVOS) + 1):
            self.pose_list.setItem(row, column, QTableWidgetItem("—"))
        self.pose_list.setRowHeight(row, 32)

    def add_snapshot_pose(self, path=None):
        """Add the selected-leg targets from a saved snapshot as a keyframe."""
        if self.recording or self.base_loop_locked or self.playback.owner is self:
            return False
        if path is None:
            path, _filter = QFileDialog.getOpenFileName(
                self, "Add pose from snapshot", str(self.snapshot_directory),
                "Snapshot JSON (*.json)",
            )
        if not path:
            return False
        try:
            snapshot = load_snapshot_reference(
                str(path), self.snapshot_directory, bounds=self.playback.bounds,
            )
            joints = self._selected_joints()
            missing = [joint for joint in joints if joint not in snapshot["positions"]]
            if missing:
                raise ValueError(
                    "Snapshot is missing selected-leg servos: " + ", ".join(missing)
                )
            self._append_pose(
                {joint: snapshot["positions"][joint] for joint in joints},
                label=snapshot["name"],
            )
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.status.setText(f"Could not add snapshot pose: {error}")
            return False
        self.status.setText(f"Added snapshot pose: {snapshot['name']}.")
        return True

    def _edit_pose_at(self, row, column):
        """Open the double-clicked pose through the shared edit action."""
        self.pose_list.setCurrentCell(row, column)
        self.edit_selected_pose()

    def edit_selected_pose(self):
        """Open the selected pose in the workspace's Static Animator."""
        if self.recording or self.playback.owner is self:
            return False
        row = self.pose_list.currentRow()
        if not 0 <= row < len(self.timeline_entries):
            return False
        frame = self.timeline_entries[row]
        if "positions" not in frame:
            self.status.setText("Select a pose row to edit, not a wait step.")
            return False

        self.pose_edit_requested.emit(self, frame)
        return True

    def apply_pose_edit(self, frame, positions):
        """Apply targets to the original step even if the selection has changed."""
        if self.recording or self.playback.owner is self:
            raise ValueError("Stop recording or playback before applying pose changes")
        row = next((i for i, entry in enumerate(self.timeline_entries) if entry is frame), None)
        if row is None:
            raise ValueError("This pose is no longer in the Motion Recorder timeline")
        frame["positions"] = self._clamp_pose(positions)
        frame["positions_changed"] = True
        self.recorded_joints.update(frame["positions"])
        self._refresh_pose_list()
        self.pose_list.selectRow(row)
        self.status.setText(f"Updated pose {row + 1}.")
        self._update_controls()
        return True

    def _confirm_timeline_replacement(self):
        if not self.timeline_entries:
            return True
        return QMessageBox.question(
            self,
            "Replace current timeline?",
            "Discard the current timeline from the Motion Recorder?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        ) == QMessageBox.StandardButton.Yes

    def new_timeline(self):
        """Start a blank timeline after confirming replacement of current content."""
        if self.recording or self.playback.owner is self:
            self.status.setText("Stop recording or playback before starting a new timeline.")
            return False
        if not self._confirm_timeline_replacement():
            return False
        self.frames.clear()
        self.timeline_entries.clear()
        self.recorded_joints.clear()
        self.base_loop_locked = False
        self.timeline_path = None
        self.timeline_options = {"loop": True, "on_end": "on"}
        self.smooth_playback.setChecked(False)
        self._refresh_pose_list()
        self._update_controls()
        self.status.setText("New timeline. Capture or import poses to add steps.")
        return True

    def load_timeline(self, path=None):
        """Load a timeline while retaining its waits and per-step settings."""
        if self.recording or self.playback.owner is self:
            self.status.setText("Stop recording or playback before loading a timeline.")
            return False
        if path is None:
            path, _filter = QFileDialog.getOpenFileName(
                self, "Load timeline", str(self.sequence_directory),
                "Timeline JSON (*.json)",
            )
        if not path:
            return False
        if not self._confirm_timeline_replacement():
            return False
        try:
            document = load_timeline_document(
                path, self.snapshot_directory, bounds=self.playback.bounds,
            )
            entries = []
            frames = []
            for step in document["steps"]:
                if step.get("kind") == "wait":
                    entries.append(copy.deepcopy(step))
                    continue
                snapshot = load_snapshot_reference(
                    step["snapshot"], self.snapshot_directory, bounds=self.playback.bounds,
                )
                positions = mirrored_positions(snapshot["positions"], mirror_mode(step))
                frame = {
                    "positions": self._clamp_pose(positions),
                    "captured_at": self.clock(),
                    "recorded": False,
                    "label": snapshot["name"],
                    "timeline_step": copy.deepcopy(step),
                }
                entries.append(frame)
                frames.append(frame)
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.status.setText(f"Could not load timeline: {error}")
            return False

        self.timeline_entries = entries
        self.frames = frames
        self.recorded_joints = {
            joint for frame in frames for joint in frame["positions"]
        }
        self.base_loop_locked = False
        self.timeline_path = Path(path).expanduser().resolve()
        self.timeline_options = {
            "loop": document.get("loop", False),
            "on_end": document.get("on_end", "on"),
        }
        self.smooth_playback.setChecked(document.get("smooth_playback", False))
        self._refresh_pose_list()
        if self.timeline_entries:
            self.pose_list.selectRow(0)
        self._update_controls()
        wait_count = sum(entry.get("kind") == "wait" for entry in entries)
        self.status.setText(
            f"Loaded {len(frames)} poses and {wait_count} wait steps from {self.timeline_path.name}."
        )
        return True

    def remove_selected_leg_data(self):
        """Remove the selected leg's targets from a pose without deleting its step."""
        if self.recording or self.playback.owner is self:
            return False
        row = self.pose_list.currentRow()
        if not 0 <= row < len(self.timeline_entries):
            return False
        frame = self.timeline_entries[row]
        if "positions" not in frame:
            self.status.setText("Select a pose row to remove leg data.")
            return False
        joints = set(self._selected_joints())
        positions = frame["positions"]
        removed = joints & positions.keys()
        if not removed:
            self.status.setText("The selected leg has no data in this pose.")
            return False
        if not positions.keys() - joints:
            self.status.setText("Remove the entire step to delete its last leg.")
            return False
        frame["positions"] = {
            joint: value for joint, value in positions.items() if joint not in joints
        }
        frame["positions_changed"] = True
        self.recorded_joints = {
            joint for pose in self.frames for joint in pose["positions"]
        }
        self._refresh_pose_list()
        self.pose_list.selectRow(row)
        self._update_controls()
        self.status.setText(f"Removed the selected leg's data from pose {row + 1}.")
        return True

    def remove_selected_pose(self):
        """Remove the selected pose or wait step from the timeline."""
        if self.recording:
            self.status.setText("Stop recording before removing a pose.")
            return False
        if self.playback.owner is self:
            self.status.setText("Stop playback before removing a pose.")
            return False
        index = self.pose_list.currentRow()
        if not 0 <= index < len(self.timeline_entries):
            self.status.setText("Select a timeline step to remove.")
            return False
        entry = self.timeline_entries.pop(index)
        if "positions" in entry:
            self.frames = [frame for frame in self.frames if frame is not entry]
        else:
            self.status.setText(f"Removed wait step {index + 1}.")
        if "positions" in entry:
            self.status.setText(f"Removed pose {index + 1}.")
        self.recorded_joints = {
            joint for frame in self.frames for joint in frame["positions"]
        }
        self._refresh_pose_list()
        self._update_controls()
        return True

    def _refresh_pose_list(self):
        """Rebuild pose and wait rows after timeline edits."""
        self.pose_list.clearContents()
        self.pose_list.setRowCount(len(self.timeline_entries))
        pose_number = 0
        for row, entry in enumerate(self.timeline_entries):
            if "positions" in entry:
                pose_number += 1
                self._set_pose_row(row, entry, pose_number)
            else:
                self._set_wait_row(row, entry)

    def _playback_step_changed(self, index):
        """Map a single-step preview back to its original timeline row."""
        self._highlight_playback_step(self._preview_row if self._preview_row is not None else index)

    def _highlight_playback_step(self, index):
        """Mark the active pose or wait without changing the user's selection."""
        for row in (self.playing_row, index):
            if row is None or not 0 <= row < self.pose_list.rowCount():
                continue
            active = row == index
            for column in range(self.pose_list.columnCount()):
                item = self.pose_list.item(row, column)
                if item is not None:
                    item.setBackground(QBrush(QColor("#385e78")) if active else QBrush())
            label = self.pose_list.item(row, 0)
            if label is not None:
                label.setText(("▶ " if active else "") + label.text().removeprefix("▶ "))
        self.playing_row = index

    def copy_leg(self):
        """Copy every pose of the selected leg, in timeline order."""
        if self.recording or self.playback.owner is self or not self.frames:
            return False
        joints = self._selected_joints()
        if any(any(joint not in frame["positions"] for joint in joints) for frame in self.frames):
            self.status.setText("The selected leg must have all three targets in every pose to copy it.")
            return False
        self.copied_leg = [tuple(frame["positions"][joint] for joint in joints) for frame in self.frames]
        self.status.setText(f"Copied {self.leg_picker.currentText()} from {len(self.frames)} poses.")
        self._update_controls()
        return True

    def paste_leg(self):
        """Paste copied targets into the selected leg, shifting pose indices cyclically."""
        if self.recording or self.playback.owner is self or self.copied_leg is None:
            return False
        if len(self.copied_leg) != len(self.frames) or not self.frames:
            self.status.setText("The number of poses changed since copying; copy the leg again.")
            return False
        joints = self._selected_joints()
        offset = self.paste_offset.value()
        clamped = False
        for index, values in enumerate(self.copied_leg):
            frame = self.frames[(index + offset) % len(self.frames)]
            targets = dict(zip(joints, values))
            bounded = self._clamp_pose(targets)
            clamped = clamped or bounded != targets
            frame["positions"].update(bounded)
            frame["positions_changed"] = True
        self.recorded_joints.update(joints)
        self._refresh_pose_list()
        self._update_controls()
        self.status.setText(
            f"Pasted into {self.leg_picker.currentText()} with offset {offset} poses."
            + (" Some targets were clamped to servo limits." if clamped else "")
        )
        return True

    def capture_pose(self):
        """Capture a manually positioned leg as a timeline pose."""
        if self.recording or self.playback.owner is self:
            self.status.setText("Stop recording or playback before adding a pose.")
            return False
        if self.base_loop_locked:
            self.status.setText("The first recording fixed the timeline length; use a loop overlay.")
            return False
        if not self._capture_current_pose():
            self.status.setText("Cannot capture: waiting for fresh feedback from every servo in the leg.")
            return False
        self.status.setText(f"Captured pose {len(self.frames)}.")
        return True

    def start_recording(self):
        """Record the base loop or overlay another leg on an existing loop."""
        if self.base_loop_locked or self.playback.owner is self:
            return self.start_overlay_recording()
        if self.recording:
            return False
        if self.playback.owner is self:
            self.status.setText("Stop playback before recording the base leg.")
            return False
        if not self._state_is_fresh(self._capture_joints()):
            self.status.setText("Cannot record: waiting for fresh feedback from every servo in the leg.")
            return False
        if any(self.torque_states.get(joint) is not False for joint in self._selected_joints()):
            self.status.setText("Request torque off for the selected leg and wait for acknowledgement.")
            return False
        now = self.clock()
        if not self._capture_current_pose(now, recorded=True):
            return False
        self.recording = True
        self.last_capture_at = now
        self.record_timer.start()
        self.status.setText("Recording. Move the released leg by hand; torque will remain off.")
        self._update_controls()
        return True

    def start_overlay_recording(self):
        """Capture a different leg once per step of the playing base loop."""
        if self.recording:
            return False
        if self._preview_row is not None:
            self.status.setText("Play the full recorded loop before recording another leg.")
            return False
        if self.playback.owner is not self:
            self.status.setText("Play the recorded loop before recording another leg over it.")
            return False
        joints = self._selected_joints()
        if self._playback_joints() & set(joints):
            self.status.setText("Select a leg that is not part of the playing loop.")
            return False
        if not self._state_is_fresh(joints):
            self.status.setText("Cannot record: waiting for fresh feedback from every servo in the leg.")
            return False
        if any(self.torque_states.get(joint) is not False for joint in joints):
            self.status.setText("Request torque off for the selected leg and wait for acknowledgement.")
            return False
        self.overlay_samples = {}
        self.overlay_last_index = None
        self.overlay_recording = True
        self.recording = True
        if self.playback.phase in ("wait", "arrival"):
            self._capture_overlay_step(self.playback.index)
        if not self.overlay_recording:
            return True
        self.status.setText(
            "Recording this leg over one complete loop. The timeline length will not change."
        )
        self._update_controls()
        return True

    def _record_tick(self):
        """Capture the latest pose at the selected sample interval."""
        if not self.recording:
            return
        if self.overlay_recording:
            return
        now = self.clock()
        if not self._state_is_fresh(self._capture_joints()):
            self.stop_recording()
            self.status.setText(
                "Recording stopped because feedback became stale for a recorded leg."
            )
            return
        if now - self.last_capture_at < self.sample_interval.value():
            return
        if self._capture_current_pose(now, recorded=True):
            self.last_capture_at = now

    def _overlay_step_ready(self, index):
        """Capture each loop pose after its playback command is acknowledged."""
        if self.overlay_recording:
            self._capture_overlay_step(index)

    def _capture_overlay_step(self, index):
        """Store selected-leg positions for the given fixed timeline step."""
        if self.playback.owner is not self:
            self._discard_overlay("Overlay recording stopped because playback ended.")
            return
        if index == self.overlay_last_index or not 0 <= index < len(self.timeline_entries):
            return
        frame = self.timeline_entries[index]
        if "positions" not in frame:
            return
        joints = self._selected_joints()
        if not self._state_is_fresh(joints):
            self._discard_overlay(
                "Overlay recording stopped because feedback for the selected leg became stale."
            )
            return
        self.overlay_samples[index] = {
            joint: clamp_to_bounds(self.positions[joint], self.playback.bounds[joint])
            if self.playback.bounds and joint in self.playback.bounds else self.positions[joint]
            for joint in joints
        }
        self.overlay_last_index = index
        if len(self.overlay_samples) == len(self.frames):
            self._finish_overlay_recording()

    def _finish_overlay_recording(self):
        """Merge a full loop of leg samples without adding timeline steps."""
        self.record_timer.stop()
        self.recording = False
        self.overlay_recording = False
        for index, pose in self.overlay_samples.items():
            frame = self.timeline_entries[index]
            frame["positions"].update(pose)
            frame["positions_changed"] = True
        self.recorded_joints.update(self._selected_joints())
        self.overlay_samples = {}
        self.overlay_last_index = None
        self._refresh_pose_list()
        if self.playback.owner is self:
            self.playback.stop("Leg overlay captured; playback stopped.")
        else:
            self._cleanup_playback_files()
        self.status.setText(
            f"Recorded one loop over {len(self.frames)} timeline poses. "
            "Play the updated loop to layer another leg."
        )
        self._update_controls()

    def _discard_overlay(self, message):
        """Stop an incomplete leg overlay without changing the base timeline."""
        self.record_timer.stop()
        self.recording = False
        self.overlay_recording = False
        self.overlay_samples = {}
        self.overlay_last_index = None
        self.status.setText(message)
        self._update_controls()

    def stop_recording(self):
        """Stop timed capture and retain the final available pose."""
        if not self.recording:
            return False
        if self.overlay_recording:
            self._discard_overlay(
                "Overlay recording stopped before a full loop; no poses were added."
            )
            return True
        self.record_timer.stop()
        self.recording = False
        self._capture_current_pose(recorded=True)
        self.base_loop_locked = True
        self.status.setText(
            f"Base timeline set to {len(self.frames)} poses. Play it to record other legs over one loop."
        )
        self._update_controls()
        return True

    def play_recorded_motion(self):
        """Preview the current timeline so another leg can be recorded alongside it."""
        return self._play_motion()

    def play_selected_step(self):
        """Play just the selected timeline step once using its saved settings."""
        row = self.pose_list.currentRow()
        if not 0 <= row < len(self.timeline_entries):
            self.status.setText("Select a timeline step to play.")
            return False
        return self._play_motion(row)

    def _play_motion(self, row=None):
        """Run the timeline or one step through the shared playback lifecycle."""
        if self.manager is None:
            self.status.setText("ROS manager is unavailable.")
            return False
        if self.playback.owner is self:
            self.status.setText("Recorded motion is already playing.")
            return False
        if self.recording or not self.timeline_entries:
            self.status.setText("Stop recording and add a timeline step before playback.")
            return False
        preview = tempfile.TemporaryDirectory(prefix="saprog_motion_preview_")
        try:
            steps = self._write_steps(self.timeline_entries, Path(preview.name))
            if row is not None:
                steps = [steps[row]]
            self._preview_row = row
            self._playback_temp_dir = preview
            self.playback.start_steps(
                self, steps, repeat=row is None and self.timeline_options["loop"],
                smooth=self.smooth_playback.isChecked(),
            )
        except (OSError, RuntimeError, ValueError, TypeError, KeyError) as error:
            preview.cleanup()
            self._cleanup_playback_files()
            self.status.setText(f"Could not play recorded motion: {error}")
            self._update_controls()
            return False
        self.status.setText(
            f"Playing selected step {row + 1} once." if row is not None else
            "Playing timeline in a loop. Select another leg and record it on top."
            if self.timeline_options["loop"] else "Playing timeline once."
        )
        self._update_controls()
        return True

    def stop_playback(self):
        """Stop the recorded motion preview."""
        if self.playback.owner is not self:
            return False
        self.playback.stop("Recorded motion playback stopped.")
        return True

    def closeEvent(self, event):
        """Stop timers and release playback resources when this tab closes."""
        self.record_timer.stop()
        self.playback.close()
        self._cleanup_playback_files()
        super().closeEvent(event)

    def _write_steps(self, entries, pose_directory, reference_directory=None):
        """Write changed pose files while retaining loaded timeline step settings."""
        steps = []
        pose_number = 0
        for index, entry in enumerate(entries):
            if "positions" not in entry:
                steps.append(copy.deepcopy(entry))
                continue
            frame = entry
            source_step = frame.get("timeline_step")
            if source_step is not None and not frame.get("positions_changed"):
                steps.append(copy.deepcopy(source_step))
                continue

            pose_number += 1
            pose_directory.mkdir(parents=True, exist_ok=True)
            pose_name = f"pose_{pose_number:04d}"
            pose_path = pose_directory / f"{pose_name}.json"
            positions = frame["positions"]
            if source_step is not None:
                positions = mirrored_positions(positions, mirror_mode(source_step), inverse=True)
            write_json_atomic(pose_path, {
                "schema_version": 1,
                "name": pose_name,
                "positions": positions,
            })
            reference = (
                make_snapshot_reference(pose_path, reference_directory)
                if reference_directory is not None
                else str(pose_path.resolve())
            )
            if source_step is not None:
                step = copy.deepcopy(source_step)
                step["snapshot"] = reference
                if "overrides" in step:
                    step["overrides"] = {
                        joint: settings for joint, settings in step["overrides"].items()
                        if joint in positions
                    }
                steps.append(step)
            else:
                next_entry = entries[index + 1] if index + 1 < len(entries) else None
                seconds = self.sample_interval.value()
                if (
                    next_entry is not None and "positions" in next_entry
                    and frame["recorded"] and next_entry["recorded"]
                ):
                    seconds = next_entry["captured_at"] - frame["captured_at"]
                steps.append({
                    "snapshot": reference,
                    "motion": {"speed": self.speed.value(), "torque": self.strength.value()},
                    "duration": {"mode": "seconds", "seconds": max(0.05, seconds)},
                })
        return steps

    def save_timeline_as(self):
        """Choose a new path for the current timeline."""
        path, _filter = QFileDialog.getSaveFileName(
            self, "Save timeline as",
            str(self.sequence_directory / "recorded_motion.json"),
            "Timeline JSON (*.json)",
        )
        return self.save_timeline(path) if path else False

    def save_timeline(self, path=None):
        """Save the current timeline without discarding its step settings."""
        if self.recording:
            self.stop_recording()
        if not self.timeline_entries:
            self.status.setText("Add at least one timeline step before saving.")
            return False
        if path is None:
            path = self.timeline_path
            if path is None:
                path, _filter = QFileDialog.getSaveFileName(
                    self, "Save timeline",
                    str(self.sequence_directory / "recorded_motion.json"),
                    "Timeline JSON (*.json)",
                )
                if not path:
                    return False
        destination = Path(path).expanduser()
        if not destination.suffix:
            destination = destination.with_suffix(".json")
        destination = destination.resolve()
        folder_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", destination.stem).strip("._")
        if not folder_name:
            self.status.setText("Choose a timeline file name with letters or numbers.")
            return False
        unique_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        pose_directory = self.snapshot_directory / "recordings" / f"{folder_name}_{unique_id}"
        try:
            steps = self._write_steps(
                self.timeline_entries, pose_directory, self.snapshot_directory,
            )
            save_timeline(
                destination, steps,
                on_end=self.timeline_options["on_end"],
                loop=self.timeline_options["loop"],
                smooth_playback=self.smooth_playback.isChecked(),
                snapshot_resolver=lambda reference: load_snapshot_reference(
                    reference, self.snapshot_directory, bounds=self.playback.bounds,
                ),
            )
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.status.setText(f"Could not save recorded timeline: {error}")
            return False
        for entry, step in zip(self.timeline_entries, steps):
            if "positions" in entry:
                entry["timeline_step"] = copy.deepcopy(step)
                entry.pop("positions_changed", None)
            else:
                entry.clear()
                entry.update(copy.deepcopy(step))
        self.timeline_path = destination
        self._update_controls()
        self.status.setText(f"Saved {len(self.frames)} poses as {destination}.")
        return True


def create_tab(ros_manager=None):
    """Build the hardware-backed motion recorder."""
    return MotionRecorderTab(ros_manager=ros_manager)
