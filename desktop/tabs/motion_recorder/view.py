"""Widget construction, control availability and timeline presentation."""

from PySide6.QtGui import QBrush, QColor
from PySide6.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QPushButton,
    QSpinBox,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
)
from widgets.servo_layout import LEG_SERVOS


class RecorderView:
    """Widget construction, control availability and timeline presentation.

    The tab supplies shared session state and UI callbacks; this helper owns
    only this workflow and never substitutes for the QWidget.
    """

    def __init__(self, tab):
        self.tab = tab

    def build(self):
        """Create controls and connect them to the tab actions."""
        tab = self.tab
        layout = QVBoxLayout(tab)
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
        tab.leg_picker = QComboBox()
        for leg, joints in LEG_SERVOS:
            tab.leg_picker.addItem(leg, joints)
        tab.leg_picker.currentIndexChanged.connect(tab._selection_changed)
        form.addRow("Leg to record", tab.leg_picker)
        tab.sample_interval = QDoubleSpinBox()
        tab.sample_interval.setRange(0.05, 5.0)
        tab.sample_interval.setDecimals(2)
        tab.sample_interval.setSingleStep(0.05)
        tab.sample_interval.setValue(0.25)
        tab.sample_interval.setSuffix(" s")
        tab.sample_interval.setAccessibleName("Recording sample interval")
        form.addRow("Sample interval", tab.sample_interval)
        tab.speed = QSpinBox()
        tab.speed.setRange(1, 1023)
        tab.speed.setValue(512)
        tab.speed.setAccessibleName("Timeline speed")
        form.addRow("Timeline speed", tab.speed)
        tab.strength = QSpinBox()
        tab.strength.setRange(1, 1023)
        tab.strength.setValue(512)
        tab.strength.setAccessibleName("Timeline torque strength")
        form.addRow("Timeline torque strength", tab.strength)
        layout.addLayout(form)

        torque_controls = QHBoxLayout()
        tab.release_button = QPushButton("Torque off for selected leg")
        tab.release_button.clicked.connect(lambda: tab.set_leg_torque(False))
        torque_controls.addWidget(tab.release_button)
        tab.enable_button = QPushButton("Torque on for selected leg")
        tab.enable_button.clicked.connect(lambda: tab.set_leg_torque(True))
        torque_controls.addWidget(tab.enable_button)
        layout.addLayout(torque_controls)

        tab.pose_status = QLabel("Waiting for joint feedback…")
        tab.pose_status.setWordWrap(True)
        layout.addWidget(tab.pose_status)

        pose_source_controls = QHBoxLayout()
        tab.new_timeline_button = QPushButton("New timeline")
        tab.new_timeline_button.clicked.connect(tab.new_timeline)
        pose_source_controls.addWidget(tab.new_timeline_button)
        tab.load_timeline_button = QPushButton("Load timeline…")
        tab.load_timeline_button.clicked.connect(
            lambda _checked=False: tab.load_timeline()
        )
        pose_source_controls.addWidget(tab.load_timeline_button)
        tab.import_pose_button = QPushButton("Add pose from snapshot…")
        tab.import_pose_button.clicked.connect(
            lambda _checked=False: tab.add_snapshot_pose()
        )
        pose_source_controls.addWidget(tab.import_pose_button)
        pose_source_controls.addStretch(1)
        layout.addLayout(pose_source_controls)

        capture_controls = QHBoxLayout()
        tab.capture_button = QPushButton("Capture current pose")
        tab.capture_button.clicked.connect(tab.capture_pose)
        capture_controls.addWidget(tab.capture_button)
        tab.play_step_button = QPushButton("Play selected step")
        tab.play_step_button.clicked.connect(tab.play_selected_step)
        capture_controls.addWidget(tab.play_step_button)
        tab.edit_pose_button = QPushButton("Edit selected pose…")
        tab.edit_pose_button.clicked.connect(tab.edit_selected_pose)
        capture_controls.addWidget(tab.edit_pose_button)
        tab.record_button = QPushButton("Record")
        tab.record_button.clicked.connect(tab.start_recording)
        capture_controls.addWidget(tab.record_button)
        tab.stop_button = QPushButton("Stop recording")
        tab.stop_button.clicked.connect(tab.stop_recording)
        tab.stop_button.setEnabled(False)
        capture_controls.addWidget(tab.stop_button)
        layout.addLayout(capture_controls)

        tab.pose_list = QTableWidget(0, len(LEG_SERVOS) + 1)
        tab.pose_list.setHorizontalHeaderLabels(["Pose", *(leg for leg, _ in LEG_SERVOS)])
        tab.pose_list.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        tab.pose_list.horizontalHeader().setSectionResizeMode(
            0, QHeaderView.ResizeMode.ResizeToContents,
        )
        tab.pose_list.verticalHeader().setVisible(False)
        tab.pose_list.setSelectionBehavior(QAbstractItemView.SelectionBehavior.SelectRows)
        tab.pose_list.setSelectionMode(QAbstractItemView.SelectionMode.SingleSelection)
        tab.pose_list.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
        tab.pose_list.setWordWrap(True)
        tab.pose_list.setAccessibleName("Recorded poses")
        tab.pose_list.currentCellChanged.connect(lambda *_args: tab._update_controls())
        tab.pose_list.cellDoubleClicked.connect(tab._edit_pose_at)
        layout.addWidget(tab.pose_list, 1)

        copy_controls = QHBoxLayout()
        tab.copy_leg_button = QPushButton("Copy selected leg")
        tab.copy_leg_button.clicked.connect(tab.copy_leg)
        copy_controls.addWidget(tab.copy_leg_button)
        tab.paste_offset = QSpinBox()
        tab.paste_offset.setRange(-100000, 100000)
        tab.paste_offset.setAccessibleName("Paste offset in poses")
        tab.paste_offset.setToolTip("Positive values shift copied poses later, wrapping around the timeline.")
        copy_controls.addWidget(QLabel("Pose offset"))
        copy_controls.addWidget(tab.paste_offset)
        tab.paste_leg_button = QPushButton("Paste to selected leg")
        tab.paste_leg_button.clicked.connect(tab.paste_leg)
        copy_controls.addWidget(tab.paste_leg_button)
        copy_controls.addStretch(1)
        layout.addLayout(copy_controls)

        pose_controls = QHBoxLayout()
        tab.remove_leg_data_button = QPushButton("Remove selected leg data")
        tab.remove_leg_data_button.clicked.connect(tab.remove_selected_leg_data)
        pose_controls.addWidget(tab.remove_leg_data_button)
        tab.remove_pose_button = QPushButton("Delete entire selected step")
        tab.remove_pose_button.clicked.connect(tab.remove_selected_pose)
        pose_controls.addWidget(tab.remove_pose_button)
        tab.play_button = QPushButton("Play recorded motion (loop)")
        tab.play_button.clicked.connect(tab.play_recorded_motion)
        pose_controls.addWidget(tab.play_button)
        tab.stop_playback_button = QPushButton("Stop playback")
        tab.stop_playback_button.clicked.connect(tab.stop_playback)
        pose_controls.addWidget(tab.stop_playback_button)
        tab.smooth_playback = QCheckBox("Smooth timed playback")
        tab.smooth_playback.setToolTip("Adjust servo speeds to pose durations, up to saved speeds.")
        pose_controls.addWidget(tab.smooth_playback)
        layout.addLayout(pose_controls)
        save_controls = QHBoxLayout()
        tab.save_button = QPushButton("Save timeline")
        tab.save_button.clicked.connect(lambda _checked=False: tab.save_timeline())
        save_controls.addWidget(tab.save_button)
        tab.save_as_button = QPushButton("Save as…")
        tab.save_as_button.clicked.connect(
            lambda _checked=False: tab.save_timeline_as()
        )
        save_controls.addWidget(tab.save_as_button)
        layout.addLayout(save_controls)
        tab.status = QLabel(
            "Connect to the robot to begin. Torque remains in its current state after recording."
        )
        tab.status.setWordWrap(True)
        layout.addWidget(tab.status)

    def _update_pose_status(self):
        """Show the latest selected-leg positions and torque states."""
        tab = self.tab
        joints = tab._selected_joints()
        values = []
        for joint in joints:
            position = tab.positions.get(joint)
            text = f"{joint}: {position:.3f}" if position is not None else f"{joint}: —"
            torque = tab.torque_states.get(joint)
            if torque is not None:
                text += f" (torque {'on' if torque else 'off'})"
            values.append(text)
        tab.pose_status.setText("    ".join(values) or "No leg selected")

    def _update_controls(self):
        """Enable only actions supported by the current connection and state."""
        tab = self.tab
        connected = tab.manager is not None
        pending = bool(tab.pending_torque)
        playback_joints = tab._playback_joints()
        selected_conflicts = bool(playback_joints & set(tab._selected_joints()))
        tab.release_button.setEnabled(
            connected and not pending and not tab.recording and not selected_conflicts
        )
        tab.enable_button.setEnabled(
            connected and not pending and not tab.recording and not selected_conflicts
        )
        tab.leg_picker.setEnabled(not tab.recording)
        tab.sample_interval.setEnabled(not tab.recording)
        tab.capture_button.setEnabled(
            not tab.recording and not tab.base_loop_locked and tab.playback.owner is not tab
            and tab._state_is_fresh(tab._capture_joints())
        )
        tab.import_pose_button.setEnabled(
            not tab.recording and not tab.base_loop_locked and tab.playback.owner is not tab
        )
        joints_off = all(
            tab.torque_states.get(joint) is False for joint in tab._selected_joints()
        )
        playing = tab.playback.owner is tab
        tab.copy_leg_button.setEnabled(bool(tab.frames) and not tab.recording and not playing)
        tab.paste_leg_button.setEnabled(
            bool(tab.frames) and tab.copied_leg is not None and not tab.recording and not playing
        )
        tab.paste_offset.setEnabled(not tab.recording and not playing)
        selected_row = tab.pose_list.currentRow()
        selected_entry = (
            tab.timeline_entries[selected_row]
            if 0 <= selected_row < len(tab.timeline_entries) else None
        )
        selected_pose = isinstance(selected_entry, dict) and "positions" in selected_entry
        selected_leg_data = selected_pose and any(
            joint in selected_entry["positions"] for joint in tab._selected_joints()
        )
        tab.record_button.setEnabled(
            connected and not tab.recording and not pending
            and tab._state_is_fresh(tab._capture_joints()) and joints_off
            and (playing or not tab.base_loop_locked)
            and (not playing or tab._preview_row is None)
        )
        tab.stop_button.setEnabled(tab.recording)
        tab.play_step_button.setEnabled(
            connected and selected_entry is not None and not tab.recording and not playing
        )
        tab.edit_pose_button.setEnabled(
            selected_pose and not tab.recording and not playing
        )
        tab.remove_pose_button.setEnabled(
            selected_entry is not None and not tab.recording and not playing
        )
        tab.remove_leg_data_button.setEnabled(
            selected_leg_data and not tab.recording and not playing
        )
        tab.save_button.setEnabled(bool(tab.timeline_entries) and not tab.recording)
        tab.save_as_button.setEnabled(bool(tab.timeline_entries) and not tab.recording)
        tab.play_button.setEnabled(
            connected and bool(tab.timeline_entries) and not tab.recording and not playing
        )
        tab.stop_playback_button.setEnabled(playing)
        tab.new_timeline_button.setEnabled(not tab.recording and not playing)
        tab.load_timeline_button.setEnabled(not tab.recording and not playing)
        tab.save_button.setText("Save timeline" if tab.timeline_path else "Save as timeline…")
        if tab.base_loop_locked:
            tab.record_button.setText(
                "Record leg over one loop" if playing else "Play loop to record another leg"
            )
        else:
            tab.record_button.setText("Record")

    def _set_pose_row(self, row, frame, pose_number):
        tab = self.tab
        label = f" — {frame['label']}" if frame.get("label") else ""
        tab.pose_list.setItem(row, 0, QTableWidgetItem(f"Pose {pose_number}{label}"))
        positions = frame["positions"]
        for column, (_leg, joints) in enumerate(LEG_SERVOS, start=1):
            values = [
                f"{joint.removeprefix('servo_')}: {positions[joint]:.3f}"
                for joint in joints if joint in positions
            ]
            tab.pose_list.setItem(
                row, column, QTableWidgetItem("\n".join(values) if values else "—"),
            )
        tab.pose_list.setRowHeight(row, 58)

    def _set_wait_row(self, row, step):
        tab = self.tab
        tab.pose_list.setItem(
            row, 0, QTableWidgetItem(f"Wait — {step['seconds']:g} s"),
        )
        for column in range(1, len(LEG_SERVOS) + 1):
            tab.pose_list.setItem(row, column, QTableWidgetItem("—"))
        tab.pose_list.setRowHeight(row, 32)

    def _refresh_pose_list(self):
        """Rebuild pose and wait rows after timeline edits."""
        tab = self.tab
        tab.pose_list.clearContents()
        tab.pose_list.setRowCount(len(tab.timeline_entries))
        pose_number = 0
        for row, entry in enumerate(tab.timeline_entries):
            if "positions" in entry:
                pose_number += 1
                tab._set_pose_row(row, entry, pose_number)
            else:
                tab._set_wait_row(row, entry)

    def _highlight_playback_step(self, index):
        """Mark the active pose or wait without changing the user's selection."""
        tab = self.tab
        for row in (tab.playing_row, index):
            if row is None or not 0 <= row < tab.pose_list.rowCount():
                continue
            active = row == index
            for column in range(tab.pose_list.columnCount()):
                item = tab.pose_list.item(row, column)
                if item is not None:
                    item.setBackground(QBrush(QColor("#385e78")) if active else QBrush())
            label = tab.pose_list.item(row, 0)
            if label is not None:
                label.setText(("▶ " if active else "") + label.text().removeprefix("▶ "))
        tab.playing_row = index
