"""Servo-to-URDF calibration, recorded previews and Cartesian foot control."""

import copy
import json
import math
import time
import xml.etree.ElementTree as ET
from bisect import bisect_right
from pathlib import Path

import numpy as np
from documents.snapshot_files import write_json_atomic
from PySide6.QtCore import QSignalBlocker, Qt, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSplitter,
    QVBoxLayout,
    QWidget,
)
from servo_limits import get_servo_limits
from widgets.servo_layout import SERVO_NAMES
from widgets.urdf_leg_view import UrdfLegView

from .joystick_panel import JoystickPanel
from .urdf_model import DESCRIPTION, LEGS, ROOT, SingleLegModel, sequence_frames

STATE_TOPIC = "/saprog/joint_states"


class SingleLegTestTab(QWidget):
    def __init__(self, ros_manager=None, parent=None, *, joystick_monitor=None, joystick_settings=None):
        super().__init__(parent)
        self.manager, self.token = ros_manager, None
        self.limits = get_servo_limits(ros_manager)
        self.model = None
        self.mode = "Manual"
        self.values, self.feedback = {}, {}
        self._feedback_revision = 0
        self._displayed_revision = -1
        self.frames = []
        self.mapping_fields = []
        layout = QVBoxLayout(self)
        title = QLabel("Single leg: URDF calibration and foot control")
        title.setObjectName("title")
        layout.addWidget(title)
        note = QLabel("Match live servo radians to CAD joints, preview a sequence, or move the foot with a "
                      "controller stick. Joystick live control is enabled separately from local previews.")
        note.setWordWrap(True)
        layout.addWidget(note)
        row = QHBoxLayout()
        self.leg_picker = QComboBox()
        for leg in LEGS:
            self.leg_picker.addItem(leg.replace("_", " ").title(), leg)
        row.addWidget(self.leg_picker)
        self.urdf_path = QLineEdit(str(DESCRIPTION / "urdf/front_right.urdf"))
        self.urdf_path.setAccessibleName("Single leg or full robot URDF")
        row.addWidget(self.urdf_path, 1)
        self.button("Browse URDF", self.browse_urdf, row)
        self.button("Load URDF", self.load_model, row)
        self.button("Export single leg URDF", self.export_urdf, row)
        layout.addLayout(row)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        layout.addWidget(splitter, 1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        panel = QWidget()
        self.settings_layout = QVBoxLayout(panel)
        self.mapping_box = QGroupBox("q_joint = gain × servo_radians + offset")
        self.mapping_layout = QVBoxLayout(self.mapping_box)
        self.settings_layout.addWidget(self.mapping_box)
        calibration = QGroupBox("Knee and foot calibration")
        form = QFormLayout(calibration)
        form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        self.multiplier = self.number(-10, 10, 2.4)
        self.knee_offset = self.number(-10, 10, -2.112820865757358)
        form.addRow("Knee / crank gain", self.multiplier)
        form.addRow("Knee offset (rad)", self.knee_offset)
        self.foot_fields = [self.number(-1000, 1000, 0, " mm") for _ in range(3)]
        for axis, field in zip("XYZ", self.foot_fields):
            form.addRow(f"Foot {axis} in lower-leg link", field)
        self.settings_layout.addWidget(calibration)
        self.button("Apply calibration", self.apply_calibration, self.settings_layout)
        row = QHBoxLayout()
        self.button("Save tuning", self.save_tuning, row)
        self.button("Load tuning", self.load_tuning, row)
        self.settings_layout.addLayout(row)
        self.joystick = JoystickPanel(ros_manager, self, monitor=joystick_monitor, settings=joystick_settings)
        self.joystick.start_requested.connect(self.start_joystick)
        self.joystick.engine.pose_changed.connect(self.joystick_pose)
        self.joystick.engine.active_changed.connect(self.joystick_active)
        self.joystick.engine.status_changed.connect(self.status_message)
        self.settings_layout.addStretch()
        scroll.setWidget(panel)
        splitter.addWidget(scroll)
        right = QWidget()
        right_layout = QVBoxLayout(right)
        self.view = UrdfLegView()
        right_layout.addWidget(self.view, 1)
        self.metrics = QLabel()
        self.metrics.setWordWrap(True)
        right_layout.addWidget(self.metrics)
        splitter.addWidget(right)
        joystick_scroll = QScrollArea()
        joystick_scroll.setWidgetResizable(True)
        joystick_scroll.setAccessibleName("Joystick controls to the right of the leg view")
        joystick_scroll.setWidget(self.joystick)
        splitter.addWidget(joystick_scroll)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([320, 650, 380])
        row = QHBoxLayout()
        self.live_button = self.button("Connect live feedback", self.connect_live, row)
        self.button("Stop preview / disconnect", self.stop, row)
        self.button("Clear foot trail", self.clear_trail, row)
        layout.addLayout(row)
        row = QHBoxLayout()
        self.sequence_path = QLineEdit(str(ROOT / "sequences/BackSteps_working.json"))
        self.sequence_path.setAccessibleName("Recorded sequence to preview")
        row.addWidget(self.sequence_path, 1)
        self.button("Browse sequence", self.browse_sequence, row)
        self.button("Preview sequence", self.preview_sequence, row)
        layout.addLayout(row)
        self.status = QLabel("Load a leg to begin.")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.timer = QTimer(self)
        self.timer.setInterval(50)
        self.timer.timeout.connect(self.tick)
        self.leg_picker.currentIndexChanged.connect(self.change_leg)
        if self.manager is not None:
            self.manager.message_received.connect(self.receive)
            self.manager.status_changed.connect(self.provider_status)
        self.live_button.setEnabled(self.manager is not None)
        self.limits.changed.connect(self.limits_changed)
        self.limits.status_changed.connect(self.status.setText)
        self.load_model()
        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.joystick.shutdown)

    def showEvent(self, event):
        super().showEvent(event)
        self.limits.refresh()

    def limits_changed(self, _bounds):
        if self.mode == "Sequence preview":
            self.stop()
        if self.model is not None:
            self.refresh()

    def status_message(self, message):
        self.status.setText(message)

    def start_joystick(self, hardware):
        try:
            model = self.calibrated_model()
            seed = {m.servo: fields["pose"].value()
                    for m, fields in zip(model.mappings, self.mapping_fields)}
            self.stop()
            self.model, self.values = model, seed
            self.joystick.start(model, seed, hardware)
            self.mode = "Joystick live (waiting for feedback)" if hardware else "Joystick preview"
            self.refresh()
        except (ValueError, RuntimeError, KeyError) as error:
            self.status.setText(str(error))
            self.joystick.status.setText(str(error))

    def joystick_pose(self, values):
        self.values = dict(values)
        self.mode = "Joystick live feedback" if self.joystick.engine.hardware else "Joystick preview"
        self.refresh()

    def joystick_active(self, active):
        if not active:
            self.mode = "Manual"
            self.refresh()

    @staticmethod
    def number(low, high, value, suffix="", decimals=5):
        field = QDoubleSpinBox()
        field.setDecimals(decimals)
        field.setRange(low, high)
        field.setValue(value)
        field.setSuffix(suffix)
        field.setSingleStep(.01 if not suffix else 1)
        return field

    @staticmethod
    def button(label, callback, layout):
        button = QPushButton(label)
        button.clicked.connect(callback)
        layout.addWidget(button)
        return button

    def change_leg(self):
        self.urdf_path.setText(str(DESCRIPTION / "urdf/URDF.urdf"))
        self.load_model()

    def browse_urdf(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load leg URDF", self.urdf_path.text(), "URDF (*.urdf)")
        if path:
            self.urdf_path.setText(path)
            self.load_model()

    def load_model(self):
        try:
            model = SingleLegModel(self.urdf_path.text(), self.leg_picker.currentData(), self.limits.bounds)
            self.stop()
            self.model = model
            self.values = {m.servo: .55 if i else 0.
                           for i, m in enumerate(model.mappings)}
            self.build_mapping_form()
            self.populate_calibration()
            self.refresh()
            self.status.setText("Single leg loaded. Servo feedback is already in calibrated radians; "
                                "start with gain 1 and offset 0. Adjust the tracked foot point with Foot X/Y/Z.")
        except (OSError, ValueError, KeyError, TypeError, ET.ParseError) as error:
            self.status.setText(str(error))

    def build_mapping_form(self):
        while self.mapping_layout.count():
            item = self.mapping_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self.mapping_fields = []
        for mapping in self.model.mappings:
            group = QGroupBox(mapping.joint)
            form = QFormLayout(group)
            form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
            picker = QComboBox()
            picker.addItems(SERVO_NAMES)
            picker.setCurrentText(mapping.servo)
            fields = {"servo": picker}
            form.addRow("Received servo name", picker)
            for key in ("gain", "offset"):
                field = self.number(-10, 10, getattr(mapping, key))
                form.addRow(key.title() + (" (rad)" if key != "gain" else ""), field)
                fields[key] = field
            bounds = self.limits.bounds.get(mapping.servo)
            fields["limits"] = QLabel(f"{bounds[0]:.5f} to {bounds[1]:.5f} rad" if bounds else "Waiting for EEPROM read")
            form.addRow("Hardware limits (read only)", fields["limits"])
            manual = self.number(-10, 10, self.values[mapping.servo], " rad")
            form.addRow("Servo pose / received", manual)
            manual.valueChanged.connect(self.manual_pose)
            fields["pose"] = manual
            self.mapping_fields.append(fields)
            self.mapping_layout.addWidget(group)

    def populate_calibration(self):
        self.multiplier.setValue(self.model.multiplier)
        self.knee_offset.setValue(self.model.knee_offset)
        for field, value in zip(self.foot_fields, self.model.foot * 1000):
            field.setValue(float(value))

    def calibrated_model(self):
        if self.model is None:
            raise ValueError("Load a valid URDF first")
        model = copy.copy(self.model)
        model.mappings = [copy.copy(m) for m in self.model.mappings]
        for mapping, fields in zip(model.mappings, self.mapping_fields):
            mapping.servo = fields["servo"].currentText()
            for key in ("gain", "offset"):
                setattr(mapping, key, fields[key].value())
        model.multiplier, model.knee_offset = self.multiplier.value(), self.knee_offset.value()
        model.foot = np.array([f.value()/1000 for f in self.foot_fields])
        model.validate()
        return model

    def apply_calibration(self):
        try:
            model = self.calibrated_model()
            values = {m.servo: fields["pose"].value()
                      for m, fields in zip(model.mappings, self.mapping_fields)}
            self.stop()
            self.model, self.values = model, values
            self.refresh()
            self.status.setText("Calibration applied. Preview the stick movement before controlling the real leg.")
        except ValueError as error:
            self.status.setText(str(error))

    def manual_pose(self):
        if self.model is None:
            return
        values = {m.servo: fields["pose"].value()
                  for m, fields in zip(self.model.mappings, self.mapping_fields)}
        self.stop()
        self.values = values
        self.refresh()

    def refresh(self):
        if self.model is None:
            return
        self.view.set_pose(self.model, self.values)
        angles = self.model.joint_angles(self.values)
        point = self.model.foot_position(self.values)*1000
        rows = [f"{m.servo} {self.values[m.servo]:.5f} → {m.joint} {angles[m.joint]:.5f} rad"
                for m in self.model.mappings]
        rows.append(f"Foot in base: X {point[0]:.2f}, Y {point[1]:.2f}, Z {point[2]:.2f} mm")
        limit_status = (" | EEPROM limits unavailable" if not self.limits.available else
                        " | OUTSIDE hardware limits" if not self.model.in_limits(self.values) else "")
        rows.append(f"Source: {self.mode}" + limit_status)
        self.metrics.setText("\n".join(rows))
        for mapping, fields in zip(self.model.mappings, self.mapping_fields):
            bounds = self.limits.bounds.get(mapping.servo)
            fields["limits"].setText(f"{bounds[0]:.5f} to {bounds[1]:.5f} rad" if bounds else "Waiting for EEPROM read")
            blocker = QSignalBlocker(fields["pose"])
            fields["pose"].setValue(self.values[mapping.servo])
            del blocker

    def clear_trail(self):
        self.view.trail.clear()
        self.view.update()

    def stop(self):
        self.joystick.engine.stop()
        self.timer.stop()
        if self.token is not None:
            self.manager.release(self.token)
            self.token = None
        self.feedback.clear()
        self._feedback_revision = 0
        self._displayed_revision = -1
        self.mode = "Manual"
        self.live_button.setText("Connect live feedback")
        self.refresh()
        self.status.setText("Stopped. Manual pose preview.")

    def connect_live(self):
        if self.model is None or self.manager is None:
            return
        self.stop()
        try:
            self.token = self.manager.request_topic(STATE_TOPIC, "sensor_msgs/msg/JointState",
                                                     package="saprog_dynamixel", executable="dynamixel_control_node")
            self.mode = "Live feedback (waiting)"
            self.live_button.setText("Reconnect live feedback")
            self.timer.start()
            self.status.setText("Waiting for fresh feedback from all three mapped servos.")
            self.refresh()
        except (RuntimeError, ValueError) as error:
            self.status.setText(str(error))

    def receive(self, topic, message):
        if self.token is None or topic != STATE_TOPIC or not isinstance(message, dict):
            return
        names, positions = message.get("name"), message.get("position")
        if not isinstance(names, (list, tuple)) or not isinstance(positions, (list, tuple)):
            return
        now = time.monotonic()
        updated = False
        for name, value in zip(names, positions):
            if (isinstance(name, str) and name in self.values and type(value) in (int, float)
                    and math.isfinite(value)):
                self.feedback[name] = (float(value), now)
                updated = True
        if updated:
            # ROS can deliver bursts. Keep only the newest readings; the display timer
            # applies one complete fresh pose, rather than repainting in this callback.
            self._feedback_revision += 1

    def provider_status(self, topic, status):
        if self.token is not None and topic in (STATE_TOPIC, "") and status != "Connected":
            self.status.setText(status)

    def browse_sequence(self):
        path, _ = QFileDialog.getOpenFileName(self, "Recorded sequence", str(ROOT / "sequences"), "JSON (*.json)")
        if path:
            self.sequence_path.setText(path)

    def preview_sequence(self):
        if self.model is None:
            return
        try:
            self.limits.require(self.values)
            frames, loop = sequence_frames(self.sequence_path.text(), self.values, self.values, self.limits.bounds)
            self.begin_preview(frames, loop, "Sequence preview")
            self.status.setText("Recorded servo targets, interpolated using saved durations. "
                                "Speed/torque dynamics are not simulated; live data is disconnected.")
        except (OSError, ValueError, KeyError, TypeError) as error:
            self.status.setText(str(error))

    def begin_preview(self, frames, loop, mode):
        self.stop()
        self.frames, self.loop = frames, loop
        self.frame_ends = np.cumsum([seconds for seconds, _ in frames]).tolist()
        self.preview_duration = self.frame_ends[-1]
        self.preview_seed = dict(self.values)
        self.started = time.monotonic()
        self.mode = mode
        self.clear_trail()
        self.timer.start()
        self.refresh()

    def tick(self):
        now = time.monotonic()
        if self.token is not None:
            if any(s not in self.feedback or now-self.feedback[s][1] >= 2 for s in self.values):
                if self.mode != "Live feedback (stale / incomplete)":
                    self.mode = "Live feedback (stale / incomplete)"
                    self.status.setText("Feedback stale or incomplete; holding the last displayed pose.")
                    self.refresh()
            elif self._feedback_revision != self._displayed_revision:
                self._displayed_revision = self._feedback_revision
                values = {s: self.feedback[s][0] for s in self.values}
                if values != self.values or self.mode != "Live feedback":
                    self.values = values
                    self.mode = "Live feedback"
                    self.status.setText("Receiving actual servo positions. Values outside limits remain visible.")
                    self.refresh()
            return
        total = self.preview_duration
        elapsed = now-self.started
        if not self.loop and elapsed >= total:
            self.values = dict(self.frames[-1][1])
            self.timer.stop()
            self.refresh()
            self.status.setText("Sequence preview complete.")
            return
        cycle = int(elapsed // total)
        elapsed %= total
        index = min(bisect_right(self.frame_ends, elapsed), len(self.frames)-1)
        seconds, target = self.frames[index]
        previous = (self.frames[index-1][1] if index else
                    self.frames[-1][1] if cycle else self.preview_seed)
        start = self.frame_ends[index-1] if index else 0.
        alpha = (elapsed-start)/seconds if seconds else 1.
        self.values = {s: previous[s]+alpha*(target[s]-previous[s]) for s in previous}
        self.refresh()

    def export_urdf(self):
        if self.model is None:
            return
        path, _ = QFileDialog.getSaveFileName(self, "Export single leg", self.model.leg + ".urdf", "URDF (*.urdf)")
        if path:
            try:
                self.calibrated_model().export_urdf(path)
                self.status.setText("Single-leg geometry exported. Save tuning separately for servo calibration and foot point.")
            except (OSError, ValueError) as error:
                self.status.setText(str(error))

    def save_tuning(self):
        try:
            model = self.calibrated_model()
            data = {"version": 2, "urdf": str(model.path), "leg": model.leg,
                    "mappings": [vars(m) for m in model.mappings], "foot": model.foot.tolist(),
                    "multiplier": model.multiplier, "knee_offset": model.knee_offset}
            path, _ = QFileDialog.getSaveFileName(self, "Save leg tuning", "leg-tuning.json", "JSON (*.json)")
            if path:
                write_json_atomic(path, data)
                self.status.setText("Leg tuning saved.")
        except (OSError, ValueError) as error:
            self.status.setText(str(error))

    def load_tuning(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load leg tuning", "", "JSON (*.json)")
        if not path:
            return
        try:
            data = json.loads(Path(path).read_text(encoding="utf-8"))
            if data["version"] not in (1, 2):
                raise ValueError("Unsupported leg tuning version")
            model = SingleLegModel(data["urdf"], data["leg"], self.limits.bounds)
            if len(data["mappings"]) != 3 or len(data["foot"]) != 3:
                raise ValueError("Incomplete leg tuning")
            for mapping, saved in zip(model.mappings, data["mappings"]):
                if saved["joint"] != mapping.joint or saved["servo"] not in SERVO_NAMES:
                    raise ValueError("Tuning does not match the URDF joints")
                for key in ("servo", "gain", "offset"):
                    if key != "servo" and (type(saved[key]) not in (int, float)
                                             or not math.isfinite(saved[key]) or not -10 <= saved[key] <= 10):
                        raise ValueError("Mapping calibration is outside the supported range")
                    setattr(mapping, key, saved[key])
            model.foot = np.array(data["foot"], dtype=float)
            model.multiplier, model.knee_offset = data["multiplier"], data["knee_offset"]
            model.validate()
            if (np.any(abs(model.foot) > 1) or abs(model.multiplier) > 10
                    or abs(model.knee_offset) > 10):
                raise ValueError("Foot or knee calibration is outside the supported range")
            self.stop()
            self.model = model
            self.values = {m.servo: .55 if i else 0. for i, m in enumerate(model.mappings)}
            blocker = QSignalBlocker(self.leg_picker)
            self.leg_picker.setCurrentIndex(list(LEGS).index(model.leg))
            del blocker
            self.urdf_path.setText(str(model.path))
            self.build_mapping_form()
            self.populate_calibration()
            self.refresh()
            self.status.setText("Leg tuning loaded.")
        except (OSError, ValueError, TypeError, KeyError, ET.ParseError) as error:
            self.status.setText(str(error))

    def closeEvent(self, event):
        self.stop()
        self.joystick.shutdown()
        super().closeEvent(event)

    def hideEvent(self, event):
        self.joystick.engine.stop("Joystick stopped: Single Leg Test is hidden")
        super().hideEvent(event)
