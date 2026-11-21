"""UI-only Creator pane used by the combined Static Animator widget."""

import json
import math
import os
import re
import tempfile
from decimal import ROUND_DOWN, Decimal
from pathlib import Path

from documents.snapshot_units import clamp_to_bounds
from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtGui import QColor, QIcon, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QSpinBox,
    QStyle,
    QVBoxLayout,
    QWidget,
)

from widgets.servo_layout import LEG_LAYOUT

from .snapshot_loader import load_keyframe


def _truncated_rotation(value):
    """Format radians truncated toward zero to two decimal places."""
    return f"{Decimal(str(value)).quantize(Decimal('0.01'), rounding=ROUND_DOWN):.2f}"


class CreatorView(QWidget):
    """Display the Creator controls without connecting them to robot actions."""

    save_requested = Signal()
    release_all_torque_requested = Signal()
    move_requested = Signal(str)
    puppet_mode_requested = Signal(str, bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.speed_slider = QSlider(Qt.Orientation.Horizontal)
        self.speed_slider.setRange(0, 1023)
        self.speed_slider.setValue(512)
        self.torque_slider = QSlider(Qt.Orientation.Horizontal)
        self.torque_slider.setRange(0, 1023)
        self.torque_slider.setValue(512)
        for title, slider in (
            ("Movement speed", self.speed_slider),
            ("Torque strength", self.torque_slider),
        ):
            row = QHBoxLayout()
            label = QLabel(title)
            label.setMinimumWidth(120)
            row.addWidget(label)
            row.addWidget(slider, stretch=1)
            value_input = QSpinBox()
            value_input.setRange(slider.minimum(), slider.maximum())
            value_input.setValue(slider.value())
            value_input.setFixedWidth(72)
            slider.valueChanged.connect(value_input.setValue)
            value_input.valueChanged.connect(slider.setValue)
            row.addWidget(value_input)
            row.addWidget(QLabel(f"/ {slider.maximum()}"))
            layout.addLayout(row)

        hint = QLabel("Applies to the next Fix or Load Snapshot.")
        hint.setStyleSheet("color: #8da6c1;")
        layout.addWidget(hint)

        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        joint_panel = QWidget()
        joint_layout = QGridLayout(joint_panel)
        joint_layout.setContentsMargins(6, 6, 6, 6)
        joint_layout.setHorizontalSpacing(8)
        joint_layout.setVerticalSpacing(6)
        self.target_inputs = {}
        self.limit_labels = {}
        self.current_labels = {}
        self.torque_labels = {}
        self.current_positions = {}
        self.torque_states = {}
        self.go_to_buttons = {}
        self.puppet_buttons = {}
        self._puppet_pending = set()
        self._ros_available = False
        self.servo_limits = {}
        for name, servos, row, column in LEG_LAYOUT:
            group = QGroupBox(name)
            servo_layout = QVBoxLayout(group)
            for servo in servos:
                servo_view = QWidget(group)
                controls = QVBoxLayout(servo_view)
                controls.setContentsMargins(2, 2, 2, 4)
                controls.setSpacing(3)

                heading = QHBoxLayout()
                heading.addWidget(QLabel(servo))
                heading.addStretch(1)
                current_label = QLabel("Current: —")
                torque_label = QLabel("Torque: Unknown")
                self.current_labels[servo] = current_label
                self.torque_labels[servo] = torque_label
                self.current_positions[servo] = None
                self.torque_states[servo] = None
                heading.addWidget(current_label)
                heading.addWidget(torque_label)
                controls.addLayout(heading)

                target_input = QLineEdit()
                target_input.setPlaceholderText("Target value")
                target_input.setToolTip(f"Target value for {servo}")
                self.target_inputs[servo] = target_input
                action_row = QHBoxLayout()
                lower, upper = self.servo_limits.get(servo, (None, None))
                minimum_label = QLabel(
                    _truncated_rotation(lower) if lower is not None else "—"
                )
                minimum_label.setToolTip("Minimum rotation (radians)")
                action_row.addWidget(minimum_label)
                action_row.addWidget(target_input, stretch=1)
                maximum_label = QLabel(
                    _truncated_rotation(upper) if upper is not None else "—"
                )
                maximum_label.setToolTip("Maximum rotation (radians)")
                self.limit_labels[servo] = (minimum_label, maximum_label)
                action_row.addWidget(maximum_label)
                go_to = QPushButton()
                go_to.setIcon(self.style().standardIcon(QStyle.StandardPixmap.SP_ArrowUp))
                go_to.setIconSize(QSize(16, 16))
                go_to.setFixedSize(32, 28)
                go_to.setToolTip(f"Move {servo} to target")
                go_to.setAccessibleName(f"Go to target for {servo}")
                go_to.setEnabled(False)
                go_to.clicked.connect(lambda _checked=False, name=servo: self.move_requested.emit(name))
                self.go_to_buttons[servo] = go_to
                action_row.addWidget(go_to)
                puppet = QPushButton()
                puppet.setIcon(self._person_icon())
                puppet.setIconSize(QSize(16, 16))
                puppet.setFixedSize(32, 28)
                puppet.setToolTip(f"Toggle torque for {servo}")
                puppet.setAccessibleName(f"Puppet mode for {servo}")
                puppet.setEnabled(False)
                puppet.setCheckable(True)
                puppet.toggled.connect(
                    lambda enabled, name=servo: self._puppet_mode_toggled(name, enabled)
                )
                self.puppet_buttons[servo] = puppet
                action_row.addWidget(puppet)
                controls.addLayout(action_row)
                servo_layout.addWidget(servo_view)
            joint_layout.setColumnStretch(column, 1)
            joint_layout.addWidget(group, row, column)
        joint_layout.setRowStretch(2, 1)
        scroll.setWidget(joint_panel)
        layout.addWidget(scroll, stretch=1)

        save_row = QHBoxLayout()
        save_row.addWidget(QLabel("Keyframe name"))
        self.name_input = QLineEdit()
        self.name_input.setPlaceholderText("Enter a name")
        save_row.addWidget(self.name_input, stretch=1)
        layout.addLayout(save_row)

        actions = QHBoxLayout()
        self.save_button = QPushButton("Save targets")
        self.save_button.clicked.connect(self.save_requested.emit)
        actions.addWidget(self.save_button)
        self.finish_edit_button = QPushButton("Finish editing")
        self.finish_edit_button.hide()
        actions.addWidget(self.finish_edit_button)
        self.release_button = QPushButton("Release all torque")
        self.release_button.setToolTip("Cancel queued motion and disable torque for every configured servo")
        self.release_button.clicked.connect(self.release_all_torque_requested.emit)
        actions.addWidget(self.release_button)
        layout.addLayout(actions)

    def save_targets(self, directory):
        """Save the current target fields as a snapshot JSON document."""
        name = self.name_input.text().strip()
        if not name:
            raise ValueError("Enter a keyframe name before saving")

        filename = re.sub(r"[^A-Za-z0-9_.-]+", "_", name).strip("._")[:80]
        if not filename:
            raise ValueError("The keyframe name must contain letters or numbers")

        positions = self.target_positions()

        directory = Path(directory).expanduser()
        directory.mkdir(parents=True, exist_ok=True)
        path = directory / f"{filename}.json"
        if path.exists():
            answer = QMessageBox.question(
                self,
                "Overwrite keyframe?",
                f"Replace {path.name}?",
                QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
                QMessageBox.StandardButton.No,
            )
            if answer != QMessageBox.StandardButton.Yes:
                return None

        document = {"schema_version": 1, "name": name, "positions": positions}
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", dir=directory, suffix=".tmp", delete=False
            ) as stream:
                temporary = Path(stream.name)
                json.dump(document, stream, indent=2, allow_nan=False)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        return path

    def target_positions(self, servos=None):
        """Validate targets for a whole snapshot or a recorded pose's joints."""
        positions = {}
        for servo in self.target_inputs if servos is None else servos:
            target_input = self.target_inputs[servo]
            text = target_input.text().strip()
            if not text:
                raise ValueError(f"Enter a target value for {servo}")
            try:
                value = float(text)
            except ValueError as error:
                raise ValueError(f"Target for {servo} must be a number") from error
            if not math.isfinite(value):
                raise ValueError(f"Target for {servo} must be finite")
            limits = self.servo_limits.get(servo)
            if limits is not None:
                value = clamp_to_bounds(value, limits)
                target_input.setText(repr(value))
            positions[servo] = value

        return positions

    def _person_icon(self):
        """Draw a small person glyph for the Puppet Mode control."""
        pixmap = QPixmap(20, 20)
        pixmap.fill(Qt.GlobalColor.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(QPen(QColor("#e5edf7"), 2))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(8, 2, 4, 4)
        painter.drawLine(10, 7, 10, 13)
        painter.drawLine(5, 9, 15, 9)
        painter.drawLine(10, 13, 6, 18)
        painter.drawLine(10, 13, 14, 18)
        painter.end()
        return QIcon(pixmap)

    def load_keyframe_fields(self, path):
        """Populate target inputs from a keyframe JSON file."""
        _name, positions = load_keyframe(path, self.target_inputs, self.servo_limits)
        for servo, value in positions.items():
            if not self.puppet_buttons[servo].isChecked():
                self.target_inputs[servo].setText(str(value))
        return len(positions)

    def set_servo_limits(self, limits):
        """Use freshly measured bounds for both labels and target validation."""
        self.servo_limits = limits
        for servo, (minimum, maximum) in self.limit_labels.items():
            lower, upper = limits.get(servo, (None, None))
            minimum.setText(_truncated_rotation(lower) if lower is not None else "—")
            maximum.setText(_truncated_rotation(upper) if upper is not None else "—")
            if lower is not None and upper is not None:
                try:
                    value = float(self.target_inputs[servo].text().strip())
                except ValueError:
                    continue
                if math.isfinite(value):
                    self.target_inputs[servo].setText(
                        repr(clamp_to_bounds(value, (lower, upper)))
                    )

    def set_ros_available(self, available):
        self._ros_available = bool(available)
        for servo in self.target_inputs:
            self._update_puppet_controls(servo)

    def set_puppet_pending(self, servo, pending):
        if pending:
            self._puppet_pending.add(servo)
        else:
            self._puppet_pending.discard(servo)
        self._update_puppet_controls(servo)

    def set_puppet_mode(self, servo, active):
        """Restore mode after a failed request without sending another command."""
        button = self.puppet_buttons[servo]
        blocked = button.blockSignals(True)
        try:
            button.setChecked(active)
        finally:
            button.blockSignals(blocked)
        self._update_puppet_controls(servo)

    def _update_puppet_controls(self, servo):
        active = self.puppet_buttons[servo].isChecked()
        pending = servo in self._puppet_pending
        self.target_inputs[servo].setEnabled(not active and not pending)
        self.go_to_buttons[servo].setEnabled(self._ros_available and not active and not pending)
        self.puppet_buttons[servo].setEnabled(self._ros_available and not pending)
        self.puppet_buttons[servo].setToolTip(
            f"{'Exit' if active else 'Enter'} Puppet Mode for {servo}"
        )

    def _puppet_mode_toggled(self, servo, enabled):
        if enabled:
            position = self.current_positions[servo]
            if position is not None:
                self.target_inputs[servo].setText(repr(position))
        self._update_puppet_controls(servo)
        self.puppet_mode_requested.emit(servo, enabled)

    def update_servo_state(self, servo, position=None, torque_enabled=None):
        if servo not in self.current_labels:
            return
        if position is not None:
            position = float(position)
            if not math.isfinite(position):
                return
            self.current_positions[servo] = position
            self.current_labels[servo].setText(f"Current: {position:.2f}")
            if self.puppet_buttons[servo].isChecked():
                self.target_inputs[servo].setText(repr(position))
        if torque_enabled is not None:
            enabled = bool(torque_enabled)
            self.torque_states[servo] = enabled
            self.torque_labels[servo].setText(f"Torque: {'On' if enabled else 'Off'}")


__all__ = ["CreatorView"]
