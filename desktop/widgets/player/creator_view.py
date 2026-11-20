"""UI-only Creator pane used by the combined Static Animator widget."""

import json
import math
import os
import re
import tempfile
from decimal import Decimal, ROUND_DOWN
from pathlib import Path

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
from PySide6.QtCore import QSize, Qt, Signal
from PySide6.QtGui import QColor, QIcon, QPainter, QPen, QPixmap
from widgets.servo_layout import LEG_LAYOUT


def _load_servo_limits():
    """Read the repository's last measured per-servo radian limits."""
    path = Path(__file__).resolve().parents[3] / "urdf" / "config" / "measured_servo_limits.json"
    try:
        with path.open("r", encoding="utf-8") as stream:
            servos = json.load(stream).get("servos", ())
    except (OSError, json.JSONDecodeError, AttributeError):
        return {}

    limits = {}
    for item in servos:
        try:
            name = f"servo_{int(item['servo_id'])}"
            limits[name] = (float(item["lower_radians"]), float(item["upper_radians"]))
        except (KeyError, TypeError, ValueError, OverflowError):
            continue
    return limits


def _truncated_rotation(value):
    """Format radians truncated toward zero to two decimal places."""
    return f"{Decimal(str(value)).quantize(Decimal('0.01'), rounding=ROUND_DOWN):.2f}"


class CreatorView(QWidget):
    """Display the Creator controls without connecting them to robot actions."""

    save_requested = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("Snapshot Creator"))

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
        self.target_inputs = {}
        self.servo_limits = _load_servo_limits()
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
                heading.addWidget(QLabel("Current: 0.00"))
                heading.addWidget(QLabel("Torque: Off"))
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
                action_row.addWidget(maximum_label)
                actions = (
                    ("Go To", self.style().standardIcon(QStyle.StandardPixmap.SP_ArrowUp)),
                    ("Puppet Mode", self._person_icon()),
                )
                for text, icon in actions:
                    button = QPushButton()
                    button.setIcon(icon)
                    button.setIconSize(QSize(16, 16))
                    button.setFixedSize(32, 28)
                    button.setToolTip(text)
                    button.setAccessibleName(text)
                    button.setEnabled(False)
                    action_row.addWidget(button)
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
        self.release_button = QPushButton("Release all joints")
        self.release_button.setEnabled(False)
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

        positions = {}
        for servo, target_input in self.target_inputs.items():
            text = target_input.text().strip()
            if not text:
                raise ValueError(f"Enter a target value for {servo}")
            try:
                value = float(text)
            except ValueError as error:
                raise ValueError(f"Target for {servo} must be a number") from error
            if not math.isfinite(value):
                raise ValueError(f"Target for {servo} must be finite")
            positions[servo] = value

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
        with Path(path).open("r", encoding="utf-8") as stream:
            document = json.load(stream)
        positions = document.get("positions") if isinstance(document, dict) else None
        if not isinstance(positions, dict):
            raise ValueError("Keyframe must contain a positions object")

        loaded = 0
        for servo, target_input in self.target_inputs.items():
            if servo not in positions:
                continue
            try:
                value = float(positions[servo])
            except (TypeError, ValueError, OverflowError) as error:
                raise ValueError(f"Invalid target value for {servo}") from error
            if not math.isfinite(value):
                raise ValueError(f"Invalid target value for {servo}")
            target_input.setText(str(positions[servo]))
            loaded += 1
        if not loaded:
            raise ValueError("Keyframe contains no matching servo positions")
        return loaded


__all__ = ["CreatorView"]
