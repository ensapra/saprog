"""Selectable, ROS-independent servo temperature history widget."""

import math
import time
from collections import deque

from PySide6.QtCore import QPointF, QRectF, QSize, QTimer, Qt, Signal, Slot
from PySide6.QtGui import QColor, QIcon, QPainter, QPen, QPixmap
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QToolButton,
    QVBoxLayout,
    QWidget,
)

from desktop_style import ACCENT_LIGHT, BACKGROUND, FOREGROUND, MUTED
from .plot import TemperaturePlot


LEG_SERVOS = (
    ("Front Right Leg", ("servo_11", "servo_12", "servo_13")),
    ("Back Right Leg", ("servo_21", "servo_22", "servo_23")),
    ("Front Left Leg", ("servo_31", "servo_32", "servo_33")),
    ("Back Left Leg", ("servo_41", "servo_42", "servo_43")),
)
SERVO_COLORS = (
    "#70b5f5", "#f2a65a", "#7fd1ae", "#e883a8", "#b49aff", "#e6d36a",
    "#58c5d0", "#fa806e", "#9bc76b", "#d58bdc", "#f0a0ce", "#a7c6ed",
)
HISTORY_SECONDS = 300.0
MAX_SAMPLES = 18000


class TemperatureGraph(QWidget):
    """Show selectable servo temperatures over a user-selected time window.

    A future data adapter can call ``update_temperature`` with a servo name,
    temperature in degrees Celsius, and an optional monotonic timestamp.
    """

    _sample_received = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self._sample_received.connect(self._apply_sample)
        self._samples = {
            name: deque(maxlen=MAX_SAMPLES)
            for _leg, names in LEG_SERVOS
            for name in names
        }
        self._selected = set(self._samples)
        self._checkboxes = {}
        self._latest_labels = {}
        self._colors = {
            name: SERVO_COLORS[index] for index, name in enumerate(self._samples)
        }

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(5)

        servo_panel = QWidget(self)
        servo_layout = QVBoxLayout(servo_panel)
        servo_layout.setContentsMargins(0, 0, 0, 0)
        servo_layout.setSpacing(4)

        servo_grid = QGridLayout()
        servo_grid.setContentsMargins(0, 0, 0, 0)
        servo_grid.setHorizontalSpacing(6)
        servo_grid.setVerticalSpacing(4)
        for index, (leg, servo_names) in enumerate(LEG_SERVOS):
            leg_group = QGroupBox(leg)
            leg_group.setStyleSheet(
                "QGroupBox { margin-top: 5px; padding: 2px 3px 3px 3px; }"
                "QGroupBox::title { left: 4px; padding: 0 2px; }"
            )
            leg_layout = QVBoxLayout(leg_group)
            leg_layout.setContentsMargins(3, 4, 3, 2)
            leg_layout.setSpacing(1)
            for servo_name in servo_names:
                row = QHBoxLayout()
                row.setContentsMargins(0, 0, 0, 0)
                row.setSpacing(3)
                checkbox = QCheckBox(servo_name)
                checkbox.setChecked(True)
                checkbox.toggled.connect(
                    lambda checked, name=servo_name: self._set_selected(name, checked)
                )
                checkbox.setStyleSheet(f"color: {self._colors[servo_name]}; font-size: 9pt;")
                latest = QLabel("—")
                latest.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
                latest.setMinimumWidth(38)
                latest.setStyleSheet(f"color: {MUTED}; font-size: 8pt;")
                row.addWidget(checkbox)
                row.addStretch(1)
                row.addWidget(latest)
                leg_layout.addLayout(row)
                self._checkboxes[servo_name] = checkbox
                self._latest_labels[servo_name] = latest
            servo_grid.addWidget(leg_group, index // 2, index % 2)
        servo_layout.addLayout(servo_grid)

        controls = QHBoxLayout()
        controls.setSpacing(4)
        select_all_button = QPushButton("Select all")
        select_all_button.setObjectName("compactAction")
        select_all_button.clicked.connect(lambda: self._select_all(True))
        deselect_all_button = QPushButton("Deselect all")
        deselect_all_button.setObjectName("compactAction")
        deselect_all_button.clicked.connect(lambda: self._select_all(False))
        controls.addWidget(select_all_button)
        controls.addWidget(deselect_all_button)
        self.status_label = QLabel()
        self.status_label.setObjectName("compactStatus")
        controls.addWidget(self.status_label)
        controls.addStretch(1)
        layout.addWidget(servo_panel)

        controls.addWidget(QLabel("Time window"))
        self.window_input = QDoubleSpinBox()
        self.window_input.setRange(1.0, HISTORY_SECONDS)
        self.window_input.setDecimals(1)
        self.window_input.setSingleStep(5.0)
        self.window_input.setValue(HISTORY_SECONDS)
        self.window_input.setSuffix(" s")
        self.window_input.setKeyboardTracking(False)
        self.window_input.setToolTip("Visible sample history (1–300 seconds)")
        self.window_input.valueChanged.connect(lambda _value: self._refresh_plot())
        controls.addWidget(self.window_input)
        controls.addWidget(self._make_zoom_button(zoom_in=True))
        controls.addWidget(self._make_zoom_button(zoom_in=False))
        layout.addLayout(controls)

        self.plot = TemperaturePlot(self)
        layout.addWidget(self.plot, stretch=1)
        self._timer = QTimer(self)
        self._timer.setInterval(250)
        self._timer.timeout.connect(self._refresh_plot)
        self._timer.start()
        self._refresh_plot()

    def _make_zoom_button(self, *, zoom_in):
        button = QToolButton()
        button.setIcon(self._zoom_icon(zoom_in))
        button.setIconSize(QSize(18, 18))
        button.setFixedSize(28, 26)
        label = "Zoom in" if zoom_in else "Zoom out"
        button.setToolTip(label)
        button.setAccessibleName(label)
        button.clicked.connect(lambda: self._zoom(0.5 if zoom_in else 2.0))
        return button

    @staticmethod
    def _zoom_icon(zoom_in):
        pixmap = QPixmap(20, 20)
        pixmap.fill(Qt.GlobalColor.transparent)
        painter = QPainter(pixmap)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setPen(QPen(QColor(FOREGROUND), 1.8))
        painter.setBrush(Qt.BrushStyle.NoBrush)
        painter.drawEllipse(QRectF(2.0, 2.0, 11.0, 11.0))
        painter.drawLine(QPointF(11.0, 11.0), QPointF(18.0, 18.0))
        painter.drawLine(QPointF(4.5, 7.5), QPointF(10.5, 7.5))
        if zoom_in:
            painter.drawLine(QPointF(7.5, 4.5), QPointF(7.5, 10.5))
        painter.end()
        return QIcon(pixmap)

    def update_temperature(self, servo_name, temperature_c, timestamp=None):
        """Queue a sample without requiring ROS message types.

        ``timestamp`` should use ``time.monotonic()`` units; if omitted, the
        widget supplies one. This method is safe to call from another thread.
        """
        self._sample_received.emit((servo_name, temperature_c, timestamp))

    @Slot(object)
    def _apply_sample(self, sample):
        servo_name, temperature_c, timestamp = sample
        if servo_name not in self._samples:
            return
        try:
            value = float(temperature_c)
            stamp = time.monotonic() if timestamp is None else float(timestamp)
        except (TypeError, ValueError):
            return
        if not math.isfinite(value) or not math.isfinite(stamp):
            return

        points = self._samples[servo_name]
        points.append((stamp, value))
        cutoff = stamp - HISTORY_SECONDS
        while points and points[0][0] < cutoff:
            points.popleft()
        self._latest_labels[servo_name].setText(f"{value:.0f} °C")

    def _set_selected(self, servo_name, selected):
        if selected:
            self._selected.add(servo_name)
        else:
            self._selected.discard(servo_name)
        self._refresh_plot()

    def _select_all(self, selected):
        self._selected = set(self._samples) if selected else set()
        for checkbox in self._checkboxes.values():
            checkbox.blockSignals(True)
            checkbox.setChecked(selected)
            checkbox.blockSignals(False)
        self._refresh_plot()

    def _zoom(self, factor):
        self.window_input.setValue(self.window_input.value() * factor)

    @Slot()
    def _refresh_plot(self, *_args):
        window_seconds = self.window_input.value()
        self.plot.set_samples(self._samples, self._selected, self._colors, window_seconds)
        selected_count = len(self._selected)
        now = time.monotonic()
        live_count = sum(
            bool(points) and now - points[-1][0] <= 2.5
            for points in self._samples.values()
        )
        self.status_label.setText(f"{selected_count}/12 selected · {live_count} live")
