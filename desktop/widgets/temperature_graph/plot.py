"""PyQtGraph view for servo temperature history."""

import time

import PySide6  # Select the application's Qt binding before importing pyqtgraph.
import pyqtgraph as pg
from PySide6.QtCore import Qt

from desktop_style import ACCENT_LIGHT, BACKGROUND, FOREGROUND, MUTED
from desktop_style import ACCENT_LIGHT


class TemperaturePlot(pg.PlotWidget):
    """Plot selected servo traces with a fixed, rolling time window."""

    def __init__(self, parent=None):
        super().__init__(parent=parent, background=BACKGROUND)
        self._curves = {}
        self._empty_message = pg.TextItem(color=MUTED, anchor=(0.5, 0.5))
        self.addItem(self._empty_message)
        self.setMinimumHeight(180)
        self.setMenuEnabled(False)
        self.setMouseEnabled(x=False, y=False)
        self.showGrid(x=True, y=True, alpha=0.25)
        self.setLabel("left", "Temperature", units="°C", color=FOREGROUND)
        self.setLabel("bottom", "Time before now", units="s", color=FOREGROUND)
        plot = self.getPlotItem()
        plot.getAxis("left").setPen(MUTED)
        plot.getAxis("bottom").setPen(MUTED)
        plot.setLimits(xMin=-300, xMax=0)
        self._temperature_limit = pg.InfiniteLine(
            pos=75,
            angle=0,
            pen=pg.mkPen(ACCENT_LIGHT, width=1.5, style=Qt.PenStyle.DashLine),
            movable=False,
            label="75 °C",
            labelOpts={"position": 0.98, "color": ACCENT_LIGHT},
        )
        self.addItem(self._temperature_limit)

    def set_samples(self, samples, selected, colors, window_seconds):
        now = time.monotonic()
        cutoff = now - window_seconds
        visible_values = []

        for name, all_points in samples.items():
            curve = self._curves.get(name)
            if curve is None:
                curve = self.plot([], [], pen=pg.mkPen(colors[name], width=2), name=name)
                self._curves[name] = curve

            is_selected = name in selected
            curve.setVisible(is_selected)
            if not is_selected:
                continue

            points = [(stamp, value) for stamp, value in all_points if stamp >= cutoff]
            visible_values.extend(value for _stamp, value in points)
            x_values, y_values = [], []
            previous_stamp = None
            for stamp, value in points:
                if previous_stamp is not None and stamp - previous_stamp > 2.5:
                    x_values.append(float("nan"))
                    y_values.append(float("nan"))
                x_values.append(stamp - now)
                y_values.append(value)
                previous_stamp = stamp
            curve.setData(x_values, y_values, connect="finite")

        self.setXRange(-window_seconds, 0, padding=0)
        if visible_values:
            low = min(min(visible_values) - 5, 70)
            high = max(max(visible_values) + 5, 80)
            if high - low < 10:
                midpoint = (low + high) / 2
                low, high = midpoint - 5, midpoint + 5
            self.setYRange(low, high, padding=0)
            self._empty_message.setVisible(False)
        else:
            self.setYRange(15, 80, padding=0)
            self._empty_message.setText(
                "Waiting for selected servo temperatures" if selected else "Select servos to graph"
            )
            self._empty_message.setPos(-window_seconds / 2, 32.5)
            self._empty_message.setVisible(True)
