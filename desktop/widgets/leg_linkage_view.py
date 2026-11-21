"""Side-view rendering of measured simulation points, without a graphics process."""

import math
from collections import deque

from desktop_style import ACCENT_LIGHT, BACKGROUND, BORDER, FOREGROUND, GRID, MUTED, ROBOT_FACE_TOP
from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget


class LegLinkageView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(320, 280)
        self.setAccessibleName("Single-leg linkage side view")
        self.points = {}
        self.trail = deque(maxlen=500)
        self.last_time = None

    def clear(self):
        self.points = {}
        self.trail.clear()
        self.last_time = None
        self.update()

    def set_state(self, state):
        points = state.get("points", {})
        if not all(name in points for name in ("O", "A", "B", "C", "D", "F")):
            return
        if not all(isinstance(p, (list, tuple)) and len(p) == 2 and
                   all(type(v) in (int, float) and math.isfinite(v) for v in p)
                   for p in points.values()):
            return
        stamp = state.get("sim_time", 0)
        if self.last_time is not None and stamp < self.last_time:
            self.trail.clear()
        if stamp != self.last_time:
            self.trail.append(tuple(points["F"]))
        self.last_time = stamp
        self.points = points
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(BACKGROUND))
        painter.setPen(QColor(MUTED))
        painter.drawText(14, 22, "Side view: X right, Z up | fixed motor and knee pivots")
        if not self.points:
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter,
                             "Start the test to view the physical linkage")
            return
        # A fixed view around O/D prevents the drawing from zooming as the foot moves.
        reach = max(math.dist(self.points["D"], self.points["F"]),
                    math.dist(self.points["O"], self.points["A"]),
                    math.dist(self.points["A"], self.points["B"]), 0.01)
        xs = [self.points[name][0] for name in ("O", "D")]
        zs = [self.points[name][1] for name in ("O", "D")]
        xmin, xmax = min(xs) - reach, max(xs) + reach
        zmin, zmax = min(zs) - reach, max(zs) + reach
        scale = min((self.width() - 80) / (xmax - xmin),
                    (self.height() - 95) / (zmax - zmin))
        cx, cz = (xmin + xmax) / 2, (zmin + zmax) / 2

        def project(point):
            return QPointF(self.width() / 2 + (point[0] - cx) * scale,
                           (self.height() + 20) / 2 - (point[1] - cz) * scale)

        painter.setPen(QPen(QColor(GRID), 1))
        spacing = 0.025
        for i in range(math.floor(xmin / spacing), math.ceil(xmax / spacing) + 1):
            painter.drawLine(project((i * spacing, zmin)), project((i * spacing, zmax)))
        for i in range(math.floor(zmin / spacing), math.ceil(zmax / spacing) + 1):
            painter.drawLine(project((xmin, i * spacing)), project((xmax, i * spacing)))
        painter.setPen(QPen(QColor(BORDER), 1, Qt.PenStyle.DashLine))
        painter.drawLine(project(self.points["O"]), project(self.points["D"]))
        if len(self.trail) > 1:
            painter.setPen(QPen(QColor(MUTED), 1))
            painter.drawPolyline(QPolygonF([project(p) for p in self.trail]))
        for a, b, color, width in (
            ("O", "A", ACCENT_LIGHT, 7), ("A", "B", FOREGROUND, 4),
            ("D", "C", ROBOT_FACE_TOP, 6), ("D", "F", ROBOT_FACE_TOP, 7),
        ):
            painter.setPen(QPen(QColor(color), width, Qt.PenStyle.SolidLine,
                                Qt.PenCapStyle.RoundCap))
            painter.drawLine(project(self.points[a]), project(self.points[b]))
        for name in ("O", "A", "C", "D", "F"):
            point = project(self.points[name])
            painter.setPen(QPen(QColor(FOREGROUND), 1))
            painter.setBrush(QColor(BACKGROUND))
            if name in ("O", "D"):
                painter.drawRect(point.x() - 5, point.y() - 5, 10, 10)
            else:
                painter.drawEllipse(point, 4, 4)
            label = {"O": "Motor", "A": "Crank pin", "C": "Knee pin", "D": "Knee", "F": "Foot"}[name]
            offset = (-painter.fontMetrics().horizontalAdvance(label) - 10) if name in ("O", "D") else 9
            painter.drawText(point + QPointF(offset, -7), label)
        painter.setPen(QColor(MUTED))
        painter.drawText(14, self.height() - 12, "25 mm grid | foot trail | hanging rig, no ground contact")
