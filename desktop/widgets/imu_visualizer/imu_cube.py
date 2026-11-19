"""Qt-painted IMU scene with ground plane, shadow, compass, and pose labels."""

import math

from PySide6.QtCore import QPointF, QRect, Qt
from PySide6.QtGui import QColor, QFont, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QWidget

from desktop_style import (
    ACCENT_LIGHT,
    BACKGROUND,
    MUTED,
    ROBOT_EDGE,
    ROBOT_FACE_BACK,
    ROBOT_FACE_BOTTOM,
    ROBOT_FACE_FRONT,
    ROBOT_FACE_LEFT,
    ROBOT_FACE_RIGHT,
    ROBOT_FACE_TOP,
    ROBOT_GROUND,
    ROBOT_GROUND_AXIS,
    ROBOT_GROUND_EDGE,
    ROBOT_GROUND_GRID,
    ROBOT_SHADOW,
    ROBOT_TEXT,
)


class IMUCube(QWidget):
    """Draw the sensor pose and its ground reference from a ROS-order quaternion."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._quaternion = None
        self.setMinimumSize(220, 180)

    def set_orientation(self, quaternion):
        """Set orientation from an ``(x, y, z, w)`` quaternion."""
        length = math.sqrt(sum(component * component for component in quaternion))
        if length <= 1e-12:
            return
        self._quaternion = tuple(component / length for component in quaternion)
        self.update()

    @staticmethod
    def _rotation_matrix(quaternion):
        x, y, z, w = quaternion
        return (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        )

    @staticmethod
    def _euler_angles(rotation):
        pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
        if abs(math.cos(pitch)) > 1e-6:
            roll = math.atan2(rotation[2][1], rotation[2][2])
            yaw = math.atan2(rotation[1][0], rotation[0][0])
        else:
            roll = 0.0
            yaw = math.atan2(-rotation[0][1], rotation[1][1])
        return roll, pitch, yaw

    @staticmethod
    def _transform(matrix, point):
        return tuple(sum(row[i] * point[i] for i in range(3)) for row in matrix)

    @staticmethod
    def _convex_hull(points):
        points = sorted(set(points))

        def cross(a, b, c):
            return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

        def half(sequence):
            hull = []
            for point in sequence:
                while len(hull) >= 2 and cross(hull[-2], hull[-1], point) <= 0:
                    hull.pop()
                hull.append(point)
            return hull

        return half(points)[:-1] + half(reversed(points))[:-1]

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(BACKGROUND))

        width, height = self.width(), self.height()
        center_x, center_y = width / 2, height / 2 - 10
        scale = min(width, height - 54) * 0.19

        orientation = self._quaternion or (0.0, 0.0, 0.0, 1.0)
        rotation = self._rotation_matrix(orientation)
        roll, pitch, yaw = self._euler_angles(rotation)

        camera_heading = math.radians(-25)
        camera_pitch_angle = math.radians(-55)
        ch, sh = math.cos(camera_heading), math.sin(camera_heading)
        cp, sp = math.cos(camera_pitch_angle), math.sin(camera_pitch_angle)

        def camera_point(point):
            x, y, z = point
            yaw_x = ch * x - sh * y
            yaw_y = sh * x + ch * y
            return (
                yaw_x,
                cp * yaw_y - sp * z,
                sp * yaw_y + cp * z,
            )

        # Match the original display's fixed IMU mounting orientation.
        mounting_rotation = ((1, 0, 0), (0, -1, 0), (0, 0, -1))
        box_points = (
            (-1.25, -0.85, -0.45), (1.25, -0.85, -0.45),
            (1.25, 0.85, -0.45), (-1.25, 0.85, -0.45),
            (-1.25, -0.85, 0.45), (1.25, -0.85, 0.45),
            (1.25, 0.85, 0.45), (-1.25, 0.85, 0.45),
        )
        world_vertices = [
            self._transform(rotation, self._transform(mounting_rotation, point))
            for point in box_points
        ]
        camera_vertices = [camera_point(point) for point in world_vertices]

        def project_camera(point):
            x, y, z = point
            focal = 7.0
            depth = max(1.5, focal - z)
            return QPointF(
                center_x + x * scale * focal / depth,
                center_y - y * scale * focal / depth,
            )

        def project(point):
            return project_camera(point)

        def ground_point(x, y):
            return project(camera_point((x, y, -1.65)))

        ground = QPolygonF([
            ground_point(x, y)
            for x, y in ((-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5))
        ])
        painter.setPen(QPen(QColor(ROBOT_GROUND_EDGE), 1.5))
        painter.setBrush(QColor(ROBOT_GROUND))
        painter.drawPolygon(ground)

        for step in range(-5, 6):
            offset = step * 0.5
            color = ROBOT_GROUND_AXIS if step == 0 else ROBOT_GROUND_GRID
            painter.setPen(QPen(QColor(color), 1.0))
            painter.drawLine(ground_point(offset, -2.5), ground_point(offset, 2.5))
            painter.drawLine(ground_point(-2.5, offset), ground_point(2.5, offset))

        shadow_hull = self._convex_hull([(x, y) for x, y, _z in world_vertices])
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QColor(ROBOT_SHADOW))
        painter.drawPolygon(QPolygonF([ground_point(x, y) for x, y in shadow_hull]))

        faces = (
            ((0, 1, 2, 3), ROBOT_FACE_FRONT), ((4, 5, 6, 7), ROBOT_FACE_TOP),
            ((0, 1, 5, 4), ROBOT_FACE_LEFT), ((1, 2, 6, 5), ROBOT_FACE_RIGHT),
            ((2, 3, 7, 6), ROBOT_FACE_BOTTOM), ((3, 0, 4, 7), ROBOT_FACE_BACK),
        )
        faces = sorted(faces, key=lambda face: sum(camera_vertices[i][2] for i in face[0]) / 4)
        painter.setPen(QPen(QColor(ROBOT_EDGE), 1.5))
        for indices, color in faces:
            painter.setBrush(QColor(color))
            painter.drawPolygon(QPolygonF([project(camera_vertices[index]) for index in indices]))

        self._draw_compass(painter, ground_point)
        painter.setPen(QColor(ROBOT_TEXT))
        font = QFont(painter.font())
        font.setBold(True)
        painter.setFont(font)
        painter.drawText(
            QRect(10, 4, width - 20, 22),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            f"Roll {math.degrees(roll):5.1f}°   Pitch {math.degrees(pitch):5.1f}°   Yaw {math.degrees(yaw):5.1f}°",
        )
        painter.setFont(QFont())
        painter.setPen(QColor(MUTED))
        state = "Waiting for IMU samples" if self._quaternion is None else "Orientation relative to zero pose"
        painter.drawText(
            QRect(10, 25, width - 20, 14),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            "Ground reference · shadow shows tilt",
        )
        painter.drawText(
            QRect(10, height - 20, width - 20, 16),
            Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter,
            state,
        )
        painter.end()

    @staticmethod
    def _draw_compass(painter, ground_point):
        """Draw a small fixed magnetic north arrow on the ground plane."""
        # North is world +X, along a constant-Y grid line. Use the exact same
        # projected grid points so the arrow lies directly on the ground plane.
        start = ground_point(2.0, 0)
        end = ground_point(2.5, 0)
        painter.setPen(QPen(QColor(ACCENT_LIGHT), 2.0))
        painter.drawLine(start, end)

        dx, dy = end.x() - start.x(), end.y() - start.y()
        length = max(math.hypot(dx, dy), 1e-6)
        ux, uy = dx / length, dy / length
        head_length, half_width = 20, 10
        tip = end
        base = QPointF(end.x() - ux * head_length, end.y() - uy * head_length)
        left = QPointF(base.x() - uy * half_width, base.y() + ux * half_width)
        right = QPointF(base.x() + uy * half_width, base.y() - ux * half_width)
        painter.setBrush(QColor(ACCENT_LIGHT))
        painter.drawPolygon(QPolygonF((tip, left, right)))

        label_center = QPointF(end.x() + 10, end.y() - 9)
        badge = QRect(
            round(label_center.x() - 8), round(label_center.y() - 8), 16, 16
        )
        painter.setPen(QPen(QColor(ACCENT_LIGHT), 1.0))
        painter.setBrush(QColor(BACKGROUND))
        painter.drawRoundedRect(badge, 3, 3)
        font = QFont(painter.font())
        font.setBold(True)
        font.setPointSize(8)
        painter.setFont(font)
        painter.setPen(QColor(ACCENT_LIGHT))
        painter.drawText(badge, Qt.AlignmentFlag.AlignCenter, "N")
