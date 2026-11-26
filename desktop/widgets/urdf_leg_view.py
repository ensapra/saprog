"""Lightweight orthographic CAD mesh viewer, using the existing Qt/numpy stack."""

import math
from collections import deque

import numpy as np
from desktop_style import ACCENT_LIGHT, BACKGROUND, GRID, MUTED
from pyqtgraph import functions as pg_functions
from PySide6.QtCore import QPointF, Qt
from PySide6.QtGui import QColor, QPainter, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import QWidget


def display_mesh(triangles, cell=.002):
    """Cluster nearby display vertices; kinematics retain the full-resolution foot point."""
    vertices = triangles.reshape(-1, 3)
    _, inverse = np.unique(np.floor(vertices/cell).astype(int), axis=0, return_inverse=True)
    counts = np.bincount(inverse)
    reduced = np.column_stack([np.bincount(inverse, weights=vertices[:, i])/counts for i in range(3)])
    faces = inverse.reshape(-1, 3)
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 0] != faces[:, 2]) & (faces[:, 1] != faces[:, 2])]
    _, unique = np.unique(np.sort(faces, axis=1), axis=0, return_index=True)
    return reduced[faces[unique]]


class UrdfLegView(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(350, 330)
        self.setAccessibleName("Single leg URDF meshes and foot path")
        self.model = None
        self.values = {}
        self.trail = deque(maxlen=600)
        self.azimuth, self.elevation = 35., 20.
        self.drag = None
        self.center = np.zeros(3)
        self.meshes = []
        self.colors = []
        self._visual_source = None
        self._mesh_cache = None
        self._mesh_key = None
        self._pose_revision = 0
        self._transforms = {}
        self._foot = np.zeros(3)
        # Reuse Qt's polygon storage instead of constructing three QPointFs per triangle.
        self._triangle = pg_functions.create_qpolygonf(3)
        self._triangle_points = pg_functions.ndarray_from_qpolygonf(self._triangle)

    def set_pose(self, model, values):
        if model is self.model and self.values == values:
            return
        if model is not self.model:
            self.trail.clear()
            transforms, _ = model.transforms(values)
            root = transforms[model.joints[0].find("child").get("link")][:3, 3]
            self.center = (root + model.foot_position(values)) / 2
        if model.visuals is not self._visual_source:
            self._visual_source = model.visuals
            self.meshes, self.colors = [], []
            for name, triangles, rgb in model.visuals:
                triangles = display_mesh(triangles, .003)
                vertices, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
                normal = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
                normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-12)
                self.meshes.append((name, vertices, indices.reshape(-1, 3), normal))
                self.colors.extend(QColor.fromRgbF(*(float(min(1, max(.35, c)*(.65+.35*i/15)))
                                                    for c in rgb)) for i in range(16))
        self.model, self.values = model, dict(values)
        self._transforms, _ = model.transforms(values)
        t = self._transforms[model.foot_link]
        foot = t[:3, :3] @ model.foot + t[:3, 3]
        self._foot = foot
        self._pose_revision += 1
        if not self.trail or np.linalg.norm(foot - self.trail[-1]) > .0001:
            self.trail.append(foot)
        self.update()

    def mousePressEvent(self, event):
        self.drag = event.position()

    def mouseMoveEvent(self, event):
        if self.drag is not None:
            delta = event.position() - self.drag
            self.azimuth -= delta.x() / 2
            self.elevation = max(-85, min(85, self.elevation + delta.y() / 2))
            self.drag = event.position()
            self.update()

    def mouseReleaseEvent(self, event):
        self.drag = None

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.fillRect(self.rect(), QColor(BACKGROUND))
        painter.setPen(QColor(MUTED))
        painter.drawText(12, 22, "URDF meshes | base Z up | drag to orbit")
        if self.model is None:
            return
        transforms = self._transforms
        a, e = math.radians(self.azimuth), math.radians(self.elevation)
        camera = np.array([[-math.sin(a), math.cos(a), 0],
                           [-math.cos(a)*math.sin(e), -math.sin(a)*math.sin(e), math.cos(e)],
                           [math.cos(a)*math.cos(e), math.sin(a)*math.cos(e), math.sin(e)]])
        foot = self._foot
        root = transforms[self.model.joints[0].find("child").get("link")][:3, 3]
        center = self.center
        scale = min(self.width() - 60, self.height() - 80) / .38
        def project(points):
            p = (np.asarray(points) - center) @ camera.T
            return np.column_stack((self.width()/2 + p[:, 0]*scale,
                                    self.height()/2 - p[:, 1]*scale, p[:, 2]))
        def point(p):
            return QPointF(float(p[0]), float(p[1]))
        painter.setPen(QPen(QColor(GRID), 1))
        for axis, label in enumerate(("X", "Y", "Z")):
            end = root + np.eye(3)[axis]*.04
            p = project([root, end])
            painter.drawLine(point(p[0]), point(p[1]))
            painter.drawText(point(p[1]), label)
        # Expose/overlay repaints reuse the mesh image until pose, camera or viewport changes.
        key = (self._pose_revision, self.width(), self.height(), self.devicePixelRatioF(),
               self.azimuth, self.elevation)
        if key != self._mesh_key:
            ratio = self.devicePixelRatioF()
            self._mesh_cache = QPixmap(round(self.width()*ratio), round(self.height()*ratio))
            self._mesh_cache.setDevicePixelRatio(ratio)
            self._mesh_cache.fill(Qt.GlobalColor.transparent)
            mesh_painter = QPainter(self._mesh_cache)
            # Triangle interiors do not need edge antialiasing. Rasterizing every shared
            # edge both costs CPU and introduces hairline seams between adjacent faces.
            mesh_painter.setRenderHint(QPainter.RenderHint.Antialiasing, False)
            self.draw_meshes(mesh_painter, camera, project)
            mesh_painter.end()
            self._mesh_key = key
        painter.drawPixmap(0, 0, self._mesh_cache)
        if len(self.trail) > 1:
            painter.setPen(QPen(QColor(ACCENT_LIGHT), 2))
            painter.drawPolyline(QPolygonF([point(p) for p in project(self.trail)]))
        painter.setPen(QPen(QColor(ACCENT_LIGHT), 2))
        painter.setBrush(QColor(ACCENT_LIGHT))
        p = point(project([foot])[0])
        painter.drawEllipse(p, 4, 4)
        painter.drawText(p + QPointF(8, -8), "Foot")
        painter.setPen(QColor(MUTED))
        painter.drawText(12, self.height()-12, "Blue: foot trail")

    def draw_meshes(self, painter, camera, project):
        """Render a changed pose. Geometry, normals and Qt point storage are reused."""
        faces, depths, colors = [], [], []
        light = np.array([.3, -.4, .866])
        for index, (name, vertices, indices, normal) in enumerate(self.meshes):
            t = self._transforms[name]
            world = vertices @ t[:3, :3].T + t[:3, 3]
            screen = project(world)[indices]
            ab, ac = screen[:, 1, :2] - screen[:, 0, :2], screen[:, 2, :2] - screen[:, 0, :2]
            area = ab[:, 0]*ac[:, 1] - ab[:, 1]*ac[:, 0]
            visible = area < -.8
            shade = np.clip((abs(normal @ (t[:3, :3].T @ light))*15).astype(int), 0, 15)
            faces.append(screen[visible])
            depths.append(screen[visible, :, 2].mean(axis=1))
            colors.append(index*16 + shade[visible])
        painter.setPen(Qt.PenStyle.NoPen)
        if faces:
            faces, depths, colors = np.concatenate(faces), np.concatenate(depths), np.concatenate(colors)
            previous_color = -1
            for index in np.argsort(depths):
                color = colors[index]
                if color != previous_color:
                    painter.setBrush(self.colors[color])
                    previous_color = color
                self._triangle_points[:] = faces[index, :, :2]
                painter.drawPolygon(self._triangle)
