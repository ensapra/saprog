"""Perspective-projected north arrow for the IMU canvas renderer."""

import math

from saprog_desktop.desktop_style import ACCENT_LIGHT


def draw_compass(canvas, camera_point, project, center, radius):
    """Draw a flat, fixed north arrow on the ground plane."""
    center_x, center_y, ground_z = center

    def screen(point):
        return project(camera_point(point))

    # Magnetic north is world +X. The marker stays fixed to the ground while
    # the robot rotates against it.
    shaft_start = (center_x - radius * 0.5, center_y, ground_z + 0.002)
    shaft_end = (center_x + radius * 0.5, center_y, ground_z + 0.002)
    canvas.create_line(
        *screen(shaft_start), *screen(shaft_end),
        fill=ACCENT_LIGHT,
        width=2,
        arrow="last",
    )
