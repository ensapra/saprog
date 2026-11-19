"""Canvas drawing for the sensor-relative IMU orientation display."""

import math

from saprog_desktop.desktop_style import (
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
from saprog_desktop.imu_model import orientation_matrix


def draw_orientation(canvas, orientation, zeroed=False, magnetic_heading=False):
    """Draw the projected IMU and fixed ground reference onto a Tk canvas."""
    canvas.delete("all")
    width = max(canvas.winfo_width(), 280)
    height = max(canvas.winfo_height(), 220)
    center_x, center_y = width / 2, height / 2 - 12
    scale = min(width, height - 70) * 0.19
    rotation = orientation_matrix(orientation or (0.0, 0.0, 0.0))
    pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
    if abs(math.cos(pitch)) > 1e-6:
        roll = math.atan2(rotation[2][1], rotation[2][2])
        yaw = math.atan2(rotation[1][0], rotation[0][0])
    else:
        roll, yaw = 0.0, math.atan2(-rotation[0][1], rotation[1][1])

    camera_heading = orientation_matrix((0.0, 0.0, math.radians(-25)))
    camera_pitch = orientation_matrix((math.radians(-55), 0.0, 0.0))

    def transform(matrix, point):
        return tuple(sum(row[i] * point[i] for i in range(3)) for row in matrix)

    def camera_point(point):
        return transform(camera_pitch, transform(camera_heading, point))

    # The physical IMU is mounted 90 degrees around its vertical Y axis.
    # Apply that mounting rotation to the box, while keeping the sensor's
    # reported roll/pitch values and the ground reference in their own frame.
    mounting_rotation = orientation_matrix((0.0, math.pi / 2, 0.0))
    world_vertices = [
        transform(rotation, transform(mounting_rotation, point))
        for point in (
            (-1.25, -0.85, -0.45), (1.25, -0.85, -0.45),
            (1.25, 0.85, -0.45), (-1.25, 0.85, -0.45),
            (-1.25, -0.85, 0.45), (1.25, -0.85, 0.45),
            (1.25, 0.85, 0.45), (-1.25, 0.85, 0.45),
        )
    ]
    vertices = [camera_point(point) for point in world_vertices]
    focal = 7.0

    def project(point):
        x, y, z = point
        depth = max(1.5, focal - z)
        return (center_x + x * scale * focal / depth, center_y - y * scale * focal / depth)

    def ground_point(x, y):
        return project(camera_point((x, y, -1.65)))

    ground = [ground_point(x, y) for x, y in ((-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5))]
    canvas.create_polygon(
        *[c for point in ground for c in point],
        fill=ROBOT_GROUND,
        outline=ROBOT_GROUND_EDGE,
        width=2,
    )
    for step in range(-5, 6):
        offset = step * 0.5
        color = ROBOT_GROUND_AXIS if step == 0 else ROBOT_GROUND_GRID
        canvas.create_line(*ground_point(offset, -2.5), *ground_point(offset, 2.5), fill=color)
        canvas.create_line(*ground_point(-2.5, offset), *ground_point(2.5, offset), fill=color)

    points = sorted(set((x, y) for x, y, _z in world_vertices))

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    def hull_half(sequence):
        hull = []
        for point in sequence:
            while len(hull) >= 2 and cross(hull[-2], hull[-1], point) <= 0:
                hull.pop()
            hull.append(point)
        return hull

    shadow = hull_half(points)[:-1] + hull_half(reversed(points))[:-1]
    canvas.create_polygon(*[c for x, y in shadow for c in ground_point(x, y)], fill=ROBOT_SHADOW, outline="")
    canvas.create_text(
        14,
        36,
        anchor="nw",
        fill=MUTED,
        text="Ground reference · shadow shows tilt; height is illustrative",
        font=("TkDefaultFont", 9),
    )
    projected = [project(point) for point in vertices]
    faces = [
        ((0, 1, 2, 3), ROBOT_FACE_FRONT), ((4, 5, 6, 7), ROBOT_FACE_TOP),
        ((0, 1, 5, 4), ROBOT_FACE_LEFT), ((1, 2, 6, 5), ROBOT_FACE_RIGHT),
        ((2, 3, 7, 6), ROBOT_FACE_BOTTOM), ((3, 0, 4, 7), ROBOT_FACE_BACK),
    ]
    faces.sort(key=lambda item: sum(vertices[i][2] for i in item[0]) / 4)
    for indices, color in faces:
        canvas.create_polygon(
            [c for i in indices for c in projected[i]],
            fill=color,
            outline=ROBOT_EDGE,
            width=2,
        )

    canvas.create_text(
        14,
        14,
        anchor="nw",
        fill=ROBOT_TEXT,
        text=(
            f"Roll {math.degrees(roll):6.1f}°   Pitch {math.degrees(pitch):6.1f}°   "
            f"Yaw {math.degrees(yaw):6.1f}°"
        ),
        font=("TkDefaultFont", 10, "bold"),
    )
    state = "Waiting for IMU samples" if orientation is None else (
        "Orientation relative to your zero pose" if zeroed else "Magnetic heading enabled"
        if magnetic_heading else "Accelerometer + gyroscope · relative heading (may drift)"
    )
    canvas.create_text(
        14, height - 14, anchor="sw", fill=MUTED, text=state, font=("TkDefaultFont", 9)
    )
