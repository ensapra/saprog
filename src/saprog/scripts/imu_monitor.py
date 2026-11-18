#!/usr/bin/env python3
"""Tk display and ROS service client; no serial I/O or sensor filtering."""
from collections import deque
from datetime import datetime
import math
import signal
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import QuaternionStamped
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from saprog.msg import ImuStatus
from desktop_style import apply_style


class ImuMonitor:
    def __init__(self, root, node, embedded=False):
        self.root = root
        self.embedded = embedded
        if not embedded:
            self.root.title("9DoF IMU")
            self.root.geometry("960x850")
            self.root.minsize(700, 650)
            apply_style(root)

        self.node = node
        self.closed = False
        self.connected = False
        self.orientation = None
        self.zeroed = False
        self.backend_status = None
        self.last_status_at = 0.0
        self.latest_raw = None
        self.latest_mag = None
        self.logs = deque(maxlen=30)
        self.requests = []
        self.clients = {}

        body = ttk.Frame(root, padding=16)
        body.pack(fill="both", expand=True)
        ttk.Label(body, text="9DoF IMU", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            body,
            text="ROS IMU monitor · sensor processing runs in the C++ backend",
            style="Muted.TLabel",
        ).pack(anchor="w", pady=(2, 12))

        actions = ttk.Frame(body)
        actions.pack(fill="x", pady=(0, 10))
        self.toggle_button = ttk.Button(
            actions, text="Toggle readings (Space)", command=self.toggle_readings,
            state="disabled",
        )
        self.toggle_button.pack(side="left")
        self.status = tk.StringVar(value="Disconnected")
        ttk.Label(actions, textvariable=self.status, style="Muted.TLabel").pack(
            side="left", padx=(12, 0)
        )

        readings = ttk.LabelFrame(body, text="Latest sample", padding=10)
        readings.pack(fill="x", pady=(0, 10))
        self.sample_status = tk.StringVar(value="Waiting for connection")
        ttk.Label(readings, textvariable=self.sample_status, style="Muted.TLabel", wraplength=820).grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(0, 6)
        )
        self.values = {}
        for column, name in enumerate(("Time (ms)", "Accel (m/s²)", "Gyro (rad/s)", "Mag (µT)")):
            ttk.Label(readings, text=name, style="Muted.TLabel").grid(
                row=1, column=column, sticky="w", padx=(0, 12)
            )
            value = tk.StringVar(value="—")
            self.values[name] = value
            ttk.Label(readings, textvariable=value).grid(
                row=2, column=column, sticky="w", padx=(0, 12)
            )

        model_frame = ttk.LabelFrame(body, text="3D orientation", padding=8)
        model_frame.pack(fill="both", expand=True, pady=(0, 10))
        reference_controls = ttk.Frame(model_frame)
        reference_controls.pack(fill="x", pady=(0, 8))
        self.zero_button = ttk.Button(
            reference_controls, text="Zero orientation", command=self.zero_orientation,
            state="disabled",
        )
        self.zero_button.pack(side="left")
        self.use_magnetometer = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            reference_controls, text="Use magnetic heading", variable=self.use_magnetometer,
            command=self.reset_orientation,
        ).pack(side="left", padx=8)
        ttk.Button(
            reference_controls, text="Clear zero", command=self.clear_zero,
        ).pack(side="left", padx=8)
        self.reference_status = tk.StringVar(value="Place the IMU flat and still, then click Zero orientation.")
        ttk.Label(model_frame, textvariable=self.reference_status, wraplength=650).pack(anchor="w")
        calibration_controls = ttk.Frame(model_frame)
        calibration_controls.pack(fill="x", pady=(6, 6))
        self.calibrate_button = ttk.Button(
            calibration_controls, text="Calibrate gyro + zero", command=self.start_calibration,
            state="disabled",
        )
        self.calibrate_button.pack(side="left")
        self.calibration_status = tk.StringVar(value="Hold flat and still for 4 seconds. Requires accel in g and gyro in degrees/s.")
        ttk.Label(calibration_controls, textvariable=self.calibration_status, wraplength=450).pack(side="left", padx=8)
        self.model = tk.Canvas(
            model_frame, height=300, background="#1d2939", highlightthickness=0
        )
        self.model.pack(fill="both", expand=True)
        self.model.bind("<Configure>", lambda _event: self.render_model())

        log_frame = ttk.LabelFrame(body, text="Serial output", padding=8)
        log_frame.pack(fill="x")
        self.log = tk.Text(log_frame, height=5, wrap="none", state="disabled")
        self.log.pack(side="left", fill="both", expand=True)
        scrollbar = ttk.Scrollbar(log_frame, orient="vertical", command=self.log.yview)
        scrollbar.pack(side="right", fill="y")
        self.log.configure(yscrollcommand=scrollbar.set)

        if not embedded:
            root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(50, self.poll_messages)
        self.render_model()

        for name in ("toggle_readings", "zero", "clear_zero", "calibrate"):
            self.clients[name] = node.create_client(Trigger, "/saprog/imu/" + name)
        self.clients["use_magnetometer"] = node.create_client(SetBool, "/saprog/imu/use_magnetometer")
        self.subscriptions = [
            node.create_subscription(ImuStatus, "/saprog/imu/status", self.receive_status, 10),
            node.create_subscription(Imu, "/saprog/imu/data_raw", self.receive_raw, qos_profile_sensor_data),
            node.create_subscription(MagneticField, "/saprog/imu/mag", self.receive_mag, qos_profile_sensor_data),
            node.create_subscription(QuaternionStamped, "/saprog/imu/orientation", self.receive_orientation, qos_profile_sensor_data),
            node.create_subscription(String, "/saprog/imu/serial", lambda msg: self.logs.append(msg.data), qos_profile_sensor_data),
        ]

    def append_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", f"{datetime.now().strftime('%H:%M:%S')}  {text}\n")
        lines = int(self.log.index("end-1c").split(".")[0])
        if lines > 301:
            self.log.delete("1.0", f"{lines - 300}.0")
        self.log.see("end")
        self.log.configure(state="disabled")


    def render_model(self):
        if not hasattr(self, "model"):
            return
        canvas = self.model
        canvas.delete("all")
        width = max(canvas.winfo_width(), 280)
        height = max(canvas.winfo_height(), 220)
        center_x, center_y = width / 2, height / 2 - 12
        scale = min(width, height - 70) * 0.19
        rotation = self.orientation_matrix(self.orientation or (0.0, 0.0, 0.0))
        pitch = math.asin(max(-1.0, min(1.0, -rotation[2][0])))
        if abs(math.cos(pitch)) > 1e-6:
            roll = math.atan2(rotation[2][1], rotation[2][2])
            yaw = math.atan2(rotation[1][0], rotation[0][0])
        else:
            roll = 0.0
            yaw = math.atan2(-rotation[0][1], rotation[1][1])

        # Look down from above the XY floor with world Z pointing up on screen.
        # Turn around world Z before pitching the camera, keeping the horizon level.
        camera_heading = self.orientation_matrix((0.0, 0.0, math.radians(-25)))
        camera_pitch = self.orientation_matrix((math.radians(-55), 0.0, 0.0))

        def transform(matrix, point):
            return tuple(sum(row[i] * point[i] for i in range(3)) for row in matrix)

        def camera_point(point):
            return transform(camera_pitch, transform(camera_heading, point))

        world_vertices = [
            transform(rotation, point)
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
            return (
                center_x + x * scale * focal / depth,
                center_y - y * scale * focal / depth,
            )

        def ground_point(x, y):
            return project(camera_point((x, y, -1.65)))

        # Fixed Z-up reference plane below the box's full rotation envelope.
        # Height is illustrative: the IMU estimates orientation, not altitude.
        ground = [ground_point(x, y) for x, y in
                  ((-2.5, -2.5), (2.5, -2.5), (2.5, 2.5), (-2.5, 2.5))]
        canvas.create_polygon(
            *[coordinate for point in ground for coordinate in point],
            fill="#26384a", outline="#64748b", width=2,
        )
        for step in range(-5, 6):
            offset = step * 0.5
            color = "#71869c" if step == 0 else "#3b5065"
            canvas.create_line(*ground_point(offset, -2.5), *ground_point(offset, 2.5), fill=color)
            canvas.create_line(*ground_point(-2.5, offset), *ground_point(2.5, offset), fill=color)

        # Overhead light: the shadow is the convex hull of all eight XY corners.
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
        canvas.create_polygon(
            *[coordinate for x, y in shadow for coordinate in ground_point(x, y)],
            fill="#172431", outline="",
        )
        canvas.create_text(
            14, 36, anchor="nw", fill="#a5b5ca",
            text="Ground reference · shadow shows tilt; height is illustrative",
            font=("TkDefaultFont", 9),
        )
        projected = [project(point) for point in vertices]

        faces = [
            ((0, 1, 2, 3), "#315276"), ((4, 5, 6, 7), "#4b8ac2"),
            ((0, 1, 5, 4), "#3a6996"), ((1, 2, 6, 5), "#27618d"),
            ((2, 3, 7, 6), "#254b70"), ((3, 0, 4, 7), "#3f78a8"),
        ]
        faces.sort(key=lambda item: sum(vertices[index][2] for index in item[0]) / 4)
        for indices, color in faces:
            coords = [coordinate for index in indices for coordinate in projected[index]]
            canvas.create_polygon(coords, fill=color, outline="#a8d8ff", width=2)

        canvas.create_text(
            14, 14, anchor="nw", fill="#d6e7f7",
            text=(
                f"Roll {math.degrees(roll):6.1f}°   "
                f"Pitch {math.degrees(pitch):6.1f}°   "
                f"Yaw {math.degrees(yaw):6.1f}°"
            ),
            font=("TkDefaultFont", 10, "bold"),
        )
        state = (
            "Waiting for IMU samples" if self.orientation is None
            else "Orientation relative to your zero pose" if self.zeroed
            else "Magnetic heading enabled" if self.use_magnetometer.get()
            else "Accelerometer + gyroscope · relative heading (may drift)"
        )
        canvas.create_text(
            14, height - 14, anchor="sw", fill="#a5b5ca", text=state,
            font=("TkDefaultFont", 9),
        )


    @staticmethod
    def orientation_matrix(angles):
        roll, pitch, yaw = angles
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return (
            (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
            (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
            (-sp, cp * sr, cp * cr),
        )


    def request(self, name, request=None):
        client = self.clients[name]
        if not client.service_is_ready():
            self.status.set("IMU ROS service unavailable; start the C++ IMU backend.")
            return
        future = client.call_async(request if request is not None else Trigger.Request())
        self.requests.append((future, time.monotonic() + 5, client))
        self.status.set("Request sent: " + name)

    def toggle_readings(self):
        self.request("toggle_readings")

    def zero_orientation(self):
        self.request("zero")

    def clear_zero(self):
        self.request("clear_zero")

    def start_calibration(self):
        self.request("calibrate")

    def reset_orientation(self):
        request = SetBool.Request()
        request.data = self.use_magnetometer.get()
        self.request("use_magnetometer", request)

    def receive_raw(self, msg):
        self.latest_raw = msg

    def receive_mag(self, msg):
        self.latest_mag = msg

    def receive_orientation(self, msg):
        q = msg.quaternion
        # Display conversion only; fusion and zero transforms happen in C++.
        self.orientation = [
            math.atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2 * (q.x*q.x + q.y*q.y)),
            math.asin(max(-1.0, min(1.0, 2 * (q.w*q.y - q.z*q.x)))),
            math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z)),
        ]

    def receive_status(self, msg):
        self.last_status_at = time.monotonic()
        self.backend_status = msg
        if msg.connected and not self.connected:
        self.connected = msg.connected
        self.zeroed = msg.zeroed
        self.use_magnetometer.set(msg.magnetic_heading)
        self.status.set(msg.message)
        self.calibration_status.set(msg.calibration_message)
        self.reference_status.set(
            "Zero saved in ROS; this display shows orientation relative to it."
            if msg.zeroed else "Place the IMU flat and still, then click Zero orientation."
        )
        self.values["Time (ms)"].set(f"{msg.sensor_time_ms:.0f}")
        state = "live" if msg.streaming else "waiting / stale"
        self.sample_status.set(
            f"{msg.sample_count} samples · {state} · {msg.sample_hz:.1f} Hz · "
            f"{msg.time_gaps} timestamp gaps · peak gyro {msg.peak_gyro_dps:.1f} °/s · "
            f"{msg.invalid_lines} rejected lines"
        )
        if not msg.orientation_valid:
            self.orientation = None

    @staticmethod
    def format_vector(vector, scale=1.0):
        return ", ".join(f"{value * scale:.3f}" for value in (vector.x, vector.y, vector.z))

    def poll_messages(self):
        if self.closed:
            return
        if not rclpy.ok():
            self.close()
            return
        deadline = time.monotonic() + 0.01
        if not self.embedded:
            for _ in range(150):
                rclpy.spin_once(self.node, timeout_sec=0)
                if time.monotonic() >= deadline:
                    break
        for future, timeout, client in list(self.requests):
            if future.done():
                self.requests.remove((future, timeout, client))
                try:
                    result = future.result()
                    self.status.set(result.message)
                    self.append_log(("OK: " if result.success else "Failed: ") + result.message)
                except Exception as error:
                    self.status.set(f"ROS request failed: {error}")
            elif time.monotonic() >= timeout:
                client.remove_pending_request(future)
                future.cancel()
                self.requests.remove((future, timeout, client))
                self.status.set("ROS request timed out; check the backend status before retrying.")
        if self.latest_raw is not None:
            self.values["Accel (m/s²)"].set(self.format_vector(self.latest_raw.linear_acceleration))
            self.values["Gyro (rad/s)"].set(self.format_vector(self.latest_raw.angular_velocity))
            self.latest_raw = None
        if self.latest_mag is not None:
            self.values["Mag (µT)"].set(self.format_vector(self.latest_mag.magnetic_field, 1e6))
            self.latest_mag = None
        if self.logs:
            self.append_log("\n".join(self.logs))
            self.logs.clear()
        available = time.monotonic() - self.last_status_at < 2 and self.backend_status is not None
        valid = available and self.backend_status.orientation_valid
        idle = not self.requests
        can_zero = valid and not self.backend_status.calibrating and idle
        self.toggle_button.configure(state="normal" if available and self.connected and idle else "disabled")
        self.zero_button.configure(state="normal" if can_zero else "disabled")
        self.calibrate_button.configure(state="normal" if can_zero else "disabled")
        if not available:
            self.orientation = None
            self.status.set("Waiting for /saprog/imu/status from the C++ ROS backend…")
        self.render_model()
        self.root.after(50, self.poll_messages)

    def close(self):
        if self.closed:
            return
        self.closed = True
        if not self.embedded:
            self.node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
            self.root.destroy()


def main():
    rclpy.init()
    root = tk.Tk()
    app = ImuMonitor(root, Node("saprog_imu_monitor"))
    signal.signal(signal.SIGTERM, lambda *_args: app.close())
    signal.signal(signal.SIGINT, lambda *_args: app.close())
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
