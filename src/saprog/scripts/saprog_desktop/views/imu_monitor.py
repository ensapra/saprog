#!/usr/bin/env python3
"""Tk display and ROS service client; no serial I/O or sensor filtering."""

if __name__ == "__main__":
    from saprog_desktop.startup import imu_monitor_main

    raise SystemExit(imu_monitor_main(entrypoint=__file__))

import time
import tkinter as tk
from collections import deque
from datetime import datetime
from tkinter import ttk

from saprog_desktop.desktop_style import PANEL, apply_style, configure_changed
from saprog_desktop.imu_model import quaternion_to_euler


class ImuMonitor:
    def __init__(self, root, node, embedded=False, dependencies=None):
        if dependencies is None:
            raise ValueError("ImuMonitor requires injected desktop runtime dependencies")
        self.root = root
        self.embedded = embedded
        self.dependencies = dependencies
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
            actions,
            text="Toggle readings (Space)",
            command=self.toggle_readings,
            state="disabled",
        )
        self.toggle_button.pack(side="left")
        self.status = tk.StringVar(value="Disconnected")
        self.status_label = ttk.Label(
            actions,
            textvariable=self.status,
            style="Muted.TLabel",
            anchor="w",
            justify="left",
        )
        self.status_label.pack(side="left", fill="x", expand=True, padx=(12, 0))
        actions.bind("<Configure>", self._fit_status)

        readings = ttk.LabelFrame(body, text="Latest sample", padding=10)
        readings.pack(fill="x", pady=(0, 10))
        self.sample_status = tk.StringVar(value="Waiting for connection")
        ttk.Label(
            readings, textvariable=self.sample_status, style="Muted.TLabel", wraplength=820
        ).grid(row=0, column=0, columnspan=4, sticky="w", pady=(0, 6))
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
            reference_controls,
            text="Zero orientation",
            command=self.zero_orientation,
            state="disabled",
        )
        self.zero_button.pack(side="left")
        self.use_magnetometer = tk.BooleanVar(value=False)
        ttk.Checkbutton(
            reference_controls,
            text="Use magnetic heading",
            variable=self.use_magnetometer,
            command=self.reset_orientation,
        ).pack(side="left", padx=8)
        ttk.Button(
            reference_controls,
            text="Clear zero",
            command=self.clear_zero,
        ).pack(side="left", padx=8)
        self.reference_status = tk.StringVar(
            value="Place the IMU flat and still, then click Zero orientation."
        )
        ttk.Label(model_frame, textvariable=self.reference_status, wraplength=650).pack(anchor="w")
        calibration_controls = ttk.Frame(model_frame)
        calibration_controls.pack(fill="x", pady=(6, 6))
        self.calibrate_button = ttk.Button(
            calibration_controls,
            text="Calibrate gyro + zero",
            command=self.start_calibration,
            state="disabled",
        )
        self.calibrate_button.pack(side="left")
        self.calibration_status = tk.StringVar(
            value="Hold flat and still for 4 seconds. Requires accel in g and gyro in degrees/s."
        )
        ttk.Label(calibration_controls, textvariable=self.calibration_status, wraplength=450).pack(
            side="left", padx=8
        )
        self.model = tk.Canvas(model_frame, height=300, background=PANEL, highlightthickness=0)
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

        for name in (
            "toggle_readings",
            "zero",
            "clear_zero",
            "calibrate",
        ):
            self.clients[name] = node.create_client(
                dependencies.Trigger, "/saprog/imu/" + name
            )
        self.clients["use_magnetometer"] = node.create_client(
            dependencies.SetBool, "/saprog/imu/use_magnetometer"
        )
        self.subscriptions = [
            node.create_subscription(
                dependencies.ImuStatus, "/saprog/imu/status", self.receive_status, 10
            ),
            node.create_subscription(
                dependencies.Imu,
                "/saprog/imu/data_raw",
                self.receive_raw,
                dependencies.qos_profile_sensor_data,
            ),
            node.create_subscription(
                dependencies.MagneticField,
                "/saprog/imu/mag",
                self.receive_mag,
                dependencies.qos_profile_sensor_data,
            ),
            node.create_subscription(
                dependencies.QuaternionStamped,
                "/saprog/imu/orientation",
                self.receive_orientation,
                dependencies.qos_profile_sensor_data,
            ),
            node.create_subscription(
                dependencies.String,
                "/saprog/imu/serial",
                self.receive_log,
                dependencies.qos_profile_sensor_data,
            ),
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
        from saprog_desktop.imu_view import draw_orientation

        if hasattr(self, "model") and not self.closed:
            draw_orientation(
                self.model, self.orientation, self.zeroed,
                self.use_magnetometer.get(),
            )
    def request(self, name, request=None):
        if self.closed:
            return
        client = self.clients[name]
        if not client.service_is_ready():
            self.status.set("IMU ROS service unavailable; start the C++ IMU backend.")
            return
        future = client.call_async(
            request if request is not None else self.dependencies.Trigger.Request()
        )
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
        request = self.dependencies.SetBool.Request()
        request.data = self.use_magnetometer.get()
        self.request("use_magnetometer", request)

    def receive_raw(self, msg):
        if self.closed:
            return
        self.latest_raw = msg

    def receive_log(self, msg):
        if not self.closed:
            self.logs.append(msg.data)

    def receive_mag(self, msg):
        if self.closed:
            return
        self.latest_mag = msg

    def receive_orientation(self, msg):
        if self.closed:
            return
        q = msg.quaternion
        # Display conversion only; fusion and zero transforms happen in C++.
        self.orientation = quaternion_to_euler((q.x, q.y, q.z, q.w))

    def receive_status(self, msg):
        if self.closed:
            return
        self.last_status_at = time.monotonic()
        self.backend_status = msg
        self.connected = msg.connected
        self.zeroed = msg.zeroed
        self.use_magnetometer.set(msg.magnetic_heading)
        self.status.set(msg.message)
        self.calibration_status.set(msg.calibration_message)
        self.reference_status.set(
            "Zero saved in ROS; this display shows orientation relative to it."
            if msg.zeroed
            else "Place the IMU flat and still, then click Zero orientation."
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
        configure_changed(
            self.toggle_button,
            state="normal" if available and self.connected and idle else "disabled",
        )
        configure_changed(self.zero_button, state="normal" if can_zero else "disabled")
        configure_changed(self.calibrate_button, state="normal" if can_zero else "disabled")
        if not available:
            self.orientation = None
            self.status.set("Waiting for /saprog/imu/status from the C++ ROS backend…")
        self.render_model()
        self.root.after(50, self.poll_messages)

    def _fit_status(self, event):
        available = event.width - self.toggle_button.winfo_reqwidth() - 24
        self.status_label.configure(wraplength=max(200, available))

    def close(self):
        if self.closed:
            return
        self.closed = True
        for future, _timeout, client in self.requests:
            client.remove_pending_request(future)
            future.cancel()
        self.requests.clear()
