#!/usr/bin/env python3
"""Live, selectable servo temperature traces on a shared Tk canvas."""
from collections import deque
import colorsys
import math
import signal
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Temperature

from desktop_style import BACKGROUND, FOREGROUND, apply_style


HISTORY_SECONDS = 300.0
STALE_SECONDS = 2.5
LEG_ROWS = (
    ("Front Right Leg", ("servo_11", "servo_12", "servo_13")),
    ("Back Right Leg", ("servo_21", "servo_22", "servo_23")),
    ("Front Left Leg", ("servo_31", "servo_32", "servo_33")),
    ("Back Left Leg", ("servo_41", "servo_42", "servo_43")),
)
SERVO_CELL = {name: (row, column + 1)
              for row, (_leg, names) in enumerate(LEG_ROWS)
              for column, name in enumerate(names)}


class TemperatureMonitor:
    def __init__(self, root, node):
        self.root, self.node = root, node
        self.history = {}
        self.selected = {}
        self.labels = {}
        self.colors = {}
        self.select_new = True
        self.closed = False
        self.window_seconds = HISTORY_SECONDS
        apply_style(root)
        root.title("Temperature Monitor")
        root.geometry("1050x700")
        root.minsize(700, 450)
        outer = ttk.Frame(root, padding=14)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="Servo temperatures", style="Title.TLabel").pack(anchor="w")
        ttk.Label(outer, text="Live °C · last 5 minutes · toggle any combination of servos",
                  style="Muted.TLabel").pack(anchor="w", pady=(2, 10))
        controls = ttk.Frame(outer)
        controls.pack(fill="x")
        ttk.Button(controls, text="Select all", command=lambda: self.select_all(True)).pack(side="left")
        ttk.Button(controls, text="Deselect all", command=lambda: self.select_all(False)).pack(side="left", padx=6)
        self.status = tk.StringVar(value="Waiting for live temperature readings…")
        ttk.Label(controls, textvariable=self.status).pack(side="left", padx=10)
        zoom = ttk.Frame(outer)
        zoom.pack(fill="x", pady=(8, 0))
        ttk.Label(zoom, text="Time window (seconds)").pack(side="left")
        self.window_input = tk.StringVar(value="300")
        window = ttk.Combobox(zoom, textvariable=self.window_input, width=8,
                              values=(5, 10, 20, 30, 60, 120, 300))
        window.pack(side="left", padx=6)
        window.bind("<<ComboboxSelected>>", self.apply_window)
        window.bind("<Return>", self.apply_window)
        ttk.Button(zoom, text="Apply", command=self.apply_window).pack(side="left")
        ttk.Button(zoom, text="Zoom in", command=lambda: self.zoom(0.5)).pack(side="left", padx=6)
        ttk.Button(zoom, text="Zoom out", command=lambda: self.zoom(2)).pack(side="left")
        self.window_hint = tk.StringVar(value="1–300 seconds; full history is kept when zooming")
        ttk.Label(zoom, textvariable=self.window_hint, style="Muted.TLabel").pack(side="left", padx=8)
        self.choices = ttk.Frame(outer)
        self.choices.pack(fill="x", pady=8)
        self.extra_servos = 0
        for row, (leg, _names) in enumerate(LEG_ROWS):
            ttk.Label(self.choices, text=leg, style="Muted.TLabel").grid(
                row=row, column=0, sticky="w", padx=(0, 16))
        self.canvas = tk.Canvas(outer, background=BACKGROUND, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.draw())
        self.subscriptions = [
            node.create_subscription(Temperature, "/saprog/temperatures", self.temperature, 100),
            node.create_subscription(JointState, "/saprog/joint_states", self.joints, 10),
        ]
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(50, self.poll)

    def add_servo(self, name):
        if name in self.history:
            return
        self.history[name] = deque(maxlen=18000)
        self.selected[name] = tk.BooleanVar(value=self.select_new)
        rgb = colorsys.hsv_to_rgb((len(self.colors) * 0.61803398875) % 1, 0.6, 1.0)
        self.colors[name] = "#" + "".join(f"{round(value * 255):02x}" for value in rgb)
        cell = ttk.Frame(self.choices)
        if name in SERVO_CELL:
            row, column = SERVO_CELL[name]
        else:
            row, column = len(LEG_ROWS) + self.extra_servos // 3, self.extra_servos % 3 + 1
            self.extra_servos += 1
        cell.grid(row=row, column=column, sticky="w", padx=(0, 16))
        ttk.Checkbutton(cell, text=name, variable=self.selected[name], command=self.draw).pack(side="left")
        label = ttk.Label(cell, text="waiting", foreground=self.colors[name])
        label.pack(side="left", padx=4)
        self.labels[name] = label

    def joints(self, message):
        for name in message.name:
            self.add_servo(name)

    def temperature(self, message):
        name = message.header.frame_id
        if not name or not math.isfinite(message.temperature):
            return
        self.add_servo(name)
        self.history[name].append((time.monotonic(), message.temperature))

    def select_all(self, selected):
        self.select_new = selected
        for variable in self.selected.values():
            variable.set(selected)
        self.draw()

    def apply_window(self, _event=None):
        try:
            seconds = float(self.window_input.get())
        except ValueError:
            seconds = math.nan
        if not math.isfinite(seconds) or not 1 <= seconds <= HISTORY_SECONDS:
            self.window_hint.set("Enter a number from 1 to 300 seconds")
            return
        self.window_seconds = seconds
        self.window_input.set(f"{seconds:g}")
        self.window_hint.set("1–300 seconds; full history is kept when zooming")
        self.draw()

    def zoom(self, factor):
        self.window_input.set(f"{min(HISTORY_SECONDS, max(1, self.window_seconds * factor)):g}")
        self.apply_window()

    def draw(self):
        canvas = self.canvas
        canvas.delete("all")
        now = time.monotonic()
        cutoff = now - self.window_seconds
        visible = []
        fresh = 0
        for name, points in self.history.items():
            while points and points[0][0] < now - HISTORY_SECONDS:
                points.popleft()
            recent = bool(points) and now - points[-1][0] <= STALE_SECONDS
            fresh += int(recent)
            self.labels[name].configure(text=(f"{points[-1][1]:.0f} °C" + (" · stale" if not recent else ""))
                                        if points else "waiting")
            if self.selected[name].get():
                visible.append((name, [(stamp, value) for stamp, value in points if stamp >= cutoff]))
        self.status.set(f"{len(visible)}/{len(self.history)} selected · {fresh} live")
        left, top = 65, 25
        right, bottom = max(left + 1, canvas.winfo_width() - 20), max(top + 1, canvas.winfo_height() - 45)
        values = [value for _name, points in visible for _stamp, value in points]
        low = math.floor((min(values) - 5) / 5) * 5 if values else 15
        high = max(low + 10, math.ceil((max(values) + 5) / 5) * 5) if values else 50
        for index in range(6):
            y = bottom - index * (bottom - top) / 5
            canvas.create_line(left, y, right, y, fill="#2b394d")
            canvas.create_text(left - 10, y, text=f"{low + index * (high - low) / 5:.0f}", anchor="e", fill=FOREGROUND)
            x = left + index * (right - left) / 5
            canvas.create_text(x, bottom + 18, text=f"{-self.window_seconds + index * self.window_seconds / 5:g}s" if index < 5 else "now", fill=FOREGROUND)
        canvas.create_text(10, top, text="°C", anchor="w", fill=FOREGROUND)
        for name, points in visible:
            segment = []
            previous = None
            for stamp, value in points:
                if previous is not None and stamp - previous > STALE_SECONDS:
                    self.draw_segment(segment, self.colors[name])
                    segment = []
                segment.extend((left + (stamp - cutoff) / self.window_seconds * (right - left),
                                bottom - (value - low) / (high - low) * (bottom - top)))
                previous = stamp
            self.draw_segment(segment, self.colors[name])
        if not values:
            canvas.create_text((left + right) / 2, (top + bottom) / 2,
                               text="Waiting for selected servo temperatures" if visible else "Select servos to graph",
                               fill=FOREGROUND)

    def draw_segment(self, coordinates, color):
        if len(coordinates) >= 4:
            self.canvas.create_line(*coordinates, fill=color, width=2)
        elif coordinates:
            x, y = coordinates
            self.canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline=color)

    def poll(self):
        if self.closed:
            return
        for _ in range(100):
            rclpy.spin_once(self.node, timeout_sec=0)
        self.draw()
        self.root.after(100, self.poll)

    def close(self):
        if not self.closed:
            self.closed = True
            self.root.destroy()


def main():
    rclpy.init()
    node = Node("saprog_temperature_monitor")
    try:
        root = tk.Tk()
        app = TemperatureMonitor(root, node)
        signal.signal(signal.SIGTERM, lambda *_args: app.close())
        signal.signal(signal.SIGINT, lambda *_args: app.close())
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
