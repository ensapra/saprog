#!/usr/bin/env python3
"""Live, selectable servo temperature traces on a shared Tk canvas."""

if __name__ == "__main__":
    from saprog_desktop.startup import temperature_monitor_main

    raise SystemExit(temperature_monitor_main(entrypoint=__file__))

import colorsys
import math
import time
import tkinter as tk
from tkinter import ttk

from saprog_desktop.desktop_style import BACKGROUND, apply_style
from saprog_desktop.temperature_model import TemperatureHistory
from saprog_desktop.temperature_view import draw_temperature

LEG_ROWS = (
    ("Front Right Leg", ("servo_11", "servo_12", "servo_13")),
    ("Back Right Leg", ("servo_21", "servo_22", "servo_23")),
    ("Front Left Leg", ("servo_31", "servo_32", "servo_33")),
    ("Back Left Leg", ("servo_41", "servo_42", "servo_43")),
)
SERVO_CELL = {
    name: (row, column + 1)
    for row, (_leg, names) in enumerate(LEG_ROWS)
    for column, name in enumerate(names)
}


class TemperatureMonitor:
    def __init__(self, parent, node, dependencies):
        self.parent, self.root, self.node = parent, parent.winfo_toplevel(), node
        self.dependencies = dependencies
        self.history = TemperatureHistory()
        self.selected = {}
        self.labels = {}
        self.colors = {}
        self.select_new = True
        self.closed = False
        outer = ttk.Frame(parent, padding=14)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="Servo temperatures", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            outer,
            text="Live °C · last 5 minutes · toggle any combination of servos",
            style="Muted.TLabel",
        ).pack(anchor="w", pady=(2, 10))
        controls = ttk.Frame(outer)
        controls.pack(fill="x")
        ttk.Button(controls, text="Select all", command=lambda: self.select_all(True)).pack(
            side="left"
        )
        ttk.Button(controls, text="Deselect all", command=lambda: self.select_all(False)).pack(
            side="left", padx=6
        )
        self.status = tk.StringVar(value="Waiting for live temperature readings…")
        ttk.Label(controls, textvariable=self.status).pack(side="left", padx=10)
        zoom = ttk.Frame(outer)
        zoom.pack(fill="x", pady=(8, 0))
        ttk.Label(zoom, text="Time window (seconds)").pack(side="left")
        self.window_input = tk.StringVar(value=str(int(self.history.window_seconds)))
        window = ttk.Combobox(
            zoom, textvariable=self.window_input, width=8, values=(5, 10, 20, 30, 60, 120, 300)
        )
        window.pack(side="left", padx=6)
        window.bind("<<ComboboxSelected>>", self.apply_window)
        window.bind("<Return>", self.apply_window)
        ttk.Button(zoom, text="Apply", command=self.apply_window).pack(side="left")
        ttk.Button(zoom, text="Zoom in", command=lambda: self.zoom(0.5)).pack(side="left", padx=6)
        ttk.Button(zoom, text="Zoom out", command=lambda: self.zoom(2)).pack(side="left")
        self.window_hint = tk.StringVar(value="1–300 seconds; full history is kept when zooming")
        ttk.Label(zoom, textvariable=self.window_hint, style="Muted.TLabel").pack(
            side="left", padx=8
        )
        self.choices = ttk.Frame(outer)
        self.choices.pack(fill="x", pady=8)
        self.extra_servos = 0
        for row, (leg, _names) in enumerate(LEG_ROWS):
            ttk.Label(self.choices, text=leg, style="Muted.TLabel").grid(
                row=row, column=0, sticky="w", padx=(0, 16)
            )
        self.canvas = tk.Canvas(outer, background=BACKGROUND, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self.draw())
        self.subscriptions = [
            node.create_subscription(
                dependencies.Temperature, "/saprog/temperatures", self.temperature, 100
            ),
            node.create_subscription(
                dependencies.JointState, "/saprog/joint_states", self.joints, 10
            ),
        ]

    def add_servo(self, name):
        if name in self.history.points:
            return
        self.history.add_servo(name)
        self.selected[name] = tk.BooleanVar(value=name in self.history.selected)
        rgb = colorsys.hsv_to_rgb((len(self.colors) * 0.61803398875) % 1, 0.6, 1.0)
        self.colors[name] = "#" + "".join(f"{round(value * 255):02x}" for value in rgb)
        cell = ttk.Frame(self.choices)
        if name in SERVO_CELL:
            row, column = SERVO_CELL[name]
        else:
            row, column = len(LEG_ROWS) + self.extra_servos // 3, self.extra_servos % 3 + 1
            self.extra_servos += 1
        cell.grid(row=row, column=column, sticky="w", padx=(0, 16))
        ttk.Checkbutton(cell, text=name, variable=self.selected[name], command=self.draw).pack(
            side="left"
        )
        label = ttk.Label(cell, text="waiting", foreground=self.colors[name])
        label.pack(side="left", padx=4)
        self.labels[name] = label

    def joints(self, message):
        if self.closed:
            return
        for name in message.name:
            self.add_servo(name)

    def temperature(self, message):
        if self.closed:
            return
        name = message.header.frame_id
        if not name or not math.isfinite(message.temperature):
            return
        self.add_servo(name)
        self.history.add_sample(name, time.monotonic(), message.temperature)

    def select_all(self, selected):
        self.select_new = selected
        for variable in self.selected.values():
            variable.set(selected)
        self.history.select_all(selected)
        self.draw()

    def apply_window(self, _event=None):
        if not self.history.set_window(self.window_input.get()):
            self.window_hint.set("Enter a number from 1 to 300 seconds")
            return
        seconds = self.history.window_seconds
        self.window_input.set(f"{seconds:g}")
        self.window_hint.set("1–300 seconds; full history is kept when zooming")
        self.draw()

    def zoom(self, factor):
        self.window_input.set(f"{self.history.zoom(factor):g}")
        self.apply_window()

    def draw(self):
        if self.closed:
            return
        for name, variable in self.selected.items():
            if variable.get():
                self.history.selected.add(name)
            else:
                self.history.selected.discard(name)
        self.status.set(draw_temperature(
            self.canvas, self.history, self.colors, self.labels, time.monotonic()
        ))

    def poll(self):
        if self.closed:
            return
        self.draw()
        return True

    def close(self):
        self.closed = True
