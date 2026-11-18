#!/usr/bin/env python3
"""Launch and manage the accelerated RViz2 robot view from Home."""
import os
import signal
import subprocess
import tkinter as tk
from tkinter import ttk


class RobotView:
    def __init__(self, parent, workspace):
        self.root = parent
        self.workspace = workspace
        self.process = None

        ttk.Label(parent, text="Robot visualization", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            parent,
            text="RViz2 renders the URDF meshes with OpenGL and follows live servo and IMU data.",
            style="Muted.TLabel", wraplength=850,
        ).pack(anchor="w", pady=(4, 14))

        controls = ttk.Frame(parent)
        controls.pack(anchor="w")
        self.start_button = ttk.Button(controls, text="Open accelerated 3D view", command=self.start)
        self.start_button.pack(side="left")
        self.stop_button = ttk.Button(controls, text="Close view", command=self.stop, state="disabled")
        self.stop_button.pack(side="left", padx=(8, 0))

        self.status = tk.StringVar(value="The 3D view opens in an RViz2 window.")
        ttk.Label(parent, textvariable=self.status, style="Muted.TLabel", wraplength=850).pack(anchor="w", pady=(12, 0))
        ttk.Label(
            parent,
            text="RViz uses the IMU quaternion for the robot body orientation and the servo readback for leg positions.",
            style="Muted.TLabel", wraplength=850,
        ).pack(anchor="w", pady=(8, 0))
        self.root.after(500, self.poll)

    def start(self):
        if self.process is not None and self.process.poll() is None:
            self.status.set("RViz2 is already open.")
            return
        try:
            self.process = subprocess.Popen(
                ["ros2", "launch", "saprog", "robot_view.launch.py"],
                cwd=str(self.workspace), env=os.environ.copy(), start_new_session=True,
            )
            self.status.set("Starting RViz2 robot visualization…")
            self.start_button.configure(state="disabled")
            self.stop_button.configure(state="normal")
        except OSError as error:
            self.process = None
            self.status.set(f"Could not start RViz2: {error}")

    def stop(self):
        if self.process is None:
            return
        if self.process.poll() is None:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
            except ProcessLookupError:
                pass
        self.process = None
        self.status.set("RViz2 closed.")
        self.start_button.configure(state="normal")
        self.stop_button.configure(state="disabled")

    def poll(self):
        if self.process is not None and self.process.poll() is not None:
            result = self.process.returncode
            self.process = None
            self.status.set("RViz2 closed." if result == 0 else f"RViz2 exited with code {result}; see launcher output.")
            self.start_button.configure(state="normal")
            self.stop_button.configure(state="disabled")
        if self.root.winfo_exists():
            self.root.after(500, self.poll)

    def close(self):
        self.stop()
