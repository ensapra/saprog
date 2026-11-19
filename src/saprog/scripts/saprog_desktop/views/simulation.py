"""Tk controls for the independent Gazebo inspection world."""

import sys
import tkinter as tk
from tkinter import ttk

from saprog_desktop.desktop_style import configure_changed, status_box
from saprog_desktop.simulation_session import SimulationSession


class SimulationView:
    def __init__(self, parent, workspace):
        self.parent = parent
        self.session = SimulationSession(workspace)
        ttk.Label(parent, text="Robot simulation", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            parent, style="Muted.TLabel", wraplength=800,
            text="Open the robot URDF in Gazebo with gravity, a floor, and collision physics. "
                 "Gazebo opens in its own window and starts paused.",
        ).pack(anchor="w", pady=(4, 12))
        controls = ttk.Frame(parent)
        controls.pack(fill="x")
        self.start_button = ttk.Button(
            controls, text="Start Gazebo", command=self.start, style="Accent.TButton",
        )
        self.start_button.pack(side="left")
        self.stop_button = ttk.Button(
            controls, text="Stop simulation", command=self.session.stop, state="disabled",
        )
        self.stop_button.pack(side="left", padx=(8, 0))
        self.status = tk.StringVar(value=self.session.message)
        status_box(parent, self.status, height=5, width=64).pack(fill="x", pady=(12, 12))
        ttk.Label(
            parent, wraplength=800,
            text="1. Start Gazebo and inspect the imported robot.\n"
                 "2. Press Play in Gazebo to let gravity move the robot.\n"
                 "3. Stop and start again to reset the world.\n\n"
                 "This first step uses passive joints and the current approximate linkage model. "
                 "The default physics engine leaves the knee mimic hinges independent. "
                 "It does not hold a standing pose or play hardware animations. "
                 "Simulation sends no commands to the physical robot.",
        ).pack(anchor="w")
        parent.after(250, self.poll)

    def start(self):
        if sys.platform != "linux":
            self.session.message = "Run the desktop in Ubuntu/WSL with ROS 2 Jazzy and Gazebo Harmonic."
            return
        self.session.start()

    def poll(self):
        state = self.session.poll()
        self.status.set(self.session.message)
        active = state in ("starting", "running", "stopping")
        configure_changed(self.start_button, state="disabled" if active else "normal")
        configure_changed(self.stop_button, state="normal" if active and state != "stopping" else "disabled")
        if not self.session.closing:
            self.parent.after(250, self.poll)

    def close(self):
        self.session.close()
