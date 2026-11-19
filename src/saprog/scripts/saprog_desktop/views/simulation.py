"""Tk controls for the independent Gazebo inspection world."""

import sys
import tkinter as tk
from pathlib import Path
from tkinter import filedialog, ttk

from saprog_desktop.desktop_style import configure_changed, status_box
from saprog_desktop.simulation_session import SimulationSession


class SimulationView:
    def __init__(self, parent, workspace):
        self.parent = parent
        self.session = SimulationSession(workspace)
        self.snapshots = Path(workspace) / "snapshots"
        self.sequences = Path(workspace) / "sequences"
        self.pending = None
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
        status_box(parent, self.status, height=3, width=64).pack(fill="x", pady=(10, 8))
        library = ttk.LabelFrame(parent, text="Play in Gazebo", padding=10)
        library.pack(fill="x")
        library.columnconfigure(1, weight=1)
        self.selection = {}
        self.choices = {}
        self.paths = {}
        self.play_buttons = []
        for row, kind in enumerate(("snapshot", "sequence")):
            ttk.Label(library, text=kind.title()).grid(row=row, column=0, sticky="w", padx=(0, 8))
            variable = tk.StringVar()
            self.selection[kind] = variable
            combo = ttk.Combobox(library, textvariable=variable, state="readonly", width=35)
            combo.grid(row=row, column=1, sticky="ew", pady=3)
            self.choices[kind] = combo
            ttk.Button(library, text="Browse…", command=lambda k=kind: self.browse(k)).grid(
                row=row, column=2, padx=5,
            )
            button = ttk.Button(library, text="Play " + kind, command=lambda k=kind: self.play(k), state="disabled")
            button.grid(row=row, column=3)
            self.play_buttons.append(button)
        actions = ttk.Frame(library)
        actions.grid(row=2, column=0, columnspan=4, sticky="w", pady=(8, 0))
        self.cancel_button = ttk.Button(actions, text="Stop playback / hold", command=lambda: self.action("stop"), state="disabled")
        self.cancel_button.pack(side="left")
        self.release_button = ttk.Button(actions, text="Release simulated joints", command=lambda: self.action("release"), state="disabled")
        self.release_button.pack(side="left", padx=6)
        ttk.Button(actions, text="Snapshot folder…", command=self.choose_directory).pack(side="left")
        ttk.Button(actions, text="Refresh", command=self.refresh_files).pack(side="left", padx=6)
        self.playback_status = tk.StringVar(value="Start Gazebo to enable playback.")
        status_box(parent, self.playback_status, height=3, width=64).pack(fill="x", pady=(8, 8))
        self.refresh_files()
        ttk.Label(
            parent, wraplength=800,
            text="Select a pose or sequence, then press Play in Gazebo to advance physics and playback. "
                 "Pause in Gazebo also pauses sequence timing. Stop/start Gazebo resets the world. "
                 "Actuator forces and knee coupling are approximations. Commands stay inside simulation.",
        ).pack(anchor="w")
        parent.after(250, self.poll)

    def refresh_files(self):
        for kind, directory in (("snapshot", self.snapshots), ("sequence", self.sequences)):
            paths = {path.name: path for path in sorted(directory.glob("*.json"))}
            self.paths[kind] = paths
            self.choices[kind].configure(values=list(paths))
            if not self.selection[kind].get() and paths:
                self.selection[kind].set(next(iter(paths)))

    def browse(self, kind):
        path = filedialog.askopenfilename(parent=self.parent, title=f"Select {kind}", filetypes=[("JSON", "*.json")])
        if path:
            self.selection[kind].set(path)

    def choose_directory(self):
        path = filedialog.askdirectory(parent=self.parent, title="Snapshot folder for sequence references", initialdir=self.snapshots)
        if path:
            self.snapshots = Path(path)
            self.selection["snapshot"].set("")
            self.refresh_files()
            self.playback_status.set(f"Snapshot references resolve in: {self.snapshots}")

    def play(self, kind):
        selected = self.selection[kind].get()
        if not selected:
            self.playback_status.set(f"Select a {kind} file first.")
            return
        self.action("play", path=self.paths[kind].get(selected, selected), kind=kind, snapshots=self.snapshots)

    def action(self, action, **options):
        try:
            self.pending = self.session.command(action, **options)
            self.playback_status.set("Loading simulation playback…" if action == "play" else "Stopping playback…")
        except (OSError, ValueError, TypeError) as error:
            self.playback_status.set(str(error))

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
        playback = self.session.playback
        ready = state == "running" and playback.get("ready", False)
        if self.pending and playback.get("request") == self.pending:
            self.pending = None
        playing = playback.get("state") == "playing"
        for button in self.play_buttons:
            configure_changed(button, state="normal" if ready and not playing and self.pending is None else "disabled")
        configure_changed(self.cancel_button, state="normal" if ready else "disabled")
        configure_changed(self.release_button, state="normal" if ready else "disabled")
        if self.pending is None and playback:
            message = playback.get("message", "")
            if playing and playback.get("paused"):
                message += " — Paused in Gazebo; press Play there to continue."
            self.playback_status.set(message)
        if not active:
            self.pending = None
        if not self.session.closing:
            self.parent.after(250, self.poll)

    def close(self):
        self.session.close()
