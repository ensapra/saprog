#!/usr/bin/env python3
"""Manage the embedded accelerated RViz robot view in Home."""

import os
import signal
import subprocess
import sys
import tempfile
import threading
import tkinter as tk
from tkinter import ttk

from desktop_style import BACKGROUND


class RobotView:
    def __init__(self, parent, workspace):
        self.root = parent
        self.workspace = workspace
        self.process = None
        self._stop_requested = False
        self._closing = False
        self._ready_file = None
        self._ready = False

        ttk.Label(parent, text="Robot visualization", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            parent,
            text="RViz2 renders the URDF meshes with OpenGL and follows live servo and IMU data.",
            style="Muted.TLabel",
            wraplength=850,
        ).pack(anchor="w", pady=(4, 10))

        controls = ttk.Frame(parent)
        controls.pack(fill="x")
        self.start_button = ttk.Button(
            controls, text="Start 3D view", command=self.start, style="Accent.TButton"
        )
        self.start_button.pack(side="left")
        self.stop_button = ttk.Button(
            controls, text="Stop view", command=self.stop, state="disabled"
        )
        self.stop_button.pack(side="left", padx=(8, 0))
        self.status = tk.StringVar(value="Start the view to render the live robot below.")
        ttk.Label(parent, textvariable=self.status, style="Muted.TLabel", wraplength=850).pack(
            anchor="w", pady=(8, 8)
        )

        self.viewport = tk.Frame(
            parent,
            background=BACKGROUND,
            highlightthickness=1,
            highlightbackground="#34465d",
            takefocus=True,
        )
        self.viewport.pack(fill="both", expand=True)
        self.root.after(250, self.poll)

    def _platform_error(self):
        if sys.platform != "linux" or not os.environ.get("DISPLAY"):
            return "Embedded RViz requires Linux/WSL with an X11 DISPLAY."
        try:
            if self.root.tk.call("tk", "windowingsystem") != "x11":
                return "Embedded RViz requires Tk running on X11."
        except tk.TclError:
            return "Could not determine whether Tk is running on X11."
        return None

    def start(self):
        if self._closing or (self.process is not None and self.process.poll() is None):
            return
        error = self._platform_error()
        if error:
            self.status.set(error)
            return
        if not self.viewport.winfo_ismapped():
            self.status.set("Select the Robot tab before starting the embedded view.")
            return

        self.viewport.update_idletasks()
        parent_window = self.viewport.winfo_id()
        width = max(1, self.viewport.winfo_width())
        height = max(1, self.viewport.winfo_height())
        descriptor, self._ready_file = tempfile.mkstemp(prefix="saprog-rviz-ready-")
        os.close(descriptor)
        os.unlink(self._ready_file)
        try:
            self.process = subprocess.Popen(
                [
                    "ros2", "launch", "saprog", "robot_view.launch.py",
                    f"parent_window:={parent_window}", f"width:={width}", f"height:={height}",
                    f"ready_file:={self._ready_file}",
                ],
                cwd=str(self.workspace),
                env=os.environ.copy(),
                start_new_session=True,
            )
        except OSError as error:
            self.process = None
            self._remove_ready_file()
            self.status.set(f"Could not start embedded RViz: {error}")
            return
        self._stop_requested = False
        self._ready = False
        self.status.set("Starting embedded RViz renderer…")
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")

    def stop(self):
        if self.process is None:
            return
        if self.process.poll() is not None:
            self._finish_process()
            return
        if not self._stop_requested:
            self._stop_requested = True
            self.status.set("Stopping embedded RViz…")
            self.start_button.configure(state="disabled")
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            # ROS launch normally exits promptly; force cleanup without blocking Tk.
            process = self.process
            self.root.after(1800, lambda: self._force_stop(process))

    def _force_stop(self, process):
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass

    def _finish_process(self):
        if self.process is None:
            return
        result = self.process.returncode
        was_ready = self._ready
        was_stopping = self._stop_requested
        process = self.process
        # The launch leader may have exited while a child is still draining.
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        timer = threading.Timer(0.8, lambda: self._force_stop(process))
        timer.start()
        self.process = None
        self._stop_requested = False
        self._remove_ready_file()
        self._ready = False
        if not self._closing:
            if was_stopping:
                self.status.set("Embedded RViz stopped.")
            elif not was_ready:
                self.status.set(
                    "Embedded RViz failed during startup; check X11, RViz plugins, and launch output."
                )
            else:
                self.status.set(
                    f"Embedded RViz exited unexpectedly (code {result}); see the ROS launch output."
                )
            self.start_button.configure(state="normal")
            self.stop_button.configure(state="disabled")

    def poll(self):
        if self.process is not None and not self._ready and self._ready_file:
            if os.path.exists(self._ready_file):
                self._ready = True
                self.status.set("Embedded RViz is running. Drag in the viewport to orbit.")
        if self.process is not None and self.process.poll() is not None:
            self._finish_process()
        if not self._closing and self.root.winfo_exists():
            self.root.after(250, self.poll)

    def close(self):
        self._closing = True
        if self.process is not None:
            try:
                os.killpg(self.process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            # Tk is closing, so keep a bounded cleanup timer instead of waiting in the UI thread.
            process = self.process

            def force_cleanup():
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                try:
                    process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    pass

            timer = threading.Timer(1.8, force_cleanup)
            timer.start()
        self._remove_ready_file()

    def _remove_ready_file(self):
        if self._ready_file:
            try:
                os.unlink(self._ready_file)
            except FileNotFoundError:
                pass
            self._ready_file = None
