#!/usr/bin/env python3
"""Tk controls and viewport for the embedded accelerated RViz robot view."""

import os
import sys
import tkinter as tk
from tkinter import ttk

from saprog_desktop.desktop_style import BACKGROUND, BORDER
from saprog_desktop.robot_view_session import RendererSession


class RobotView:
    def __init__(self, parent, workspace):
        self.root = parent
        self.session = RendererSession(workspace)

        ttk.Label(parent, text="Robot visualization", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            parent,
            text=("URDF preview: joints start at zero and retain their last received position. "
                  "IMU orientation follows live data; floor height is a visual reference."),
            style="Muted.TLabel",
            wraplength=850,
        ).pack(anchor="w", pady=(4, 10))
        controls = ttk.Frame(parent)
        controls.pack(fill="x")
        self.start_button = ttk.Button(
            controls, text="Start 3D view", command=self.start, style="Accent.TButton"
        )
        self.start_button.pack(side="left")
        self.stop_button = ttk.Button(controls, text="Stop view", command=self.stop, state="disabled")
        self.stop_button.pack(side="left", padx=(8, 0))
        self.status = tk.StringVar(value="Start the view to render the live robot below.")
        ttk.Label(parent, textvariable=self.status, style="Muted.TLabel", wraplength=850).pack(
            anchor="w", pady=(8, 8)
        )
        self.viewport = tk.Frame(
            parent, background=BACKGROUND, highlightthickness=1,
            highlightbackground=BORDER, takefocus=True,
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
        if self.session.closing or self.session.state in ("starting", "running", "stopping"):
            return
        error = self._platform_error()
        if error:
            self.status.set(error)
            return
        if not self.viewport.winfo_ismapped():
            self.status.set("Select the Robot tab before starting the embedded view.")
            return
        self.viewport.update_idletasks()
        if not self.session.start(
            self.viewport.winfo_id(), max(1, self.viewport.winfo_width()),
            max(1, self.viewport.winfo_height()),
        ):
            self.status.set(self.session.message)
            return
        self.status.set(self.session.message)
        self.start_button.configure(state="disabled")
        self.stop_button.configure(state="normal")

    def stop(self):
        self.session.stop()
        self.status.set(self.session.message)

    def poll(self):
        state = self.session.poll()
        self.status.set(self.session.message)
        active = state in ("starting", "running", "stopping")
        self.start_button.configure(state="disabled" if active else "normal")
        self.stop_button.configure(state="normal" if active else "disabled")
        if not self.session.closing and self.root.winfo_exists():
            self.root.after(250, self.poll)

    def close(self):
        self.session.close()
