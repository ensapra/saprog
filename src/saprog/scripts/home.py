#!/usr/bin/env python3
"""Show live temperatures and launch the desktop tools in the workspace."""

import os
import signal
import subprocess
import sys
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk

import rclpy
from desktop_style import apply_style
from imu_monitor import ImuMonitor
from rclpy.node import Node
from robot_view import RobotView
import snapshot_player
from snapshot_files import default_snapshot_directory
from snapshot_window import SnapshotWindow
from temperature_monitor import TemperatureMonitor
from timeline_window import TimelineWindow

WORKSPACE = Path(__file__).resolve().parents[3]
TABS = WORKSPACE / "tabs"


class Home:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.closed = False
        self.children = {}
        self.backend = None
        self.imu_backend = None
        self.animator_nodes = []
        self._scripts = None
        apply_style(root)
        root.title("Saprog")
        root.geometry("1440x900")
        root.minsize(900, 650)
        outer = ttk.Frame(root, padding=16)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="Saprog workspace", style="Title.TLabel").pack(
            anchor="w", pady=(0, 8)
        )
        self.panels = ttk.Notebook(outer)
        self.panels.pack(fill="both", expand=True)
        home_panel = ttk.Frame(self.panels)
        robot_panel = ttk.Frame(self.panels, padding=14)
        animator_panel = ttk.Frame(self.panels, padding=14)
        tools_panel = ttk.Frame(self.panels, padding=14)
        self.panels.add(home_panel, text="Home")
        self.panels.add(robot_panel, text="Robot")
        self.panels.add(animator_panel, text="Static Animator")
        self.panels.add(tools_panel, text="Other panels")
        workspace = ttk.Panedwindow(home_panel, orient="horizontal")
        workspace.pack(fill="both", expand=True)
        temperature_panel = ttk.Frame(workspace)
        imu_panel = ttk.Frame(workspace)
        workspace.add(temperature_panel, weight=1)
        workspace.add(imu_panel, weight=1)
        ttk.Label(tools_panel, text="Open another tool", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            tools_panel, text="These tools open in their own windows.", style="Muted.TLabel"
        ).pack(anchor="w", pady=(2, 10))
        self.buttons = ttk.Frame(tools_panel)
        self.buttons.pack(fill="x")
        ttk.Button(tools_panel, text="Refresh tools", command=self.refresh).pack(
            anchor="w", pady=(8, 0)
        )
        self.status = tk.StringVar(value="")
        ttk.Label(outer, textvariable=self.status, wraplength=380).pack(anchor="w", pady=(10, 0))
        self.monitor = TemperatureMonitor(temperature_panel, self.node)
        self.imu_monitor = ImuMonitor(imu_panel, self.node, embedded=True)
        self.robot_view = RobotView(robot_panel, WORKSPACE)
        snapshot_player._load_runtime(sys.argv[1:], require_timeline=True, entrypoint=__file__)
        self.animator_nodes = [
            snapshot_player.Node("saprog_static_animator_creator"),
            snapshot_player.Node("saprog_static_animator_player"),
            snapshot_player.Node("saprog_static_animator_timeline"),
        ]
        directory = default_snapshot_directory()
        animator_panel.columnconfigure(0, weight=1, uniform="animator")
        animator_panel.columnconfigure(1, weight=1, uniform="animator")
        animator_panel.rowconfigure(0, weight=2)
        animator_panel.rowconfigure(1, weight=3)
        creator_panel = ttk.Frame(animator_panel)
        player_panel = ttk.Frame(animator_panel)
        timeline_panel = ttk.Frame(animator_panel)
        creator_panel.grid(row=0, column=0, rowspan=2, sticky="nsew", padx=(0, 6), pady=6)
        player_panel.grid(row=0, column=1, sticky="nsew", padx=(6, 0), pady=(6, 3))
        timeline_panel.grid(row=1, column=1, sticky="nsew", padx=(6, 0), pady=(3, 6))
        SnapshotWindow(root, self.animator_nodes[0], directory, container=creator_panel)
        snapshot_player.SnapshotPlayer(
            root, self.animator_nodes[1], directory, container=player_panel
        )
        self.timeline = TimelineWindow(
            root, self.animator_nodes[2], directory, container=timeline_panel
        )
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()
        self._ensure_backend()
        self._ensure_imu_backend()
        self.root.after(100, self.watch)

    @staticmethod
    def title(script):
        return script.stem.removeprefix("launch_").replace("_", " ").title()

    def refresh(self):
        scripts = sorted(
            (
                item
                for item in TABS.glob("*.sh")
                if item.is_file()
                and item.name not in {
                    "launch_imu_monitor.sh",
                    "launch_snapshot_creator.sh",
                    "launch_snapshot_player.sh",
                    "launch_timeline.sh",
                    "launch_static_animator.sh",
                }
            ),
            key=lambda item: item.name.casefold(),
        )
        if scripts == self._scripts:
            return
        self._scripts = scripts
        for widget in self.buttons.winfo_children():
            widget.destroy()
        if not scripts:
            ttk.Label(self.buttons, text="No launchers found in tabs/").pack(anchor="w")
            return
        for script in scripts:
            ttk.Button(
                self.buttons, text=self.title(script), command=lambda path=script: self.launch(path)
            ).pack(fill="x", pady=3)

    def watch(self):
        if self.closed:
            return
        for _ in range(100):
            rclpy.spin_once(self.node, timeout_sec=0)
        for node in self.animator_nodes:
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0)
        self.monitor.poll()
        self.refresh()
        for script, (process, notified) in list(self.children.items()):
            result = process.poll()
            if result is not None and not notified:
                if result:
                    self.status.set(
                        f"{self.title(script)} failed to launch (exit {result}); see launcher output"
                    )
                else:
                    self.status.set(f"{self.title(script)} closed")
                self.children[script] = (process, True)
        if self.backend is not None and self.backend.poll() is not None:
            self.backend = None
            self.status.set("Shared controller stopped")
        if self.imu_backend is not None and self.imu_backend.poll() is not None:
            self.imu_backend = None
            self.status.set("IMU ROS backend stopped; reopen the IMU monitor to restart it")
        if (
            self.backend is not None
            and self.children
            and not any(process.poll() is None for process, _notified in self.children.values())
        ):
            self._stop_backend()
        self.root.after(100, self.watch)

    def _ensure_backend(self):
        if self.backend is not None and self.backend.poll() is None:
            return True
        helper = TABS / "_launch_window.bash"
        env = os.environ.copy()
        env.pop("SAPROG_HOME_BACKEND_PID", None)
        try:
            self.backend = subprocess.Popen(
                ["bash", str(helper), "backend"], cwd=str(WORKSPACE), env=env
            )
            return True
        except OSError as error:
            self.backend = None
            self.status.set(f"Could not start the controller: {error}")
            messagebox.showerror("Could not start controller", str(error), parent=self.root)
            return False

    def _stop_backend(self):
        if self.backend is None:
            return
        if self.backend.poll() is None:
            self.backend.terminate()
            try:
                self.backend.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.backend.kill()
                self.backend.wait()
        self.backend = None

    def _ensure_imu_backend(self):
        if self.imu_backend is not None and self.imu_backend.poll() is None:
            return True
        try:
            self.imu_backend = subprocess.Popen(
                ["ros2", "launch", "saprog", "imu.launch.py", "gui:=false"],
                cwd=str(WORKSPACE),
                env=os.environ.copy(),
                start_new_session=True,
            )
            return True
        except OSError as error:
            self.imu_backend = None
            self.status.set(f"Could not start the IMU backend: {error}")
            return False

    def launch(self, script):
        process, _notified = self.children.get(script, (None, False))
        if process and process.poll() is None:
            self.status.set(f"{self.title(script)} is already open")
            return
        if not self._ensure_backend():
            return
        try:
            env = os.environ.copy()
            env["SAPROG_HOME_BACKEND_PID"] = str(self.backend.pid)
            process = subprocess.Popen(
                ["bash", str(script)],
                cwd=str(WORKSPACE),
                env=env,
            )
            self.children[script] = (process, False)
            self.status.set(f"Opened {self.title(script)}")
        except OSError as error:
            self.status.set(f"Could not open {self.title(script)}: {error}")
            messagebox.showerror("Could not open window", str(error), parent=self.root)
            if not any(process.poll() is None for process, _notified in self.children.values()):
                self._stop_backend()

    def close(self):
        if self.closed:
            return
        self.closed = True
        for script, (process, _notified) in self.children.items():
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        self.imu_monitor.close()
        for node in self.animator_nodes:
            node.destroy_node()
        self.robot_view.close()
        self._stop_backend()
        if self.imu_backend is not None and self.imu_backend.poll() is None:
            try:
                os.killpg(self.imu_backend.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self.imu_backend.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self.imu_backend.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self.imu_backend.wait()
        self.node.destroy_node()
        rclpy.shutdown()
        self.root.destroy()


def main():
    rclpy.init()
    root = tk.Tk()
    node = Node("saprog_home_temperature_monitor")
    app = Home(root, node)
    signal.signal(signal.SIGTERM, lambda *_args: app.close())
    signal.signal(signal.SIGINT, lambda *_args: app.close())
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
