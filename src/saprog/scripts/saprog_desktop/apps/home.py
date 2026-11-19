#!/usr/bin/env python3
"""Show live temperatures and launch the desktop tools in the workspace."""

if __name__ == "__main__":
    from saprog_desktop.startup import home_main

    raise SystemExit(home_main(entrypoint=__file__))

import os
import signal
import subprocess
import tkinter as tk
import time
from pathlib import Path
from tkinter import messagebox, ttk

from saprog_desktop.desktop_style import BACKGROUND, apply_style
from saprog_desktop.views.robot_view import RobotView
from saprog_desktop.views.simulation import SimulationView
from saprog_desktop.views.controller import ControllerTab
from saprog_desktop.views import snapshot_player
from saprog_desktop.snapshot_files import default_snapshot_directory
from saprog_desktop.apps.static_animator import build_animator_views

WORKSPACE = Path(__file__).resolve().parents[5]
TABS = WORKSPACE / "tabs"


class Home:
    def __init__(self, root, node, dependencies, scheduler):
        from saprog_desktop.views.imu_monitor import ImuMonitor
        from saprog_desktop.views.temperature_monitor import TemperatureMonitor

        self.root = root
        self.node = node
        self.dependencies = dependencies
        self.scheduler = scheduler
        self.closed = False
        self.children = {}
        self.backend = None
        self.imu_backend = None
        self._imu_retry_at = 0.0
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
        home_panel, home_content = self._new_panel(self.panels)
        robot_panel, robot_content = self._new_panel(self.panels, padding=14)
        simulation_panel, simulation_content = self._new_panel(self.panels, padding=14)
        animator_panel, animator_content = self._new_panel(self.panels, padding=14)
        tools_panel, tools_content = self._new_panel(self.panels, padding=14)
        controller_panel, controller_content = self._new_panel(self.panels, padding=14)
        self.panels.add(home_panel, text="Home")
        self.panels.add(robot_panel, text="Robot")
        self.panels.add(simulation_panel, text="Simulation")
        self.panels.add(animator_panel, text="Static Animator")
        self.panels.add(tools_panel, text="Other panels")
        self.panels.add(controller_panel, text="Controller")
        self.controller_panel = controller_panel
        self._panel_names = {
            str(home_panel): "Home",
            str(robot_panel): "Robot",
            str(simulation_panel): "Simulation",
            str(animator_panel): "Static Animator",
            str(tools_panel): "Other panels",
            str(controller_panel): "Controller",
        }
        self._detached_panels = {}
        self.panels.bind("<Double-Button-1>", self._toggle_panel_window)
        workspace = ttk.Panedwindow(home_content, orient="horizontal")
        workspace.pack(fill="both", expand=True)
        temperature_panel = ttk.Frame(workspace)
        imu_panel = ttk.Frame(workspace)
        workspace.add(temperature_panel, weight=1)
        workspace.add(imu_panel, weight=1)
        ttk.Label(tools_content, text="Open another tool", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            tools_content, text="These tools open in their own windows.", style="Muted.TLabel"
        ).pack(anchor="w", pady=(2, 10))
        self.buttons = ttk.Frame(tools_content)
        self.buttons.pack(fill="x")
        ttk.Button(tools_content, text="Refresh tools", command=self.refresh).pack(
            anchor="w", pady=(8, 0)
        )
        self.status = tk.StringVar(value="")
        ttk.Label(outer, textvariable=self.status, wraplength=380).pack(anchor="w", pady=(10, 0))
        self.monitor = TemperatureMonitor(temperature_panel, self.node, dependencies)
        self.imu_monitor = ImuMonitor(imu_panel, self.node, embedded=True, dependencies=dependencies)
        self.robot_view = RobotView(robot_content, WORKSPACE)
        self.simulation_view = SimulationView(simulation_content, WORKSPACE)
        self.animator_nodes = [
            dependencies.Node("saprog_static_animator_creator"),
            dependencies.Node("saprog_static_animator_player"),
            dependencies.Node("saprog_static_animator_timeline"),
        ]
        for animator_node in self.animator_nodes:
            scheduler.add_node(animator_node)
        directory = default_snapshot_directory()
        self.animator_views = build_animator_views(
            root,
            animator_content,
            self.animator_nodes[0],
            self.animator_nodes[1],
            self.animator_nodes[2],
            directory,
            dependencies=dependencies,
            torque_strength_changed=self._save_animator_torque_strength,
            on_closed=self._finish_close,
        )
        self.creator = self.animator_views.creator
        self.player = self.animator_views.player
        self.timeline = self.animator_views.timeline
        self.controller = ControllerTab(
            controller_content,
            root,
            dependencies,
            directory,
            self.timeline.sequence_directory,
            usb_helper=WORKSPACE / "scripts" / "ensure_controller_usb.bash",
            on_snapshot=self._play_controller_snapshot,
            on_sequence=self._play_controller_sequence,
            on_torque=self._toggle_controller_torque,
            on_release=self._release_all_controller,
        )
        self.panels.bind("<<NotebookTabChanged>>", self._on_panel_changed)
        self._torque_subscription = self.node.create_subscription(
            dependencies.JointState, "/saprog/joint_states", self._observe_torque, 10
        )
        self._observed_torque = {}
        self._torque_observed_at = 0.0
        self._controller_torque_pending = False
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()
        self._ensure_backend()
        self._ensure_imu_backend()
        self.root.after(100, self.watch)

    @staticmethod
    def _new_panel(parent, *, padding=0):
        """Create a wm-manageable page with a themed content frame."""
        panel = tk.Frame(parent, background=BACKGROUND)
        content = ttk.Frame(panel, padding=padding)
        content.pack(fill="both", expand=True)
        return panel, content

    def _toggle_panel_window(self, event):
        """Detach the clicked tab, or restore its detached placeholder."""
        try:
            tab_id = self.panels.identify(event.x, event.y)
            if not tab_id:
                return
            index = self.panels.index(f"@{event.x},{event.y}")
            panel = self.panels.nametowidget(self.panels.tabs()[index])
        except (tk.TclError, IndexError):
            return

        if panel in self._detached_panels:
            self._restore_panel(panel)
            return

        name = self._panel_names.get(str(panel))
        if name is None:
            return
        placeholder = ttk.Frame(self.panels, padding=14)
        ttk.Label(
            placeholder,
            text=f"{name} is open in its own window. Double-click this tab to return it.",
            style="Muted.TLabel",
            wraplength=520,
        ).pack(anchor="nw")
        self.panels.forget(panel)
        try:
            self.root.tk.call("wm", "manage", str(panel))
        except tk.TclError:
            self.panels.add(panel, text=name)
            placeholder.destroy()
            return
        self.root.tk.call("wm", "title", str(panel), f"Saprog — {name}")
        self.root.tk.call("raise", str(panel))
        self.panels.insert(index, placeholder, text=name)
        self._detached_panels[placeholder] = (panel, placeholder, index)
        callback = self.root.register(lambda item=placeholder: self._restore_panel(item))
        self.root.tk.call("wm", "protocol", str(panel), "WM_DELETE_WINDOW", callback)
        self.panels.select(placeholder)

    def _on_panel_changed(self, _event=None):
        if self.panels.select() == str(self.controller_panel):
            self.controller.refresh_files()

    def _restore_panel(self, placeholder):
        original, _placeholder, index = self._detached_panels.pop(placeholder)
        self.panels.forget(placeholder)
        self.root.tk.call("wm", "forget", str(original))
        self.panels.insert(index, original, text=self._panel_names[str(original)])
        self.panels.select(original)
        placeholder.destroy()

    @staticmethod
    def title(script):
        return script.stem.removeprefix("launch_").replace("_", " ").title()

    def _save_animator_torque_strength(self, value):
        strength = max(0, min(1023, round(float(value))))
        try:
            snapshot_player.save_torque_strength(
                snapshot_player.player_settings_path(), strength
            )
        except OSError as error:
            self.creator.status.set(
                f"Torque strength changed, but could not save preference: {error}"
            )

    def _observe_torque(self, message):
        if len(message.name) != len(message.effort):
            return
        observed = {
            name: bool(value >= 0.5)
            for name, value in zip(message.name, message.effort)
            if name in self.player.joints and value in (0.0, 1.0)
        }
        self._observed_torque = observed
        if all(joint in observed for joint in self.player.joints):
            self._torque_observed_at = time.monotonic()

    def _toggle_controller_torque(self):
        joints = self.player.joints
        if (
            not joints
            or any(joint not in self._observed_torque for joint in joints)
            or time.monotonic() - self._torque_observed_at > 1.5
        ):
            self.controller.status.set("Waiting for observed torque state from all joints")
            return
        if self.player.torque_batch is not None:
            self.controller.status.set("Wait for the current torque request to finish")
            return
        if self.timeline.run_steps is not None or self.creator.playback_state or self.creator.pending:
            self.controller.status.set("Finish the active motion before toggling torque")
            return
        # If any joint is off, make the set uniformly enabled; otherwise release all.
        enabled = not all(self._observed_torque[joint] for joint in joints)
        if enabled:
            self.player.set_all_torque(True)
        else:
            self._release_all_controller()
        self._controller_torque_pending = self.player.torque_batch is not None
        self.controller.status.set(
            "Enabling torque on all joints..." if enabled else "Disabling torque on all joints..."
        )

    def _release_all_controller(self):
        """Cancel active animation/playback and release every joint before other actions resume."""
        if self.timeline.run_steps is not None:
            self.timeline.stop()
        self.player.stop_playback_and_disable_all_torque()
        self.creator.release_all()
        self._controller_torque_pending = self.player.torque_batch is not None
        self.controller.status.set("Disabling torque on all joints...")

    def _play_controller_snapshot(self, path):
        if self.timeline.run_steps is not None:
            self.controller.status.set("Finish the running sequence before playing a snapshot")
            return
        self.player.play(path)

    def _play_controller_sequence(self, path):
        if self.timeline.run_steps is not None:
            self.controller.status.set("A sequence is already running")
            return
        if self.timeline._load_from_path(path):
            self.timeline.play_timeline()

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
        self.monitor.poll()
        self.refresh()
        if self._controller_torque_pending and self.player.torque_batch is None:
            self.controller.status.set(self.player.status.get())
            self._controller_torque_pending = False
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
            self._imu_retry_at = time.monotonic() + 5.0
            self.status.set("IMU ROS backend stopped; retrying in 5 seconds")
        if self.imu_backend is None and time.monotonic() >= self._imu_retry_at:
            self._ensure_imu_backend()
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
            self._imu_retry_at = time.monotonic() + 5.0
            self.status.set(f"Could not start the IMU backend: {error}")
            return False

    def launch(self, script):
        process, _notified = self.children.get(script, (None, False))
        if process and process.poll() is None:
            self.status.set(f"{self.title(script)} is already open")
            return
        hardware_free = script.name == "launch_simulation.sh"
        if not hardware_free and not self._ensure_backend():
            return
        try:
            env = os.environ.copy()
            if not hardware_free:
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
        self.animator_views.request_close()

    def _finish_close(self):
        if self.closed:
            return
        self.closed = True
        self.controller.close()
        self.node.destroy_subscription(self._torque_subscription)
        for script, (process, _notified) in self.children.items():
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
        self.imu_monitor.close()
        self.robot_view.close()
        self.simulation_view.close()
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
        self.scheduler.request_close()
