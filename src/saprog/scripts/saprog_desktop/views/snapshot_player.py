#!/usr/bin/env python3
"""Open the snapshot player directly with ``python3 snapshot_player.py``."""

import json
import math
import os
import time
from pathlib import Path

from saprog_desktop.desktop_style import BACKGROUND, apply_style, configure_changed, status_box
from saprog_desktop.motion_controls import add_slider, set_motion
from saprog_desktop.player_preferences import (
    load_torque_strength,
    player_settings_path,
    save_torque_strength,
)
from saprog_desktop.snapshot_config import SnapshotConfigurationClient
from saprog_desktop.snapshot_units import validate_snapshot
from saprog_desktop.torque import TorqueBatch

TARGET_TOLERANCE = 0.05
TORQUE_TIMEOUT = 8.0


class SnapshotPlayer:
    """Tk view and ROS client driven by its root owner's event scheduler.

    Snapshot playback finishes immediately after command publication.
    The root owner injects ROS/Tk dependencies and owns node scheduling/teardown.
    """

    def __init__(
        self,
        root,
        node,
        directory,
        container=None,
        on_pose_selected=None,
        moving_speed=None,
        torque_strength=None,
        hold_after_load=None,
        can_start_playback=None,
        dependencies=None,
    ):
        if dependencies is None:
            raise ValueError("SnapshotPlayer requires injected desktop runtime dependencies")
        self.root, self.node, self.directory = root, node, directory
        self.dependencies = dependencies
        self.container = container or root
        self.joints, self.bounds = [], {}
        self.param_client = dependencies.AsyncParameterClient(node, "/saprog_position_controller")
        self.configuration = SnapshotConfigurationClient(
            node.create_client(dependencies.Trigger, "/saprog/refresh_limits"),
            self.param_client,
            dependencies.Trigger,
            self._configuration_result,
        )
        self.configuration.refresh()
        self.command_pub = node.create_publisher(
            dependencies.JointState, "/saprog/position_command", 100
        )
        self.state_subscription = node.create_subscription(
            dependencies.JointState, "/saprog/joint_states", self._on_state, 10
        )
        self.current_positions = {}
        self.last_state_stamp = 0
        self.last_state_received = 0.0
        self.latest_state_complete = False
        self.torque_clients = {}
        self.playback = None
        self.selected_path = None
        self.on_pose_selected = on_pose_selected
        self.selection_mode = on_pose_selected is not None
        shared = (moving_speed, torque_strength, hold_after_load)
        self.shared_controls = any(value is not None for value in shared)
        if self.shared_controls and any(value is None for value in shared):
            raise ValueError("Shared player controls require speed, strength, and hold variables")
        self.can_start_playback = can_start_playback or (lambda: True)
        self.hold_after_load = (
            hold_after_load
            if hold_after_load is not None
            else dependencies.tk.BooleanVar(value=True)
        )
        self.settings_path = player_settings_path()
        self.torque_strength = (
            torque_strength
            if torque_strength is not None
            else dependencies.tk.IntVar(value=load_torque_strength(self.settings_path))
        )
        self.moving_speed = (
            moving_speed if moving_speed is not None else dependencies.tk.IntVar(value=1023)
        )
        self.torque_batch = None
        self.target_tolerance = TARGET_TOLERANCE
        self.buttons = []
        self._file_signature = None
        self.status = dependencies.tk.StringVar(
            value="Waiting for separately started saprog controller"
        )
        self._build()
        self.refresh()
        self.root.after(100, self._tick)
        self.root.after(2000, self._periodic_refresh)
        if hasattr(self.root, "bind"):
            self.root.bind("<Destroy>", self._root_destroyed, add="+")

    def _root_destroyed(self, event):
        if event.widget is self.root:
            self.close()

    def close(self):
        self.configuration.close()
        adapter = getattr(self, "torque_adapter", None)
        if adapter is not None:
            adapter.close()
        if self.playback is not None:
            release_adapter = self.playback.get("release_adapter")
            if release_adapter is not None:
                release_adapter.close()

    def _build(self):
        if self.container is self.root:
            apply_style(self.root)
            self.root.title("Snapshot Player")
            self.root.geometry("620x680")
        outer = self.dependencies.ttk.Frame(self.container, padding=14)
        outer.pack(fill="both", expand=True)
        self.dependencies.ttk.Label(outer, text="Snapshot Player", style="Title.TLabel").pack(
            anchor="w"
        )
        actions = self.dependencies.ttk.Frame(outer)
        actions.pack(fill="x", pady=(8, 4))
        self.refresh_button = self.dependencies.ttk.Button(
            actions, text="Refresh", command=self.refresh
        )
        self.refresh_button.pack(side="right")
        self.browse_button = self.dependencies.ttk.Button(
            actions, text="Browse...", command=self.browse_snapshot
        )
        self.browse_button.pack(side="right", padx=5)
        self.load_button = self.dependencies.ttk.Button(
            actions,
            text="Play selected pose" if self.selection_mode else "Load Snapshot",
            style="Accent.TButton",
            command=self.load_selected,
        )
        self.load_button.pack(side="right", padx=5)
        self.hold_toggle = self.speed_scale = self.torque_scale = None
        self.disable_torque_button = None
        if not self.shared_controls:
            self.dependencies.ttk.Label(
                outer,
                text="Snapshot targets are in radians. Playback sends one command; release torque manually when needed.",
                wraplength=560,
            ).pack(anchor="w")
            self.speed_scale = add_slider(
                outer,
                "Movement speed",
                self.moving_speed,
                1,
                ttk_module=self.dependencies.ttk,
            )
            torque_row = self.dependencies.ttk.Frame(outer)
            torque_row.pack(fill="x", pady=4)
            self.dependencies.ttk.Label(torque_row, text="Torque strength", width=16).pack(
                side="left"
            )
            self.torque_scale = self.dependencies.ttk.Scale(
                torque_row,
                from_=0,
                to=1023,
                variable=self.torque_strength,
                command=self._change_torque_strength,
            )
            self.torque_scale.pack(side="left", fill="x", expand=True, padx=8)
            self.torque_value = self.dependencies.ttk.Label(
                torque_row, text=f"{self.torque_strength.get()} / 1023", width=12, anchor="e"
            )
            self.torque_value.pack(side="left")
            self.disable_torque_button = self.dependencies.ttk.Button(
                outer, text="Disable torque on all joints", command=self.disable_all_torque
            )
            self.disable_torque_button.pack(anchor="w", pady=(2, 6))
            self.dependencies.ttk.Label(
                outer,
                text="Speed and strength apply to all joints on the next playback.",
                style="Muted.TLabel",
            ).pack(anchor="w", pady=(0, 6))
        self.selected_label = self.dependencies.ttk.Label(
            outer, text="No file selected", wraplength=560
        )
        self.selected_label.pack(anchor="w", pady=(0, 4))
        area = self.dependencies.ttk.Frame(outer)
        area.pack(fill="both", expand=True)
        self.canvas = self.dependencies.tk.Canvas(area, highlightthickness=0, background=BACKGROUND)
        scrollbar = self.dependencies.ttk.Scrollbar(
            area, orient="vertical", command=self.canvas.yview
        )
        self.canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.list_frame = self.dependencies.ttk.Frame(self.canvas)
        self.canvas_window = self.canvas.create_window((0, 0), window=self.list_frame, anchor="nw")
        self.list_frame.bind(
            "<Configure>",
            lambda _event: self.canvas.configure(scrollregion=self.canvas.bbox("all")),
        )
        self.canvas.bind(
            "<Configure>",
            lambda event: self.canvas.itemconfigure(self.canvas_window, width=event.width),
        )
        self.status_box = status_box(
            outer, self.status, height=3, width=64, tk_module=self.dependencies.tk
        )
        self.status_box.pack(fill="x", pady=(10, 0))

    def _change_torque_strength(self, value):
        strength = max(0, min(1023, round(float(value))))
        self.torque_strength.set(strength)
        self.torque_value.configure(text=f"{strength} / 1023")
        try:
            save_torque_strength(self.settings_path, strength)
        except OSError as exc:
            self.status.set(f"Torque strength changed, but could not save preference: {exc}")

    def _configuration_result(self, config, error):
        if config is None:
            if not self.joints:
                self.status.set(f"Controller configuration unavailable: {error}")
            return
        try:
            torque_clients = {
                joint: self.node.create_client(self.dependencies.SetBool, f"/saprog/{joint}/torque")
                for joint in config.joints
            }
            torque_adapter = TorqueBatch(
                torque_clients, self.dependencies.SetBool, timeout=TORQUE_TIMEOUT
            )
            self.target_tolerance = config.tolerance
            self.joints, self.bounds = list(config.joints), config.bounds
            self.torque_clients, self.torque_adapter = torque_clients, torque_adapter
            self.status.set(
                f"Configured {len(config.joints)} joints; waiting for command subscriber"
                if not self._ready()
                else "Ready"
            )
        except Exception as exc:
            self.status.set(f"Controller configuration unavailable: {exc}")

    def _ready(self):
        return bool(self.joints) and self.node.count_subscribers("/saprog/position_command") > 0

    def refresh(self, force=True):
        if self.playback is not None:
            return
        if force:
            self.configuration.refresh()
        try:
            paths = sorted(
                os.path.join(self.directory, filename)
                for filename in os.listdir(self.directory)
                if filename.lower().endswith(".json")
            )
            signature = []
            for path in paths:
                try:
                    metadata = os.stat(path)
                    signature.append(
                        (path, metadata.st_mtime_ns, metadata.st_ctime_ns, metadata.st_size)
                    )
                except OSError:
                    signature.append((path, None))
        except OSError as exc:
            for child in self.list_frame.winfo_children():
                child.destroy()
            self.buttons = []
            self._file_signature = None
            self.status.set(f"Snapshot directory unavailable: {exc}")
            return
        if not force and signature == self._file_signature:
            return
        self._file_signature = signature
        for child in self.list_frame.winfo_children():
            child.destroy()
        self.buttons = []
        if not paths:
            self.dependencies.ttk.Label(self.list_frame, text="No JSON snapshots found.").pack(
                anchor="w", pady=6
            )
        for path in paths:
            filename = os.path.basename(path)
            display_name = filename
            try:
                with open(path, encoding="utf-8") as stream:
                    doc = json.load(stream)
                if (
                    isinstance(doc, dict)
                    and isinstance(doc.get("name"), str)
                    and doc["name"].strip()
                ):
                    display_name = doc["name"].strip()
            except (OSError, ValueError, UnicodeError):
                pass
            if self.selection_mode:
                callback = lambda p=path: self.select_snapshot(p)
            else:
                callback = lambda p=path: self.play(p)
            button = self.dependencies.ttk.Button(
                self.list_frame,
                text=f"{display_name}  -  {filename}",
                command=callback,
            )
            button.pack(fill="x", pady=3)
            self.buttons.append(button)
        self._update_buttons()

    def _update_buttons(self):
        ready = self._ready() and self.playback is None and self.torque_batch is None
        for button in self.buttons:
            selectable = self.selection_mode and self.playback is None and self.torque_batch is None
            configure_changed(button, state="normal" if ready or selectable else "disabled")
        configure_changed(
            self.load_button, state="normal" if ready and self.selected_path else "disabled"
        )
        state = (
            "disabled" if self.playback is not None or self.torque_batch is not None else "normal"
        )
        for widget in (
            self.refresh_button,
            self.browse_button,
            self.hold_toggle,
            self.torque_scale,
            self.speed_scale,
        ):
            if widget is not None:
                configure_changed(widget, state=state)
        if self.disable_torque_button is not None:
            configure_changed(self.disable_torque_button, state="normal")

    def select_snapshot(self, path):
        self.selected_path = path
        self.selected_label.configure(text=Path(path).name)
        self.on_pose_selected(path)
        self._update_buttons()

    def disable_all_torque(self):
        self.stop_playback_and_disable_all_torque()

    def stop_playback_and_disable_all_torque(self):
        """Discard queued pose commands and attempt release on every configured joint."""
        if self.torque_batch is not None:
            self.torque_adapter.close()
            self.torque_adapter = TorqueBatch(
                self.torque_clients, self.dependencies.SetBool, timeout=TORQUE_TIMEOUT
            )
            self.torque_batch = None
        if self.playback is not None:
            release_adapter = self.playback.get("release_adapter")
            if release_adapter is not None:
                release_adapter.close()
            self.playback = None
            self._update_buttons()
        if not self.joints:
            self.status.set("Cannot release torque: joint configuration is not available")
            return
        self._start_disable_all_torque()

    def set_all_torque(self, enabled):
        """Set all configured joints' torque without interrupting active motion."""
        if not enabled:
            self.stop_playback_and_disable_all_torque()
            return
        if self.playback is not None or self.torque_batch is not None:
            self.status.set("Finish the active motion or torque operation before enabling torque")
            return
        if not self.joints:
            self.status.set("Cannot enable torque: joint configuration is not available")
            return
        self._start_torque_batch(True)

    def _start_disable_all_torque(self):
        self._start_torque_batch(False)

    def _start_torque_batch(self, enabled):
        self.torque_batch = {
            "pending": set(self.joints),
            "failed": {},
            "enabled": bool(enabled),
        }
        self._update_buttons()
        action = "Enabling" if enabled else "Disabling"
        self.status.set(f"{action} torque on all joints...")
        batch = self.torque_batch
        batch["adapter"] = self.torque_adapter
        self.torque_adapter.start(enabled)

    def _finish_disable_torque(self, batch):
        if self.torque_batch is not batch or batch["pending"]:
            return
        self.torque_batch = None
        self._update_buttons()
        if batch["failed"]:
            action = "enable" if batch["enabled"] else "disable"
            self.status.set(
                f"Torque {action} incomplete: "
                + "; ".join(f"{joint}: {reason}" for joint, reason in batch["failed"].items())
            )
        else:
            state = "enabled" if batch["enabled"] else "disabled"
            self.status.set(f"Torque {state} on all {len(self.joints)} joints")

    def _check_torque_timeout(self):
        batch = self.torque_batch
        if batch is None:
            return
        outcomes = batch["adapter"].poll()
        for joint in tuple(batch["pending"]):
            outcome = outcomes[joint]
            if outcome == "pending":
                continue
            batch["pending"].remove(joint)
            if outcome != "success":
                batch["failed"][joint] = outcome.split(": ", 1)[-1]
        self._finish_disable_torque(batch)

    def play(self, path):
        if not self.can_start_playback():
            self.status.set("Finish the current creator operation before starting playback")
            return
        if self.playback is not None or not self._ready():
            self.status.set("Waiting for controller configuration and command subscriber")
            return
        if self.torque_batch is not None:
            self.status.set("Wait for torque disable to finish")
            return
        try:
            with open(path, encoding="utf-8") as stream:
                name, positions = validate_snapshot(json.load(stream), self.joints, self.bounds)
        except Exception as exc:
            self.status.set(f"Snapshot rejected; no commands sent: {exc}")
            return
        self._start_playback(name, positions)

    def _start_playback(self, name, positions):
        if self.torque_strength.get() <= 0:
            self.status.set(
                "Set Torque strength above zero before playback; zero cannot drive the servos"
            )
            return
        self.playback = {
            "name": name,
            "targets": positions,
            "queue": [(list(self.joints), [positions[joint] for joint in self.joints])],
            "phase": "sending",
            "last_send": 0.0,
        }
        self._set_playback_controls(True)
        self.status.set(f"Sending commands for '{name}'...")

    def browse_snapshot(self):
        path = self.dependencies.filedialog.askopenfilename(
            parent=self.root,
            title="Load snapshot",
            initialdir=os.path.expanduser(self.directory),
            filetypes=(("JSON snapshots", "*.json"), ("All files", "*")),
        )
        if path:
            if self.selection_mode:
                self.select_snapshot(path)
            else:
                self.selected_path = path
                self.selected_label.configure(text=path)
                self.status.set("Selected snapshot; choose Load Snapshot to validate and send it")
            self._update_buttons()

    def load_selected(self):
        if not self.selected_path:
            self.status.set("Choose a JSON snapshot with Browse first")
            return
        self.play(self.selected_path)

    def _set_playback_controls(self, locked):
        self._update_buttons()

    def _on_state(self, message):
        self.last_state_received = time.monotonic()
        self.last_state_stamp = (
            message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        )
        found = {}
        if len(message.name) == len(message.position):
            for joint, position in zip(message.name, message.position):
                if joint in self.joints and math.isfinite(position):
                    found[joint] = float(position)
        self.current_positions = found
        valid = bool(self.joints) and set(found) == set(self.joints)
        self.latest_state_complete = valid

    def _finish_playback(self, status):
        self.status.set(status)
        self.playback = None
        self._set_playback_controls(False)

    def _apply_motion(self, message):
        set_motion(message, self.moving_speed.get(), self.torque_strength.get())

    def _playback_tick(self):
        playback = self.playback
        if playback is None:
            return
        now = time.monotonic()
        if playback["phase"] == "sending":
            while playback["queue"] and now - playback["last_send"] >= 0.12:
                joints, positions = playback["queue"].pop(0)
                message = self.dependencies.JointState()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.name, message.position = joints, positions
                self._apply_motion(message)
                try:
                    self.command_pub.publish(message)
                except Exception as exc:
                    self._finish_playback(
                        f"Synchronized command publication failed; torque was not released: {exc}"
                    )
                    return
                playback["last_send"] = now
                if not playback["queue"]:
                    self._finish_playback(
                        "Pose command sent; controls ready. No automatic torque release."
                    )
            return

    def _tick(self):
        self.configuration.poll()
        self._check_torque_timeout()
        self._playback_tick()
        if self.playback is None:
            if self.joints and not self._ready():
                self.status.set("Configured; waiting for /saprog/position_command subscriber")
            self._update_buttons()
        self.root.after(100, self._tick)

    def _periodic_refresh(self):
        if self.playback is None:
            self.refresh(force=False)
        self.root.after(2000, self._periodic_refresh)


if __name__ == "__main__":
    from saprog_desktop.startup import snapshot_player_main

    raise SystemExit(snapshot_player_main())
