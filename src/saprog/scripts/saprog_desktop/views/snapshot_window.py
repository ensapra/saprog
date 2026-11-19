#!/usr/bin/env python3
"""Tk desktop interface for saving and replaying saprog joint poses."""

if __name__ == "__main__":
    from saprog_desktop.startup import snapshot_creator_main

    raise SystemExit(snapshot_creator_main(entrypoint=__file__))

import json
import math
import os
import re
import time

from saprog_desktop.desktop_style import BACKGROUND, apply_style, configure_changed, status_box
from saprog_desktop.motion_controls import add_slider, set_motion
from saprog_desktop.playback import FRESH_SECONDS
from saprog_desktop.snapshot_config import SnapshotConfigurationClient
from saprog_desktop.snapshot_files import write_json_atomic
from saprog_desktop.snapshot_units import (
    LEG_JOINTS,
    to_radians,
    validate_snapshot,
)
from saprog_desktop.torque import TorqueBatch

SCHEMA = 1
TARGET_TOLERANCE = 0.05
TORQUE_TIMEOUT = 8.0
LEG_LAYOUT = tuple(
    (name, position, joints)
    for (name, joints), position in zip(LEG_JOINTS, ((0, 0), (0, 1), (1, 0), (1, 1)))
)
LEG_FOR_JOINT = {joint: name for name, _position, joints in LEG_LAYOUT for joint in joints}


def safe_filename(name):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip()).strip("._")[:80] or "snapshot"


class SnapshotWindow:
    def __init__(
        self,
        root,
        node,
        directory,
        container=None,
        show_snapshot_loader=True,
        initial_torque_strength=1023,
        torque_strength_changed=None,
        show_status_box=True,
        dependencies=None,
    ):
        if dependencies is None:
            raise ValueError("SnapshotWindow requires injected desktop runtime dependencies")
        self.root, self.node, self.directory = root, node, directory
        self.dependencies = dependencies
        self.container = container or root
        self.show_snapshot_loader = show_snapshot_loader
        self.pending_snapshot_path = None
        self.external_busy = lambda: False
        self.stop_external_operation = None
        self.torque_strength_changed = torque_strength_changed
        self.show_status_box = show_status_box
        self.joints = []
        self.positions = {}
        self.last_state = 0.0
        self.last_state_stamp = 0
        self.playback_state = None
        self.fixed = set()
        self.torque_state = {}
        self.pending = {}
        self.bounds = {}
        self.services = {}
        self.torque_adapters = {}
        self.target_values = {}
        self.target_tolerance = TARGET_TOLERANCE
        self.target_edited = set()
        self.target_inputs = set()
        self.manual_buttons = {}
        self.torque_buttons = {}
        self.current_labels = {}
        self.status = dependencies.tk.StringVar(
            value="Waiting for controller configuration and joint states"
        )
        self.snapshot_name = dependencies.tk.StringVar()
        self.selected = dependencies.tk.StringVar()
        self.selected_path = None
        self.hold_after_load = dependencies.tk.BooleanVar(value=True)
        self.moving_speed = dependencies.tk.IntVar(value=1023)
        self.torque_strength = dependencies.tk.IntVar(value=initial_torque_strength)
        self._build()
        self.param_client = dependencies.AsyncParameterClient(node, "/saprog_position_controller")
        self.configuration = SnapshotConfigurationClient(
            node.create_client(dependencies.Trigger, "/saprog/refresh_limits"),
            self.param_client,
            dependencies.Trigger,
            self._configuration_result,
        )
        self.configuration.refresh()
        self.subscription = node.create_subscription(
            dependencies.JointState, "/saprog/joint_states", self._state, 10
        )
        self.command_pub = node.create_publisher(
            dependencies.JointState, "/saprog/position_command", 100
        )
        self.timer = node.create_timer(0.1, lambda: None)
        self.refresh_files()
        self.root.after(30, self._poll)
        if hasattr(self.root, "bind"):
            self.root.bind("<Destroy>", self._root_destroyed, add="+")

    def _root_destroyed(self, event):
        if event.widget is self.root:
            self.close()

    def close(self):
        self.configuration.close()
        for adapter in self.torque_adapters.values():
            adapter.close()
        if self.playback_state is not None:
            adapter = self.playback_state.get("release_adapter")
            if adapter is not None:
                adapter.close()

    def _build(self):
        if self.container is self.root:
            apply_style(self.root)
            self.root.title("Snapshot Creator")
            self.root.geometry("940x820")
        frame = self.dependencies.ttk.Frame(self.container, padding=14)
        frame.pack(fill="both", expand=True)
        self.dependencies.ttk.Label(frame, text="Snapshot Creator", style="Title.TLabel").pack(
            anchor="w"
        )
        self.speed_scale = add_slider(
            frame,
            "Movement speed",
            self.moving_speed,
            1,
            ttk_module=self.dependencies.ttk,
        )
        self.torque_scale = add_slider(
            frame,
            "Torque strength",
            self.torque_strength,
            0,
            self.torque_strength_changed,
            ttk_module=self.dependencies.ttk,
        )
        self.dependencies.ttk.Label(
            frame,
            text="Applies to the next Fix or Load Snapshot.",
            style="Muted.TLabel",
        ).pack(anchor="w")
        rows = self.dependencies.ttk.Frame(frame)
        rows.pack(fill="both", expand=True, pady=10)
        self.canvas = self.dependencies.tk.Canvas(
            rows, height=430, highlightthickness=0, background=BACKGROUND
        )
        scrollbar = self.dependencies.ttk.Scrollbar(
            rows, orient="vertical", command=self.canvas.yview
        )
        self.canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.joint_frame = self.dependencies.ttk.Frame(self.canvas)
        self.joint_canvas_item = self.canvas.create_window(
            (0, 0), window=self.joint_frame, anchor="nw"
        )
        self.joint_frame.bind(
            "<Configure>", lambda _e: self.canvas.configure(scrollregion=self.canvas.bbox("all"))
        )
        self.canvas.bind(
            "<Configure>",
            lambda event: self.canvas.itemconfigure(self.joint_canvas_item, width=event.width),
        )
        self.leg_frames = {}
        for name, (grid_row, grid_column), _joints in LEG_LAYOUT:
            leg_frame = self.dependencies.ttk.LabelFrame(self.joint_frame, text=name, padding=8)
            leg_frame.grid(row=grid_row, column=grid_column, sticky="nsew", padx=5, pady=5)
            self.leg_frames[name] = leg_frame
        for column in range(2):
            self.joint_frame.columnconfigure(column, weight=1, uniform="leg-column")
        for row in range(2):
            self.joint_frame.rowconfigure(row, weight=1, uniform="leg-row")
        self.other_servos_frame = None
        name_row = self.dependencies.ttk.Frame(frame)
        name_row.pack(fill="x", pady=5)
        self.dependencies.ttk.Label(name_row, text="Snapshot name").pack(side="left")
        self.dependencies.ttk.Entry(name_row, textvariable=self.snapshot_name).pack(
            side="left", fill="x", expand=True, padx=8
        )
        self.save_button = self.dependencies.ttk.Button(
            name_row, text="Save targets", style="Accent.TButton", command=self.save
        )
        self.save_button.pack(side="right")
        self.combo = self.refresh_button = self.browse_button = self.load_button = None
        if self.show_snapshot_loader:
            load_row = self.dependencies.ttk.Frame(frame)
            load_row.pack(fill="x", pady=5)
            self.combo = self.dependencies.ttk.Combobox(
                load_row, textvariable=self.selected, state="readonly"
            )
            self.combo.pack(side="left", fill="x", expand=True)
            self.combo.bind(
                "<<ComboboxSelected>>", lambda _event: setattr(self, "selected_path", None)
            )
            self.refresh_button = self.dependencies.ttk.Button(
                load_row, text="Refresh", command=self.refresh_files
            )
            self.refresh_button.pack(side="left", padx=6)
            self.browse_button = self.dependencies.ttk.Button(
                load_row, text="Browse...", command=self.browse_snapshot
            )
            self.browse_button.pack(side="left", padx=3)
            self.load_button = self.dependencies.ttk.Button(
                load_row, text="Load Snapshot", command=self.load
            )
            self.load_button.pack(side="left")
        self.fixed_toggle = None
        self.dependencies.ttk.Label(
            frame,
            text="Pose playback sends one command and leaves controls available. Use Release all joints to release torque.",
            wraplength=480,
        ).pack(anchor="w")
        self.release_all_button = self.dependencies.ttk.Button(
            frame, text="Release all joints", style="Release.TButton", command=self.release_all
        )
        self.release_all_button.pack(anchor="w", pady=8)
        if self.show_status_box:
            self._build_status_box(frame, 64)

    def _build_status_box(self, parent, width):
        self.status_box = status_box(
            parent, self.status, height=3, width=width, tk_module=self.dependencies.tk
        )
        self.status_box.pack(fill="x", pady=8)

    def _configuration_result(self, config, error):
        if config is None:
            if not self.joints:
                self.status.set(f"Configuration unavailable: {error}")
            return
        self._configure(config)

    def _configure(self, config):
        try:
            joints = list(config.joints)
            services = {
                joint: self.node.create_client(self.dependencies.SetBool, f"/saprog/{joint}/torque")
                for joint in joints
            }
            torque_adapters = {
                joint: TorqueBatch(
                    {joint: client}, self.dependencies.SetBool, timeout=TORQUE_TIMEOUT
                )
                for joint, client in services.items()
            }
            self.target_tolerance = config.tolerance
            self.joints, self.bounds = joints, config.bounds
            self.services, self.torque_adapters = services, torque_adapters
            for leg_frame in self.leg_frames.values():
                for child in leg_frame.winfo_children():
                    child.destroy()
            if self.other_servos_frame is not None:
                for child in self.other_servos_frame.winfo_children():
                    child.destroy()
            self.current_labels.clear()
            self.target_values.clear()
            self.target_edited.clear()
            self.target_inputs.clear()
            self.manual_buttons.clear()
            self.torque_buttons.clear()
            unknown_joints = [joint for joint in self.joints if joint not in LEG_FOR_JOINT]
            if unknown_joints and self.other_servos_frame is None:
                self.other_servos_frame = self.dependencies.ttk.LabelFrame(
                    self.joint_frame, text="Other servos", padding=8
                )
                self.other_servos_frame.grid(
                    row=2, column=0, columnspan=2, sticky="nsew", padx=5, pady=5
                )
            for joint in self.joints:
                leg_frame = (
                    self.leg_frames[LEG_FOR_JOINT[joint]]
                    if joint in LEG_FOR_JOINT
                    else self.other_servos_frame
                )
                servo_row = self.dependencies.ttk.Frame(leg_frame, padding=(4, 5))
                servo_row.pack(fill="x", pady=2)
                heading = self.dependencies.ttk.Frame(servo_row)
                heading.pack(fill="x")
                self.dependencies.ttk.Label(heading, text=joint, style="TLabelframe.Label").pack(
                    side="left", anchor="w"
                )
                label = self.dependencies.ttk.Label(
                    heading, text="Waiting for telemetry", anchor="e"
                )
                label.pack(side="left", fill="x", expand=True)
                self.current_labels[joint] = label
                low, high = self.bounds[joint]
                self.dependencies.ttk.Label(
                    servo_row,
                    text=f"Limits  {low:.3f} to {high:.3f} rad",
                    style="Muted.TLabel",
                ).pack(anchor="w", pady=(1, 4))
                controls = self.dependencies.ttk.Frame(servo_row)
                controls.pack(fill="x")
                self.dependencies.ttk.Label(controls, text="Target (rad)").pack(
                    side="left", padx=(0, 6)
                )
                target = self.dependencies.tk.StringVar()
                entry = self.dependencies.ttk.Entry(controls, textvariable=target, width=10)
                entry.pack(side="left", padx=(0, 5))
                entry.bind("<FocusIn>", lambda _event, j=joint: self.target_inputs.add(j))
                entry.bind("<FocusOut>", lambda _event, j=joint: self.target_inputs.discard(j))
                target.trace_add("write", lambda *_args, j=joint: self._update_joint_buttons(j))
                self.target_values[joint] = target
                toggle = self.dependencies.ttk.Button(
                    controls,
                    text="Fix",
                    style="Fix.TButton",
                    command=lambda j=joint: self.toggle_torque(j),
                )
                toggle.pack(side="left", padx=3)
                self.torque_buttons[joint] = toggle
            self.status.set(
                f"Configured {len(self.joints)} joints. Reading position and torque state."
            )
            self.refresh_files()
            if self.pending_snapshot_path:
                path, self.pending_snapshot_path = self.pending_snapshot_path, None
                self.load_targets_from_file(path)
        except Exception as exc:
            self.joints = []
            self.status.set(f"Configuration unavailable: {exc}")

    def _state(self, msg):
        now = time.monotonic()
        self.last_state_stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        found = {}
        if len(msg.name) == len(msg.position):
            has_torque = len(msg.effort) == len(msg.name)
            for index, (joint, position) in enumerate(zip(msg.name, msg.position)):
                if joint in self.joints and math.isfinite(position):
                    found[joint] = float(position)
                    if has_torque and msg.effort[index] in (0.0, 1.0):
                        enabled = msg.effort[index] == 1.0
                        if enabled:
                            self.fixed.add(joint)
                            self.torque_state[joint] = "Torque on"
                        else:
                            self.fixed.discard(joint)
                            self.torque_state[joint] = "Torque off"
                    else:
                        self.fixed.discard(joint)
                        self.torque_state[joint] = "Torque unknown"
        for joint in set(self.joints) - set(found):
            self.fixed.discard(joint)
            self.torque_state[joint] = "Torque unknown"
        self.positions, self.last_state = found, now
        valid = bool(self.joints) and set(found) == set(self.joints)
        self.latest_state_complete = valid
        for joint in self.joints:
            label = self.current_labels.get(joint)
            if label:
                value = found.get(joint)
                state = self.torque_state.get(joint, "torque unknown")
                configure_changed(
                    label,
                    text=f"Current {value:.3f} rad | {state}"
                    if value is not None
                    else "No current reading",
                )
                if value is not None and joint not in self.target_inputs:
                    self.target_values[joint].set(repr(value))
                self._update_joint_buttons(joint)

    def _complete_state(self):
        return (
            bool(self.joints)
            and time.monotonic() - self.last_state <= FRESH_SECONDS
            and set(self.positions) == set(self.joints)
        )

    def torque(self, joint, enabled):
        if enabled and self.external_busy():
            self.status.set("Wait for snapshot playback to finish before changing torque")
            return
        if enabled and self.playback_state is not None:
            self.status.set("Wait for the current snapshot operation to finish")
            return
        if joint in self.pending:
            if enabled:
                return
            self.torque_adapters[joint].close()
            self.torque_adapters[joint] = TorqueBatch(
                {joint: self.services[joint]}, self.dependencies.SetBool, timeout=TORQUE_TIMEOUT
            )
        if not enabled:
            self._wait_post_fix.pop(joint, None)
        token = object()
        self.pending[joint] = (token, time.monotonic(), enabled)
        self._update_joint_buttons(joint)
        self.status.set(f"Waiting for {joint} torque operation")
        try:
            self.torque_adapters[joint].start(enabled)
        except Exception as exc:
            self._apply_torque_result(joint, enabled, token, f"failure: {exc}")

    def _apply_torque_result(self, joint, enabled, token, result):
        if joint not in self.pending or self.pending[joint][0] is not token:
            return
        del self.pending[joint]
        if result == "success":
            self.status.set(f"{joint} torque request accepted; waiting for telemetry")
        else:
            self.status.set(f"{joint} torque operation failed: {result.split(': ', 1)[-1]}")
        self._update_joint_buttons(joint)

    def toggle_torque(self, joint):
        if joint in self.pending:
            return
        if joint in self.fixed:
            if self._target_differs_from_current(joint):
                try:
                    target = to_radians(
                        float(self.target_values[joint].get().strip()),
                        self.bounds[joint],
                        "radians",
                    )
                except ValueError as exc:
                    self.status.set(f"{joint}: {exc}")
                    return
                self.move_to_manual_value(joint, target=target, apply_motion_settings=False)
            else:
                self.torque(joint, False)
            return
        if self.external_busy() or self.playback_state is not None:
            self.status.set("Wait for the current snapshot operation to finish")
            return
        if self.torque_strength.get() <= 0:
            self.status.set("Set Torque strength above zero before moving")
            return
        try:
            target = to_radians(
                float(self.target_values[joint].get().strip()), self.bounds[joint], "radians"
            )
        except ValueError as exc:
            self.status.set(f"{joint}: {exc}")
            return
        self.move_to_manual_value(
            joint, target=target, require_fresh=False, apply_motion_settings=True
        )

    def _target_differs_from_current(self, joint):
        current = self.positions.get(joint)
        if current is None:
            return False
        try:
            target = to_radians(
                float(self.target_values[joint].get().strip()), self.bounds[joint], "radians"
            )
        except (ValueError, KeyError):
            return False
        return not math.isclose(target, current, rel_tol=0.0, abs_tol=1e-9)

    def _update_joint_buttons(self, joint):
        button = self.torque_buttons.get(joint)
        if button is None:
            return
        pending = joint in self.pending or self.playback_state is not None or self.external_busy()
        is_fixed = joint in self.fixed
        moving = is_fixed and self._target_differs_from_current(joint)
        configure_changed(
            button,
            text="Move" if moving else "Release" if is_fixed else "Fix",
            style="Accent.TButton" if moving else "Release.TButton" if is_fixed else "Fix.TButton",
            state="disabled" if pending else "normal",
        )
        manual = self.manual_buttons.get(joint)
        if manual is not None:
            configure_changed(manual, state="disabled" if pending else "normal")

    def move_to_manual_value(
        self, joint, target=None, require_fresh=True, apply_motion_settings=False
    ):
        if self.external_busy():
            self.status.set("Wait for snapshot playback to finish before sending a manual move")
            return
        if apply_motion_settings and self.torque_strength.get() <= 0:
            self.status.set("Set Torque strength above zero before moving")
            return
        if self.playback_state is not None or joint in self.pending:
            self.status.set("Wait for the current snapshot or torque operation to finish")
            return
        if require_fresh and not self._complete_state():
            self.status.set("Manual move requires fresh readings for every configured joint")
            return
        if target is None:
            try:
                target = to_radians(
                    float(self.target_values[joint].get().strip()),
                    self.bounds[joint],
                    "radians",
                )
            except ValueError as exc:
                self.status.set(f"{joint}: {exc}")
                return
        message = self.dependencies.JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name, message.position = [joint], [target]
        if apply_motion_settings:
            set_motion(message, self.moving_speed.get(), self.torque_strength.get())
        try:
            self.command_pub.publish(message)
        except Exception as exc:
            self.status.set(f"Could not send {joint} target: {exc}")
            return
        self.target_edited.add(joint)
        self.status.set(f"Sent {joint} target {target:.3f} rad; waiting for telemetry")
        self._update_joint_buttons(joint)

    @property
    def _wait_post_fix(self):
        if not hasattr(self, "_post_fix"):
            self._post_fix = {}
        return self._post_fix

    def release_all(self):
        if not self.joints and self.stop_external_operation is not None:
            self.status.set("Releasing torque through the Player controller")
            self.stop_external_operation()
            return
        was_playing = self.playback_state is not None
        self.playback_state = None
        if was_playing:
            self._set_playback_controls(False)
        external_release = self.external_busy() and self.stop_external_operation is not None
        if external_release:
            self.stop_external_operation()
        for joint in self.joints:
            if not external_release or joint in self.pending:
                self.torque(joint, False)
        if not self.joints:
            self.status.set("Cannot release torque: joint configuration is not available")

    def refresh_files(self):
        if self.combo is None:
            return
        try:
            os.makedirs(self.directory, exist_ok=True)
            files = sorted(p[:-5] for p in os.listdir(self.directory) if p.endswith(".json"))
            self.combo["values"] = files
        except OSError as exc:
            self.status.set(f"Snapshot directory error: {exc}")

    def browse_snapshot(self):
        path = self.dependencies.filedialog.askopenfilename(
            parent=self.root,
            title="Load snapshot",
            initialdir=self.directory,
            filetypes=(("JSON snapshots", "*.json"), ("All files", "*")),
        )
        if path:
            self.selected_path = path
            self.selected.set(os.path.basename(path))
            self.status.set(f"Selected {path}; choose Load Snapshot to validate and send it")

    def load_targets_from_file(self, path):
        """Load a validated pose file into the editable target fields; send no commands."""
        if not self.joints:
            self.pending_snapshot_path = path
            self.status.set("Pose selected; waiting for controller configuration to load it")
            return False
        try:
            with open(path, encoding="utf-8") as stream:
                name, positions = validate_snapshot(json.load(stream), self.joints, self.bounds)
        except Exception as exc:
            self.status.set(f"Selected pose rejected; targets unchanged: {exc}")
            return False
        for joint, value in positions.items():
            self.target_values[joint].set(repr(value))
            self.target_edited.add(joint)
        self.snapshot_name.set(name)
        self.status.set(f"Loaded '{name}' into the editor; no commands sent")
        return True

    def save(self):
        name = self.snapshot_name.get().strip()
        if not name or not self.joints or self.pending or self.playback_state is not None:
            self.status.set("Save requires a name, configured joints, and no pending operation")
            return
        try:
            positions = {}
            for joint in self.joints:
                try:
                    typed = float(self.target_values[joint].get().strip())
                    radians = to_radians(typed, self.bounds[joint], "radians")
                    positions[joint] = radians
                except ValueError as exc:
                    raise ValueError(f"{joint}: {exc}") from exc
            doc = {"schema_version": SCHEMA, "name": name, "positions": positions}
            validate_snapshot(doc, self.joints, self.bounds)
        except ValueError as exc:
            self.status.set(f"Save rejected: {exc}")
            return
        try:
            os.makedirs(self.directory, exist_ok=True)
            path = os.path.join(self.directory, safe_filename(name) + ".json")
            if os.path.exists(path) and not self.dependencies.messagebox.askyesno(
                "Overwrite snapshot?", f"Replace {os.path.basename(path)}?", parent=self.root
            ):
                return
            write_json_atomic(path, doc)
            self.status.set(f"Saved target values '{name}'")
            self.refresh_files()
        except Exception as exc:
            self.status.set(f"Save failed: {exc}")

    def load(self):
        if self.torque_strength.get() <= 0:
            self.status.set("Set Torque strength above zero before playback")
            return
        if (
            not self.selected.get()
            or self.pending
            or self.playback_state is not None
            or not self._complete_state()
        ):
            self.status.set(
                "Load requires a selected snapshot, fresh complete telemetry, and no torque operation"
            )
            return
        path = self.selected_path or os.path.join(
            self.directory, os.path.basename(self.selected.get()) + ".json"
        )
        try:
            with open(path, encoding="utf-8") as stream:
                name, positions = validate_snapshot(json.load(stream), self.joints, self.bounds)
        except Exception as exc:
            self.status.set(f"Snapshot rejected; no commands sent: {exc}")
            return
        self.playback_state = {
            "name": name,
            "targets": positions,
            "queue": [(list(self.joints), [positions[joint] for joint in self.joints])],
            "phase": "sending",
            "last_send": 0.0,
        }
        self._set_playback_controls(True)
        self.status.set(f"Sending commands for '{name}'...")

    def _set_playback_controls(self, locked):
        state = "disabled" if locked else "normal"
        for button in (
            self.save_button,
            self.refresh_button,
            self.browse_button,
            self.load_button,
            self.fixed_toggle,
            self.speed_scale,
            self.torque_scale,
        ):
            if button is not None:
                configure_changed(button, state=state)
        if self.combo is not None:
            configure_changed(self.combo, state="disabled" if locked else "readonly")
        for joint in self.joints:
            self._update_joint_buttons(joint)

    def _finish_playback(self, message):
        self.status.set(message)
        self.playback_state = None
        self._set_playback_controls(False)

    def _playback_tick(self):
        playback = self.playback_state
        if playback is None:
            return
        now = time.monotonic()
        if playback["phase"] == "sending":
            if playback["queue"] and now - playback["last_send"] >= 0.12:
                joints, positions = playback["queue"].pop(0)
                message = self.dependencies.JointState()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.name, message.position = joints, positions
                set_motion(message, self.moving_speed.get(), self.torque_strength.get())
                try:
                    self.command_pub.publish(message)
                except Exception as exc:
                    self._finish_playback(
                        f"Command publication failed; torque was not released: {exc}"
                    )
                    return
                for joint, position in zip(joints, positions):
                    if joint in self.target_values:
                        self.target_values[joint].set(repr(position))
                        self.target_edited.add(joint)
                playback["last_send"] = now
                if not playback["queue"]:
                    self._finish_playback(
                        "Pose command sent; controls ready. No automatic torque release."
                    )
            return

    def _poll(self):
        self.configuration.poll()
        if not self.joints and self.pending_snapshot_path is None:
            self.status.set(self.configuration.error)
        for joint, (token, _started, enabled) in list(self.pending.items()):
            outcome = self.torque_adapters[joint].poll()[joint]
            if outcome != "pending":
                self._apply_torque_result(joint, enabled, token, outcome)
        self._playback_tick()
        can_load = (
            self.joints
            and self._complete_state()
            and not self.pending
            and self.playback_state is None
        )
        if not self.playback_state and self.load_button is not None:
            configure_changed(self.load_button, state="normal" if can_load else "disabled")
        configure_changed(self.release_all_button, state="normal")
        shared_state = (
            "disabled" if self.external_busy() or self.playback_state is not None else "normal"
        )
        for widget in (self.speed_scale, self.torque_scale, self.fixed_toggle):
            if widget is not None:
                configure_changed(widget, state=shared_state)
        for joint in self.joints:
            self._update_joint_buttons(joint)
        self.root.after(30, self._poll)
