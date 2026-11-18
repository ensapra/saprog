#!/usr/bin/env python3
"""Tk desktop interface for saving and replaying saprog joint poses."""
import argparse
import json
import math
import os
from snapshot_arrival import arrival_progress, AX_TICK_RADIANS
from snapshot_units import joint_bounds, to_radians, validate_snapshot
from snapshot_files import default_snapshot_directory, write_json_atomic
import re
import time
import tkinter as tk
from tkinter import filedialog, messagebox, ttk
from desktop_style import BACKGROUND, apply_style, configure_changed
from motion_controls import add_slider, set_motion

import rclpy
from rclpy.node import Node
from snapshot_limits_client import HardwareParameterClient as AsyncParameterClient
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


SCHEMA = 1
FRESH_SECONDS = 2.5
TARGET_TOLERANCE = 0.05
STABLE_SAMPLES = 3
STABLE_SECONDS = 1.0
ARRIVAL_TIMEOUT = 30.0
TORQUE_TIMEOUT = 8.0
LEG_LAYOUT = (
    ("Front Left Leg", (0, 0), ("servo_31", "servo_32", "servo_33")),
    ("Front Right Leg", (0, 1), ("servo_11", "servo_12", "servo_13")),
    ("Back Left Leg", (1, 0), ("servo_41", "servo_42", "servo_43")),
    ("Back Right Leg", (1, 1), ("servo_21", "servo_22", "servo_23")),
)
LEG_FOR_JOINT = {joint: name for name, _position, joints in LEG_LAYOUT for joint in joints}


def safe_filename(name):
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name.strip()).strip("._")[:80] or "snapshot"


class SnapshotWindow:
    def __init__(self, root, node, directory):
        self.root, self.node, self.directory = root, node, directory
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
        self.target_values = {}
        self.target_tolerance = TARGET_TOLERANCE
        self.target_edited = set()
        self.manual_buttons = {}
        self.torque_buttons = {}
        self.current_labels = {}
        self._displayed_status = None
        self.status = tk.StringVar(value="Waiting for controller configuration and joint states")
        self.snapshot_name = tk.StringVar()
        self.selected = tk.StringVar()
        self.selected_path = None
        self.hold_after_load = tk.BooleanVar(value=True)
        self.moving_speed = tk.IntVar(value=1023)
        self.torque_strength = tk.IntVar(value=1023)
        self._build()
        self.param_client = AsyncParameterClient(node, "/saprog_position_controller")
        self.param_future = None
        self.subscription = node.create_subscription(JointState, "/saprog/joint_states", self._state, 10)
        self.command_pub = node.create_publisher(JointState, "/saprog/position_command", 100)
        self.timer = node.create_timer(0.1, lambda: None)
        self.refresh_files()
        self.root.after(30, self._poll)

    def _build(self):
        apply_style(self.root)
        self.root.title("Snapshot Creator")
        self.root.geometry("940x820")
        frame = ttk.Frame(self.root, padding=14)
        frame.pack(fill="both", expand=True)
        ttk.Label(frame, text="Snapshot Creator", style="Title.TLabel").pack(anchor="w")
        self.speed_scale = add_slider(frame, "Movement speed", self.moving_speed, 1)
        self.torque_scale = add_slider(frame, "Torque strength", self.torque_strength, 0)
        ttk.Label(frame, text="Applies to every joint on the next Move or Load Snapshot.",
                  style="Muted.TLabel").pack(anchor="w")
        rows = ttk.Frame(frame)
        rows.pack(fill="both", expand=True, pady=10)
        self.canvas = tk.Canvas(rows, height=430, highlightthickness=0, background=BACKGROUND)
        scrollbar = ttk.Scrollbar(rows, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.joint_frame = ttk.Frame(self.canvas)
        self.joint_canvas_item = self.canvas.create_window((0, 0), window=self.joint_frame, anchor="nw")
        self.joint_frame.bind("<Configure>", lambda _e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda event: self.canvas.itemconfigure(self.joint_canvas_item, width=event.width))
        self.leg_frames = {}
        for name, (grid_row, grid_column), _joints in LEG_LAYOUT:
            leg_frame = ttk.LabelFrame(self.joint_frame, text=name, padding=8)
            leg_frame.grid(row=grid_row, column=grid_column, sticky="nsew", padx=5, pady=5)
            self.leg_frames[name] = leg_frame
        for column in range(2):
            self.joint_frame.columnconfigure(column, weight=1, uniform="leg-column")
        for row in range(2):
            self.joint_frame.rowconfigure(row, weight=1, uniform="leg-row")
        name_row = ttk.Frame(frame)
        name_row.pack(fill="x", pady=5)
        ttk.Label(name_row, text="Snapshot name").pack(side="left")
        ttk.Entry(name_row, textvariable=self.snapshot_name).pack(side="left", fill="x", expand=True, padx=8)
        self.save_button = ttk.Button(name_row, text="Save targets", style="Accent.TButton", command=self.save)
        self.save_button.pack(side="right")
        load_row = ttk.Frame(frame)
        load_row.pack(fill="x", pady=5)
        self.combo = ttk.Combobox(load_row, textvariable=self.selected, state="readonly")
        self.combo.pack(side="left", fill="x", expand=True)
        self.combo.bind("<<ComboboxSelected>>", lambda _event: setattr(self, "selected_path", None))
        self.refresh_button = ttk.Button(load_row, text="Refresh", command=self.refresh_files)
        self.refresh_button.pack(side="left", padx=6)
        self.browse_button = ttk.Button(load_row, text="Browse...", command=self.browse_snapshot)
        self.browse_button.pack(side="left", padx=3)
        self.load_button = ttk.Button(load_row, text="Load Snapshot", command=self.load)
        self.load_button.pack(side="left")
        self.fixed_toggle = ttk.Checkbutton(
            frame, text="Fixed: keep torque holding the loaded targets", variable=self.hold_after_load)
        self.fixed_toggle.pack(anchor="w", pady=4)
        ttk.Label(frame, text="Both modes move to saved positions. Fixed off releases torque only after every joint arrives and holds stable.", wraplength=620).pack(anchor="w")
        self.release_all_button = ttk.Button(frame, text="Release all joints", command=self.release_all)
        self.release_all_button.pack(anchor="w", pady=8)
        self._build_status_box(frame, 620)

    def _build_status_box(self, parent, width):
        self.status_box = tk.Text(parent, height=3, wrap="word", state="disabled", width=width)
        self.status_box.pack(fill="x", pady=8)
        self.status.trace_add("write", lambda *_args: self._refresh_status())
        self.status_box.bind("<Control-a>", self._select_all_status)
        self.status_box.bind("<Control-c>", self._copy_status)
        self.status_box.bind("<Control-A>", self._select_all_status)
        self.status_box.bind("<Control-C>", self._copy_status)
        self._refresh_status()

    def _refresh_status(self):
        if self.status.get() == self._displayed_status:
            return
        self._displayed_status = self.status.get()
        self.status_box.configure(state="normal")
        self.status_box.delete("1.0", "end")
        self.status_box.insert("1.0", self.status.get())
        self.status_box.configure(state="disabled")

    def _select_all_status(self, _event=None):
        self.status_box.configure(state="normal")
        self.status_box.tag_add("sel", "1.0", "end-1c")
        self.status_box.configure(state="disabled")
        return "break"

    def _copy_status(self, _event=None):
        try:
            selected = self.status_box.get("sel.first", "sel.last")
        except tk.TclError:
            selected = self.status_box.get("1.0", "end-1c")
        self.root.clipboard_clear()
        self.root.clipboard_append(selected)
        return "break"

    def _configure(self, future):
        try:
            if not future.done():
                self.param_future = future
                return
            values = future.result().values
            ids, zeros, directions = [list(v.integer_array_value) for v in values[:3]]
            self.target_tolerance = values[3].double_value
            if not math.isfinite(self.target_tolerance) or self.target_tolerance < 0:
                raise ValueError("Controller returned invalid target tolerance")
            if not ids or len(ids) != len(zeros) or len(ids) != len(directions):
                raise ValueError("Controller returned incomplete configuration")
            self.joints = [f"servo_{i}" for i in ids]
            unknown_joints = sorted(set(self.joints) - set(LEG_FOR_JOINT))
            if unknown_joints:
                raise ValueError(f"No leg position is defined for {', '.join(unknown_joints)}")
            lower, upper = [list(v.integer_array_value) for v in values[4:6]]
            self.bounds = joint_bounds(ids, zeros, directions, lower, upper)
            for joint in self.joints:
                self.services[joint] = self.node.create_client(SetBool, f"/saprog/{joint}/torque")
            for leg_frame in self.leg_frames.values():
                for child in leg_frame.winfo_children():
                    child.destroy()
            self.current_labels.clear()
            self.target_values.clear()
            self.target_edited.clear()
            self.manual_buttons.clear()
            self.torque_buttons.clear()
            for joint in self.joints:
                leg_frame = self.leg_frames[LEG_FOR_JOINT[joint]]
                servo_row = ttk.Frame(leg_frame)
                servo_row.pack(fill="x", pady=3)
                heading = ttk.Frame(servo_row)
                heading.pack(fill="x")
                ttk.Label(heading, text=joint, width=9).pack(side="left")
                label = ttk.Label(heading, text="Waiting for telemetry", width=24, anchor="e")
                label.pack(side="left", fill="x", expand=True)
                self.current_labels[joint] = label
                low, high = self.bounds[joint]
                ttk.Label(servo_row, text=f"Limits: {low:.3f} .. {high:.3f} rad").pack(anchor="w")
                controls = ttk.Frame(servo_row)
                controls.pack(fill="x")
                ttk.Label(controls, text="Target (rad)").pack(side="left", padx=(0, 3))
                target = tk.StringVar()
                entry = ttk.Entry(controls, textvariable=target, width=10)
                entry.pack(side="left")
                entry.bind("<KeyRelease>", lambda _event, j=joint: self.target_edited.add(j))
                self.target_values[joint] = target
                move = ttk.Button(controls, text="Move", command=lambda j=joint: self.move_to_manual_value(j))
                move.pack(side="left", padx=3)
                self.manual_buttons[joint] = move
                toggle = ttk.Button(controls, text="Fix", style="Fix.TButton", command=lambda j=joint: self.toggle_torque(j))
                toggle.pack(side="left", padx=3)
                self.torque_buttons[joint] = toggle
            self.status.set(f"Configured {len(self.joints)} joints. Reading position and torque state.")
            self.refresh_files()
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
                        if joint not in self.pending and self.playback_state is None:
                            if enabled:
                                self.fixed.add(joint)
                                self.torque_state[joint] = "fixed"
                            else:
                                self.fixed.discard(joint)
                                self.torque_state[joint] = "released"
                    elif joint not in self.pending and self.playback_state is None:
                        self.fixed.discard(joint)
                        self.torque_state[joint] = "torque state unknown"
        self.positions, self.last_state = found, now
        valid = bool(self.joints) and set(found) == set(self.joints)
        self.latest_state_complete = valid
        playback = self.playback_state
        if playback is not None and playback["phase"] == "waiting":
            stamp = self.last_state_stamp
            source_age = (self.node.get_clock().now().nanoseconds - stamp) / 1_000_000_000 if stamp else float("inf")
            off_target = valid and any(
                abs(found[joint] - playback["targets"][joint]) > self.target_tolerance + AX_TICK_RADIANS / 2 for joint in self.joints)
            if not valid or off_target or source_age < 0.0 or source_age > FRESH_SECONDS:
                playback["stable_count"], playback["stable_since"], playback["stable_first_stamp"] = 0, None, 0
        for joint in self.joints:
            label = self.current_labels.get(joint)
            if label:
                value = found.get(joint)
                state = self.torque_state.get(joint, "torque unknown")
                configure_changed(label, text=f"{value:.3f} rad | {state}" if value is not None else "No current reading")
                if value is not None and joint not in self.target_edited:
                    self.target_values[joint].set(repr(value))
                    # Seed the editor once; subsequent telemetry belongs only
                    # in the live label, even after a Move command is sent.
                    self.target_edited.add(joint)
                self._update_joint_buttons(joint)

    def _complete_state(self):
        return bool(self.joints) and time.monotonic() - self.last_state <= FRESH_SECONDS and set(self.positions) == set(self.joints)

    def torque(self, joint, enabled):
        if self.playback_state is not None:
            self.status.set("Wait for the current snapshot operation to finish")
            return
        if joint in self.pending:
            return
        client = self.services[joint]
        if not client.service_is_ready():
            self.status.set(f"Torque service unavailable for {joint}")
            return
        req = SetBool.Request()
        req.data = enabled
        future = client.call_async(req)
        self.fixed.discard(joint)
        self.torque_state[joint] = "torque operation pending"
        token = object()
        self.pending[joint] = (token, time.monotonic())
        self._update_joint_buttons(joint)
        self.status.set(f"Waiting for {joint} torque operation")
        future.add_done_callback(lambda f, j=joint, e=enabled, t=token: self._torque_done(j, e, t, f))

    def _torque_done(self, joint, enabled, token, future):
        if joint not in self.pending or self.pending[joint][0] is not token:
            return
        del self.pending[joint]
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            if enabled:
                self.fixed.add(joint)
                self.torque_state[joint] = "fixed"
                self.status.set(f"{joint} holds its freshly read position; waiting for new telemetry")
                self._wait_post_fix[joint] = self.node.get_clock().now().nanoseconds
            else:
                self.fixed.discard(joint)
                self.torque_state[joint] = "released"
                self.status.set(f"{joint} released")
        except Exception as exc:
            self.torque_state[joint] = "torque state unknown"
            self.status.set(f"{joint} torque operation failed: {exc}")
        self._update_joint_buttons(joint)

    def toggle_torque(self, joint):
        if joint in self.pending:
            return
        self.torque(joint, joint not in self.fixed)

    def _update_joint_buttons(self, joint):
        button = self.torque_buttons.get(joint)
        if button is None:
            return
        pending = joint in self.pending or self.playback_state is not None
        is_fixed = joint in self.fixed
        configure_changed(button, text="..." if joint in self.pending else "Release" if is_fixed else "Fix",
                         style="Release.TButton" if is_fixed else "Fix.TButton",
                         state="disabled" if pending else "normal")
        manual = self.manual_buttons.get(joint)
        if manual is not None:
            configure_changed(manual, state="disabled" if pending else "normal")

    def move_to_manual_value(self, joint):
        if self.torque_strength.get() <= 0:
            self.status.set("Set Torque strength above zero before moving")
            return
        if self.playback_state is not None or joint in self.pending:
            self.status.set("Wait for the current snapshot or torque operation to finish")
            return
        if not self._complete_state():
            self.status.set("Manual move requires fresh readings for every configured joint")
            return
        try:
            value = to_radians(float(self.target_values[joint].get().strip()), self.bounds[joint], "radians")
        except ValueError as exc:
            self.status.set(f"{joint}: {exc}")
            return
        message = JointState()
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.name, message.position = [joint], [value]
        set_motion(message, self.moving_speed.get(), self.torque_strength.get())
        try:
            self.command_pub.publish(message)
        except Exception as exc:
            self.status.set(f"Could not send {joint} target: {exc}")
            return
        self.fixed.add(joint)
        self.torque_state[joint] = "moving to manual target"
        self.target_edited.add(joint)
        self.status.set(f"Sent {joint} target {value:.3f} rad; waiting for telemetry")
        self._update_joint_buttons(joint)

    @property
    def _wait_post_fix(self):
        if not hasattr(self, "_post_fix"):
            self._post_fix = {}
        return self._post_fix

    def release_all(self):
        for joint in self.joints:
            if joint not in self.pending:
                self.torque(joint, False)

    def refresh_files(self):
        try:
            os.makedirs(self.directory, exist_ok=True)
            files = sorted(p[:-5] for p in os.listdir(self.directory) if p.endswith(".json"))
            self.combo["values"] = files
        except OSError as exc:
            self.status.set(f"Snapshot directory error: {exc}")

    def browse_snapshot(self):
        path = filedialog.askopenfilename(
            title="Load snapshot", initialdir=self.directory,
            filetypes=(("JSON snapshots", "*.json"), ("All files", "*")))
        if path:
            self.selected_path = path
            self.selected.set(os.path.basename(path))
            self.status.set(f"Selected {path}; choose Load Snapshot to validate and send it")

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
            if os.path.exists(path) and not messagebox.askyesno("Overwrite snapshot?", f"Replace {os.path.basename(path)}?"):
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
        if not self.selected.get() or self.pending or self.playback_state is not None or not self._complete_state():
            self.status.set("Load requires a selected snapshot, fresh complete telemetry, and no torque operation")
            return
        path = self.selected_path or os.path.join(self.directory, os.path.basename(self.selected.get()) + ".json")
        try:
            with open(path, encoding="utf-8") as stream:
                name, positions = validate_snapshot(json.load(stream), self.joints, self.bounds)
        except Exception as exc:
            self.status.set(f"Snapshot rejected; no commands sent: {exc}")
            return
        release_after = not self.hold_after_load.get()
        if release_after and any(not self.services[joint].service_is_ready() for joint in self.joints):
            self.status.set("Cannot start: torque release services are not ready for every joint")
            return
        self.playback_state = {
            "name": name, "targets": positions,
            "queue": [(list(self.joints), [positions[joint] for joint in self.joints])],
            "release_after": release_after, "phase": "sending",
            "last_send": 0.0, "sent_stamp": 0, "deadline": None,
            "last_sample_stamp": 0, "stable_count": 0, "stable_since": None,
            "stable_first_stamp": 0,
            "release_pending": {}, "release_results": {},
        }
        self.fixed.clear()
        for joint in self.joints:
            self.torque_state[joint] = "torque state unknown after command"
        self._set_playback_controls(True)
        self.status.set(f"Sending commands for '{name}'...")

    def _set_playback_controls(self, locked):
        state = "disabled" if locked else "normal"
        for button in (self.save_button, self.refresh_button, self.browse_button,
                       self.load_button, self.release_all_button, self.fixed_toggle,
                       self.speed_scale, self.torque_scale):
            configure_changed(button, state=state)
        configure_changed(self.combo, state="disabled" if locked else "readonly")
        for joint in self.joints:
            self._update_joint_buttons(joint)

    def _finish_playback(self, message):
        self.status.set(message)
        self.playback_state = None
        self._set_playback_controls(False)

    def _release_playback_joints(self, playback):
        playback["phase"] = "releasing"
        for joint in self.joints:
            client = self.services[joint]
            if not client.service_is_ready():
                playback["release_results"][joint] = "service unavailable"
                continue
            request = SetBool.Request()
            request.data = False
            token = object()
            playback["release_pending"][joint] = (token, time.monotonic())
            try:
                future = client.call_async(request)
            except Exception as exc:
                del playback["release_pending"][joint]
                playback["release_results"][joint] = str(exc)
                continue
            future.add_done_callback(
                lambda done, j=joint, t=token, p=playback: self._playback_release_done(p, j, t, done))
        if not playback["release_pending"]:
            if playback["release_results"]:
                failures = playback["release_results"]
                self._finish_playback("Targets reached; torque release incomplete: " + "; ".join(
                    f"{joint}: {reason}" for joint, reason in failures.items()))
                return
            failures = playback["release_results"]
            self._finish_playback("Reached targets, but torque release failed: " + "; ".join(
                f"{joint}: {reason}" for joint, reason in failures.items()))
        else:
            self.status.set("All targets reached; waiting for acknowledged torque release on every joint")

    def _playback_release_done(self, playback, joint, token, future):
        if self.playback_state is not playback or joint not in playback["release_pending"]:
            return
        if playback["release_pending"][joint][0] is not token:
            return
        del playback["release_pending"][joint]
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            playback["release_results"][joint] = None
            self.torque_state[joint] = "released"
        except Exception as exc:
            playback["release_results"][joint] = str(exc)
        if playback["release_pending"]:
            return
        failures = {j: reason for j, reason in playback["release_results"].items() if reason}
        if failures:
            self._finish_playback("Targets reached; torque release incomplete: " + "; ".join(
                f"{joint}: {reason}" for joint, reason in failures.items()))
        else:
            self._finish_playback(f"Targets reached; torque release acknowledged for all {len(self.joints)} joints")

    def _playback_tick(self):
        playback = self.playback_state
        if playback is None:
            return
        now = time.monotonic()
        if playback["phase"] == "sending":
            if playback["queue"] and now - playback["last_send"] >= 0.12:
                joints, positions = playback["queue"].pop(0)
                message = JointState()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.name, message.position = joints, positions
                set_motion(message, self.moving_speed.get(), self.torque_strength.get())
                try:
                    self.command_pub.publish(message)
                except Exception as exc:
                    self._finish_playback(f"Command publication failed; torque was not released: {exc}")
                    return
                for joint, position in zip(joints, positions):
                    if joint in self.target_values:
                        self.target_values[joint].set(repr(position))
                        self.target_edited.add(joint)
                playback["last_send"] = now
                if not playback["queue"]:
                    playback["sent_stamp"] = self.node.get_clock().now().nanoseconds
                    playback["deadline"] = now + ARRIVAL_TIMEOUT
                    playback["phase"] = "waiting"
                    self.status.set("Commands sent; waiting for all joints to reach and stably hold their targets")
            return
        if playback["phase"] == "waiting":
            if now >= playback["deadline"]:
                self._finish_playback("Movement incomplete: " + playback.get("arrival_detail", "no fresh complete telemetry after command") + "; timed out. Torque unchanged.")
                return
            fresh = now - self.last_state <= FRESH_SECONDS
            if not fresh or not getattr(self, "latest_state_complete", False):
                playback["arrival_detail"] = "waiting for fresh readings from every configured joint"
                self.status.set(playback["arrival_detail"])
                playback["stable_count"], playback["stable_since"], playback["stable_first_stamp"] = 0, None, 0
                return
            stamp = self.last_state_stamp
            if not stamp or stamp <= playback["sent_stamp"]:
                playback["arrival_detail"] = "waiting for telemetry recorded after the command"
                self.status.set(playback["arrival_detail"])
                playback["stable_count"], playback["stable_since"], playback["stable_first_stamp"] = 0, None, 0
                return
            source_age = (self.node.get_clock().now().nanoseconds - stamp) / 1_000_000_000
            if source_age < 0.0 or source_age > FRESH_SECONDS:
                playback["arrival_detail"] = "telemetry timestamp is stale or ahead of the controller clock"
                self.status.set(playback["arrival_detail"])
                playback["stable_count"], playback["stable_since"], playback["stable_first_stamp"] = 0, None, 0
                return
            if stamp <= playback["last_sample_stamp"]:
                return
            playback["last_sample_stamp"] = stamp
            within, stalled, detail = arrival_progress(playback, self.positions, self.joints, self.target_tolerance, now)
            playback["arrival_detail"] = detail
            self.status.set(detail)
            if stalled:
                self._finish_playback("Movement incomplete: no progress toward targets for 8 seconds. " + detail + ". Check controller errors and torque strength. Torque unchanged.")
                return
            if not within:
                playback["stable_count"], playback["stable_since"], playback["stable_first_stamp"] = 0, None, 0
                return
            if playback["stable_since"] is None:
                playback["stable_since"] = now
                playback["stable_first_stamp"] = stamp
            playback["stable_count"] += 1
            source_span = (stamp - playback["stable_first_stamp"]) / 1_000_000_000
            if playback["stable_count"] >= STABLE_SAMPLES and now - playback["stable_since"] >= STABLE_SECONDS and source_span >= STABLE_SECONDS:
                if playback["release_after"]:
                    self._release_playback_joints(playback)
                else:
                    self._finish_playback(f"All targets reached and stable; torque remains holding all {len(self.joints)} joints")
            return
        if playback["phase"] == "releasing":
            for joint, (token, started) in list(playback["release_pending"].items()):
                if now - started > TORQUE_TIMEOUT:
                    del playback["release_pending"][joint]
                    playback["release_results"][joint] = "torque release timed out; state unknown"
                    self.torque_state[joint] = "torque state unknown"
            if not playback["release_pending"]:
                failures = {j: reason for j, reason in playback["release_results"].items() if reason}
                if failures:
                    self._finish_playback("Targets reached; torque release incomplete: " + "; ".join(
                        f"{joint}: {reason}" for joint, reason in failures.items()))
                else:
                    self._finish_playback(f"Targets reached; torque release acknowledged for all {len(self.joints)} joints")

    def _poll(self):
        if not self.joints:
            self.status.set(self.param_client.limits_error)
        if self.param_future is None and not self.joints and self.param_client.services_are_ready():
            self.param_future = self.param_client.get_parameters(
                ["servo_ids", "zero_positions", "directions", "target_tolerance", "lower_limits", "upper_limits"])
        if self.param_future is not None and self.param_future.done():
            future, self.param_future = self.param_future, None
            self._configure(future)
        for joint, (token, started) in list(self.pending.items()):
            if time.monotonic() - started > 8.0:
                del self.pending[joint]
                self.fixed.discard(joint)
                self.torque_state[joint] = "torque result timed out; state unknown"
                self.status.set(f"{joint} torque operation timed out; state is unknown")
        self._playback_tick()
        can_load = self.joints and self._complete_state() and not self.pending and self.playback_state is None
        if not self.playback_state:
            configure_changed(self.load_button, state="normal" if can_load else "disabled")
        self.root.after(30, self._poll)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", default=default_snapshot_directory())
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node("saprog_snapshot_window")
    root = tk.Tk()
    app = SnapshotWindow(root, node, os.path.abspath(os.path.expanduser(args.directory)))
    try:
        def spin():
            if rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                root.after(10, spin)
        root.after(10, spin)
        root.mainloop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
