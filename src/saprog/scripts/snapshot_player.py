#!/usr/bin/env python3
"""Open the snapshot player directly with ``python3 snapshot_player.py``."""
import argparse
import json
import math
import os
from pathlib import Path
from snapshot_arrival import arrival_progress, AX_TICK_RADIANS
from snapshot_units import joint_bounds, validate_snapshot
from snapshot_files import default_snapshot_directory, write_json_atomic
import platform
import sys
import time
from desktop_style import BACKGROUND, apply_style, configure_changed
from motion_controls import add_slider, set_motion


FRESH_SECONDS = 2.5
TARGET_TOLERANCE = 0.05
STABLE_SAMPLES = 3
STABLE_SECONDS = 1.0
ARRIVAL_TIMEOUT = 30.0
TORQUE_TIMEOUT = 8.0


def player_settings_path():
    base = os.environ.get("APPDATA") if os.name == "nt" else os.environ.get("XDG_CONFIG_HOME")
    return (Path(base) if base else Path.home() / ".config") / "saprog" / "snapshot_player.json"


def load_torque_strength(path):
    try:
        settings = json.loads(path.read_text(encoding="utf-8"))
        value = settings.get("torque_strength") if isinstance(settings, dict) else None
        if type(value) is int and 0 <= value <= 1023:
            return value
    except (OSError, ValueError, UnicodeError):
        pass
    return 1023


def save_torque_strength(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    write_json_atomic(path, {"torque_strength": value})


class SnapshotPlayer:
    def __init__(self, root, node, directory):
        self.root, self.node, self.directory = root, node, directory
        self.joints, self.bounds = [], {}
        self.param_client = AsyncParameterClient(node, "/saprog_position_controller")
        self.param_future = None
        self.next_param_attempt = 0.0
        self.command_pub = node.create_publisher(JointState, "/saprog/position_command", 100)
        self.state_subscription = node.create_subscription(
            JointState, "/saprog/joint_states", self._on_state, 10)
        self.current_positions = {}
        self.last_state_stamp = 0
        self.last_state_received = 0.0
        self.latest_state_complete = False
        self.torque_clients = {}
        self.playback = None
        self.selected_path = None
        self.hold_after_load = tk.BooleanVar(value=True)
        self.settings_path = player_settings_path()
        self.torque_strength = tk.IntVar(value=load_torque_strength(self.settings_path))
        self.moving_speed = tk.IntVar(value=1023)
        self.torque_batch = None
        self.target_tolerance = TARGET_TOLERANCE
        self.buttons = []
        self._file_signature = None
        self._displayed_status = None
        self.status = tk.StringVar(value="Waiting for separately started saprog controller")
        self._build()
        self.refresh()
        self.root.after(100, self._tick)
        self.root.after(2000, self._periodic_refresh)

    def _build(self):
        apply_style(self.root)
        self.root.title("Snapshot Player")
        self.root.geometry("620x680")
        outer = ttk.Frame(self.root, padding=14)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="Snapshot Player", style="Title.TLabel").pack(anchor="w")
        actions = ttk.Frame(outer)
        actions.pack(fill="x", pady=(8, 4))
        self.refresh_button = ttk.Button(actions, text="Refresh", command=self.refresh)
        self.refresh_button.pack(side="right")
        self.browse_button = ttk.Button(actions, text="Browse...", command=self.browse_snapshot)
        self.browse_button.pack(side="right", padx=5)
        self.load_button = ttk.Button(actions, text="Load Snapshot", style="Accent.TButton", command=self.load_selected)
        self.load_button.pack(side="right", padx=5)
        self.hold_toggle = ttk.Checkbutton(
            outer, text="Fixed: keep torque holding the loaded targets", variable=self.hold_after_load)
        self.hold_toggle.pack(anchor="w", pady=4)
        ttk.Label(outer, text="Snapshot targets are in radians. Both modes move to saved positions. Fixed off releases torque only after every joint arrives and holds stable.", wraplength=560).pack(anchor="w")
        self.speed_scale = add_slider(outer, "Movement speed", self.moving_speed, 1)
        torque_row = ttk.Frame(outer)
        torque_row.pack(fill="x", pady=4)
        ttk.Label(torque_row, text="Torque strength", width=16).pack(side="left")
        self.torque_scale = ttk.Scale(torque_row, from_=0, to=1023, variable=self.torque_strength,
                                      command=self._change_torque_strength)
        self.torque_scale.pack(side="left", fill="x", expand=True, padx=8)
        self.torque_value = ttk.Label(torque_row, text=f"{self.torque_strength.get()} / 1023", width=12, anchor="e")
        self.torque_value.pack(side="left")
        self.disable_torque_button = ttk.Button(outer, text="Disable torque on all joints",
                                                command=self.disable_all_torque)
        self.disable_torque_button.pack(anchor="w", pady=(2, 6))
        ttk.Label(outer, text="Speed and strength apply to all joints on the next playback.",
                  style="Muted.TLabel").pack(anchor="w", pady=(0, 6))
        self.selected_label = ttk.Label(outer, text="No file selected", wraplength=560)
        self.selected_label.pack(anchor="w", pady=(0, 4))
        area = ttk.Frame(outer)
        area.pack(fill="both", expand=True)
        self.canvas = tk.Canvas(area, highlightthickness=0, background=BACKGROUND)
        scrollbar = ttk.Scrollbar(area, orient="vertical", command=self.canvas.yview)
        self.canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        self.canvas.pack(side="left", fill="both", expand=True)
        self.list_frame = ttk.Frame(self.canvas)
        self.canvas_window = self.canvas.create_window((0, 0), window=self.list_frame, anchor="nw")
        self.list_frame.bind("<Configure>", lambda _event: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.bind("<Configure>", lambda event: self.canvas.itemconfigure(self.canvas_window, width=event.width))
        self.status_box = tk.Text(outer, height=3, wrap="word", state="disabled", width=64)
        self.status_box.pack(fill="x", pady=(10, 0))
        self.status.trace_add("write", lambda *_args: self._refresh_status())
        self.status_box.bind("<Control-a>", self._select_all_status)
        self.status_box.bind("<Control-c>", self._copy_status)
        self.status_box.bind("<Control-A>", self._select_all_status)
        self.status_box.bind("<Control-C>", self._copy_status)
        self._refresh_status()

    def _change_torque_strength(self, value):
        strength = max(0, min(1023, round(float(value))))
        self.torque_strength.set(strength)
        self.torque_value.configure(text=f"{strength} / 1023")
        try:
            save_torque_strength(self.settings_path, strength)
        except OSError as exc:
            self.status.set(f"Torque strength changed, but could not save preference: {exc}")

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

    def _request_configuration(self):
        if self.joints or self.param_future is not None or time.monotonic() < self.next_param_attempt:
            return
        self.next_param_attempt = time.monotonic() + 1.0
        if not self.param_client.services_are_ready():
            self.status.set(self.param_client.limits_error)
            return
        self.param_future = self.param_client.get_parameters(
            ["servo_ids", "zero_positions", "directions", "target_tolerance", "lower_limits", "upper_limits"])

    def _read_configuration(self):
        future, self.param_future = self.param_future, None
        if not future.done():
            self.param_future = future
            return
        try:
            values = future.result().values
            if len(values) != 6:
                raise ValueError("controller returned incomplete configuration")
            ids, zeros, directions = (list(value.integer_array_value) for value in values[:3])
            self.target_tolerance = values[3].double_value
            if not math.isfinite(self.target_tolerance) or self.target_tolerance < 0:
                raise ValueError("controller returned invalid target tolerance")
            if not ids or len(ids) != len(zeros) or len(ids) != len(directions):
                raise ValueError("controller returned incomplete configuration")
            lower, upper = [list(v.integer_array_value) for v in values[4:6]]
            bounds = joint_bounds(ids, zeros, directions, lower, upper)
            joints = [f"servo_{servo}" for servo in ids]
            self.joints, self.bounds = joints, bounds
            self.torque_clients = {
                joint: self.node.create_client(SetBool, f"/saprog/{joint}/torque") for joint in joints}
            self.status.set(f"Configured {len(joints)} joints; waiting for command subscriber" if not self._ready() else "Ready")
        except Exception as exc:
            self.next_param_attempt = time.monotonic() + 1.0
            self.status.set(f"Controller configuration unavailable: {exc}")

    def _ready(self):
        return bool(self.joints) and self.node.count_subscribers("/saprog/position_command") > 0

    def refresh(self, force=True):
        if self.playback is not None:
            return
        try:
            paths = sorted(os.path.join(self.directory, filename)
                           for filename in os.listdir(self.directory) if filename.lower().endswith(".json"))
            signature = []
            for path in paths:
                try:
                    metadata = os.stat(path)
                    signature.append((path, metadata.st_mtime_ns, metadata.st_ctime_ns, metadata.st_size))
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
            ttk.Label(self.list_frame, text="No JSON snapshots found.").pack(anchor="w", pady=6)
        for path in paths:
            filename = os.path.basename(path)
            display_name = filename
            try:
                with open(path, encoding="utf-8") as stream:
                    doc = json.load(stream)
                if isinstance(doc, dict) and isinstance(doc.get("name"), str) and doc["name"].strip():
                    display_name = doc["name"].strip()
            except (OSError, ValueError, UnicodeError):
                pass
            button = ttk.Button(self.list_frame, text=f"{display_name}  -  {filename}",
                                command=lambda p=path: self.play(p))
            button.pack(fill="x", pady=3)
            self.buttons.append(button)
        self._update_buttons()

    def _update_buttons(self):
        ready = self._ready() and self.playback is None and self.torque_batch is None
        for button in self.buttons:
            configure_changed(button, state="normal" if ready else "disabled")
        configure_changed(self.load_button, state="normal" if ready and self.selected_path else "disabled")
        state = "disabled" if self.playback is not None or self.torque_batch is not None else "normal"
        for widget in (self.refresh_button, self.browse_button, self.hold_toggle,
                       self.torque_scale, self.speed_scale, self.disable_torque_button):
            configure_changed(widget, state=state)

    def disable_all_torque(self):
        if not self._ready() or self.playback is not None or self.torque_batch is not None:
            self.status.set("Wait for the controller and any current operation to finish")
            return
        unavailable = [joint for joint in self.joints if not self.torque_clients[joint].service_is_ready()]
        if unavailable:
            self.status.set("Torque services unavailable: " + ", ".join(unavailable))
            return
        self.torque_batch = {"pending": set(self.joints), "failed": {}}
        self._update_buttons()
        self.status.set("Disabling torque on all joints...")
        batch = self.torque_batch
        for joint in self.joints:
            request = SetBool.Request()
            request.data = False
            try:
                future = self.torque_clients[joint].call_async(request)
                future.add_done_callback(lambda done, j=joint, b=batch: self._disable_torque_done(b, j, done))
            except Exception as exc:
                self._disable_torque_failed(batch, joint, str(exc))

    def _disable_torque_failed(self, batch, joint, reason):
        if self.torque_batch is not batch or joint not in batch["pending"]:
            return
        batch["pending"].remove(joint)
        batch["failed"][joint] = reason
        self._finish_disable_torque(batch)

    def _disable_torque_done(self, batch, joint, future):
        if self.torque_batch is not batch or joint not in batch["pending"]:
            return
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            batch["pending"].remove(joint)
        except Exception as exc:
            self._disable_torque_failed(batch, joint, str(exc))
            return
        self._finish_disable_torque(batch)

    def _finish_disable_torque(self, batch):
        if self.torque_batch is not batch or batch["pending"]:
            return
        self.torque_batch = None
        self._update_buttons()
        if batch["failed"]:
            self.status.set("Torque disable incomplete: " + "; ".join(
                f"{joint}: {reason}" for joint, reason in batch["failed"].items()))
        else:
            self.status.set(f"Torque disabled on all {len(self.joints)} joints")

    def play(self, path):
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
            self.status.set("Set Torque strength above zero before playback; zero cannot drive the servos")
            return
        release_after = not self.hold_after_load.get()
        if release_after and any(not self.torque_clients[j].service_is_ready() for j in self.joints):
            self.status.set("Cannot start: torque release services are not ready for every joint")
            return
        self.playback = {
            "name": name, "targets": positions,
            "queue": [(list(self.joints), [positions[joint] for joint in self.joints])],
            "release_after": release_after, "phase": "sending", "last_send": 0.0,
            "sent_stamp": 0, "deadline": None, "last_sample_stamp": 0,
            "stable_count": 0, "stable_since": None, "stable_first_stamp": 0,
            "release_pending": {}, "release_results": {},
        }
        self._set_playback_controls(True)
        self.status.set(f"Sending commands for '{name}'...")

    def browse_snapshot(self):
        path = filedialog.askopenfilename(
            title="Load snapshot", initialdir=os.path.expanduser(self.directory),
            filetypes=(("JSON snapshots", "*.json"), ("All files", "*")))
        if path:
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
        self.last_state_stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        found = {}
        if len(message.name) == len(message.position):
            for joint, position in zip(message.name, message.position):
                if joint in self.joints and math.isfinite(position):
                    found[joint] = float(position)
        self.current_positions = found
        valid = bool(self.joints) and set(found) == set(self.joints)
        self.latest_state_complete = valid
        playback = self.playback
        if playback is not None and playback["phase"] == "waiting":
            stamp = self.last_state_stamp
            age = (self.node.get_clock().now().nanoseconds - stamp) / 1_000_000_000 if stamp else float("inf")
            off_target = valid and any(
                abs(found[joint] - playback["targets"][joint]) > self.target_tolerance + AX_TICK_RADIANS / 2 for joint in self._arrival_joints())
            if not valid or off_target or age < 0.0 or age > FRESH_SECONDS:
                playback["stable_count"], playback["stable_since"], playback["stable_first_stamp"] = 0, None, 0

    def _finish_playback(self, status, success=False):
        self.status.set(status)
        self.playback = None
        self._set_playback_controls(False)

    def _begin_release(self, playback):
        playback["phase"] = "releasing"
        for joint in self.joints:
            client = self.torque_clients[joint]
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
                lambda done, j=joint, t=token, p=playback: self._release_done(p, j, t, done))
        if not playback["release_pending"]:
            failures = playback["release_results"]
            self._finish_playback("Targets reached; torque release incomplete: " + "; ".join(
                f"{joint}: {reason}" for joint, reason in failures.items()))
        else:
            self.status.set("All targets reached; waiting for acknowledged torque release on every joint")

    def _release_done(self, playback, joint, token, future):
        if self.playback is not playback or joint not in playback["release_pending"]:
            return
        if playback["release_pending"][joint][0] is not token:
            return
        del playback["release_pending"][joint]
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            playback["release_results"][joint] = None
        except Exception as exc:
            playback["release_results"][joint] = str(exc)
        if not playback["release_pending"]:
            self._complete_release(playback)

    def _complete_release(self, playback):
        failures = {j: reason for j, reason in playback["release_results"].items() if reason}
        if failures:
            self._finish_playback("Targets reached; torque release incomplete: " + "; ".join(
                f"{joint}: {reason}" for joint, reason in failures.items()))
        else:
            self._finish_playback(f"Targets reached; torque release acknowledged for all {len(self.joints)} joints")

    def _apply_motion(self, message):
        set_motion(message, self.moving_speed.get(), self.torque_strength.get())

    def _targets_reached(self):
        self._finish_playback(f"All targets reached and stable; torque remains holding all {len(self.joints)} joints")

    def _arrival_joints(self):
        return self.joints

    def _requires_stable_arrival(self):
        return True

    def _command_interval(self):
        return 0.12

    def _playback_tick(self):
        playback = self.playback
        if playback is None:
            return
        now = time.monotonic()
        if playback["phase"] == "sending":
            while playback["queue"] and now - playback["last_send"] >= self._command_interval():
                joints, positions = playback["queue"].pop(0)
                message = JointState()
                message.header.stamp = self.node.get_clock().now().to_msg()
                message.name, message.position = joints, positions
                self._apply_motion(message)
                try:
                    self.command_pub.publish(message)
                except Exception as exc:
                    self._finish_playback(f"Synchronized command publication failed; torque was not released: {exc}")
                    return
                playback["last_send"] = now
                if not playback["queue"]:
                    playback["sent_stamp"] = self.node.get_clock().now().nanoseconds
                    playback["deadline"] = now + ARRIVAL_TIMEOUT
                    playback["phase"] = "waiting"
                    self.status.set("Commands sent; waiting for all joints to reach their targets" +
                                    (" and hold stable" if self._requires_stable_arrival() else ""))
            return
        if playback["phase"] == "waiting":
            if now >= playback["deadline"]:
                self._finish_playback("Movement incomplete: " + playback.get("arrival_detail", "no fresh complete telemetry after command") + "; timed out. Torque unchanged.")
                return
            if now - self.last_state_received > FRESH_SECONDS or not self.latest_state_complete:
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
            within, stalled, detail = arrival_progress(playback, self.current_positions, self._arrival_joints(), self.target_tolerance, now)
            playback["arrival_detail"] = detail
            self.status.set(detail)
            if stalled:
                self._finish_playback("Movement incomplete: no progress toward targets for 8 seconds. " + detail + ". Check controller errors and torque strength. Torque unchanged.")
                return
            if not within:
                playback["stable_count"], playback["stable_since"], playback["stable_first_stamp"] = 0, None, 0
                return
            if not self._requires_stable_arrival():
                self._targets_reached()
                return
            if playback["stable_since"] is None:
                playback["stable_since"] = now
                playback["stable_first_stamp"] = stamp
            playback["stable_count"] += 1
            source_span = (stamp - playback["stable_first_stamp"]) / 1_000_000_000
            if playback["stable_count"] >= STABLE_SAMPLES and now - playback["stable_since"] >= STABLE_SECONDS and source_span >= STABLE_SECONDS:
                if playback["release_after"]:
                    self._begin_release(playback)
                else:
                    self._targets_reached()
            return
        if playback["phase"] == "releasing":
            for joint, (token, started) in list(playback["release_pending"].items()):
                if now - started > TORQUE_TIMEOUT:
                    del playback["release_pending"][joint]
                    playback["release_results"][joint] = "torque release timed out; state unknown"
            if not playback["release_pending"]:
                self._complete_release(playback)
        self._update_buttons()

    def _tick(self):
        if self.param_future is not None:
            self._read_configuration()
        else:
            self._request_configuration()
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


def _load_runtime(args, require_timeline=False):
    """Load ROS/Tk, sourcing the system ROS setup once when needed."""
    global rclpy, Node, AsyncParameterClient, JointState, SetBool
    global ActionClient, PlayTimeline, tk, ttk, filedialog
    try:
        import rclpy as rclpy_module
        from rclpy.node import Node as RosNode
        from snapshot_limits_client import HardwareParameterClient as ParameterClient
        from sensor_msgs.msg import JointState as JointStateMessage
        from std_srvs.srv import SetBool as SetBoolService
        import tkinter as tkinter_module
        from tkinter import filedialog as filedialog_module, ttk as ttk_module
        rclpy, Node = rclpy_module, RosNode
        AsyncParameterClient, JointState = ParameterClient, JointStateMessage
        SetBool = SetBoolService
        if require_timeline:
            from rclpy.action import ActionClient as RosActionClient
            from saprog.action import PlayTimeline as TimelineAction
            ActionClient, PlayTimeline = RosActionClient, TimelineAction
        tk, ttk, filedialog = tkinter_module, ttk_module, filedialog_module
        os.environ.pop("SAPROG_SNAPSHOT_PLAYER_BOOTSTRAPPED", None)
        return
    except ImportError as exc:
        missing = exc

    marker = "SAPROG_SNAPSHOT_PLAYER_BOOTSTRAPPED"
    if platform.system() == "Linux" and marker not in os.environ:
        distro = os.environ.get("ROS_DISTRO") or "jazzy"
        setup = f"/opt/ros/{distro}/setup.bash"
        python = "/usr/bin/python3"
        if os.path.isfile(setup) and os.path.isfile(python) and os.access(python, os.X_OK):
            env = os.environ.copy()
            env[marker] = "1"
            command = (
                'if ! source "$1"; then '
                'echo "Could not source ROS setup: $1" >&2; exit 1; fi; '
                'exec "$2" "$3" "${@:4}"'
            )
            os.execvpe("/bin/bash", ["bash", "-c", command, "snapshot-player", setup,
                                      python, os.path.abspath(__file__), *args], env)
    detail = str(missing)
    distro = os.environ.get("ROS_DISTRO") or "jazzy"
    dependencies = "rclpy, sensor_msgs, and Tk"
    if require_timeline:
        dependencies += ", rclpy_action, and the built saprog action interface"
    raise SystemExit(
        f"Could not load the snapshot player dependencies ({dependencies}). "
        f"Build the workspace, install ROS 2 {distro} with its Python packages and python3-tk, then source "
        f"/opt/ros/{distro}/setup.bash. Details: {detail}"
    )


def main():
    parser = argparse.ArgumentParser(
        description="Open the saprog snapshot player. A separately started saprog controller is required to send commands."
    )
    parser.add_argument("--directory", default=default_snapshot_directory())
    parser.add_argument("--timeline", action="store_true", help="Open the timeline editor")
    args, ros_args = parser.parse_known_args()
    _load_runtime(sys.argv[1:], require_timeline=args.timeline)
    rclpy.init(args=ros_args)
    node = Node("saprog_snapshot_player")
    root = tk.Tk()
    player_class = SnapshotPlayer
    if args.timeline:
        sys.modules["snapshot_player"] = sys.modules[__name__]
        from timeline_window import TimelineWindow
        player_class = TimelineWindow
    player_class(root, node, os.path.abspath(os.path.expanduser(args.directory)))
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
