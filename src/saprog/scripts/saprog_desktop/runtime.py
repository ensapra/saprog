"""ROS/Tk bootstrap and root-owned event-loop scheduling for desktop apps."""

from dataclasses import dataclass
import os
from pathlib import Path
import platform
import sys


@dataclass(frozen=True)
class DesktopDependencies:
    """Runtime modules and message/client types injected into desktop views."""

    rclpy: object
    Node: type
    AsyncParameterClient: type
    JointState: type
    SetBool: type
    tk: object
    ttk: object
    filedialog: object
    messagebox: object
    ActionClient: object = None
    PlayTimeline: object = None
    Imu: type = None
    MagneticField: type = None
    Temperature: type = None
    QuaternionStamped: type = None
    String: type = None
    ImuStatus: type = None
    Trigger: type = None
    qos_profile_sensor_data: object = None


def load_desktop_dependencies(args, require_timeline=False, entrypoint=None):
    """Load GUI/ROS dependencies, preserving the sourced-ROS relaunch fallback."""
    try:
        from geometry_msgs.msg import QuaternionStamped
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from saprog.msg import ImuStatus
        from sensor_msgs.msg import Imu, JointState, MagneticField, Temperature
        from rclpy.parameter_client import AsyncParameterClient
        from std_msgs.msg import String
        from std_srvs.srv import SetBool, Trigger
        import tkinter as tk
        from tkinter import filedialog, messagebox, ttk

        action_client = timeline_action = None
        if require_timeline:
            from rclpy.action import ActionClient as action_client
            from saprog.action import PlayTimeline as timeline_action

        os.environ.pop("SAPROG_SNAPSHOT_PLAYER_BOOTSTRAPPED", None)
        return DesktopDependencies(
            rclpy=rclpy,
            Node=Node,
            AsyncParameterClient=AsyncParameterClient,
            JointState=JointState,
            SetBool=SetBool,
            tk=tk,
            ttk=ttk,
            filedialog=filedialog,
            messagebox=messagebox,
            ActionClient=action_client,
            PlayTimeline=timeline_action,
            Imu=Imu,
            MagneticField=MagneticField,
            Temperature=Temperature,
            QuaternionStamped=QuaternionStamped,
            String=String,
            ImuStatus=ImuStatus,
            Trigger=Trigger,
            qos_profile_sensor_data=qos_profile_sensor_data,
        )
    except ImportError as error:
        missing = error

    marker = "SAPROG_SNAPSHOT_PLAYER_BOOTSTRAPPED"
    distro = os.environ.get("ROS_DISTRO") or "jazzy"
    setup = f"/opt/ros/{distro}/setup.bash"
    python = "/usr/bin/python3"
    if (
        platform.system() == "Linux"
        and marker not in os.environ
        and os.path.isfile(setup)
        and os.path.isfile(python)
        and os.access(python, os.X_OK)
    ):
        env = os.environ.copy()
        env[marker] = "1"
        command = (
            'if ! source "$1"; then '
            'echo "Could not source ROS setup: $1" >&2; exit 1; fi; '
            'exec "$2" "$3" "${@:4}"'
        )
        os.execvpe(
            "/bin/bash",
            [
                "bash",
                "-c",
                command,
                "saprog-desktop",
                setup,
                python,
                str(Path(entrypoint or sys.argv[0]).resolve()),
                *args,
            ],
            env,
        )

    needed = "ROS 2 Python packages and Tk"
    if require_timeline:
        needed += ", rclpy_action, and the built saprog action interface"
    raise SystemExit(
        f"Could not load desktop dependencies ({needed}). Build the workspace, install ROS 2 "
        f"{distro} with its Python packages and python3-tk, then source "
        f"{setup}. Details: {missing}"
    )


class RootRosScheduler:
    """Track root callbacks and spin registered nodes with a bounded fair budget."""

    def __init__(self, root, dependencies, nodes=(), interval_ms=10, spin_budget=16):
        if interval_ms < 1 or spin_budget < 1:
            raise ValueError("Scheduler interval and spin budget must be positive")
        self.root = root
        self.rclpy = dependencies.rclpy
        self.nodes = list(nodes)
        self.interval_ms = interval_ms
        self.spin_budget = spin_budget
        self._closed = False
        self._after_id = None
        self._pending_after = set()
        self._next_node = 0
        self._original_after = root.after
        self._original_after_cancel = root.after_cancel
        root.after = self._tracked_after
        root.after_cancel = self._tracked_after_cancel

    def add_node(self, node):
        if self._closed:
            raise RuntimeError("Cannot register a node after scheduler shutdown")
        if node not in self.nodes:
            self.nodes.append(node)

    def start(self):
        if self._closed or self._after_id is not None:
            return
        self._after_id = self._tracked_after(self.interval_ms, self._spin_once)

    def _tracked_after(self, delay_ms, callback=None, *args):
        if callback is None:
            return self._original_after(delay_ms)
        holder = {}

        def run():
            after_id = holder.get("id")
            if after_id is not None:
                self._pending_after.discard(after_id)
                if self._after_id == after_id:
                    self._after_id = None
            if not self._closed:
                callback(*args)

        after_id = self._original_after(delay_ms, run)
        holder["id"] = after_id
        self._pending_after.add(after_id)
        return after_id

    def _tracked_after_cancel(self, after_id):
        self._pending_after.discard(after_id)
        if self._after_id == after_id:
            self._after_id = None
        return self._original_after_cancel(after_id)

    def _spin_once(self):
        if self._closed or not self.rclpy.ok() or not self.nodes:
            return
        count = min(self.spin_budget, len(self.nodes))
        for _ in range(count):
            if self._closed or not self.rclpy.ok():
                break
            node = self.nodes[self._next_node % len(self.nodes)]
            self._next_node = (self._next_node + 1) % len(self.nodes)
            self.rclpy.spin_once(node, timeout_sec=0.0)
        if not self._closed and self.rclpy.ok():
            self._after_id = self._tracked_after(self.interval_ms, self._spin_once)

    def stop_callbacks(self):
        if self._closed:
            return
        self._closed = True
        for after_id in tuple(self._pending_after):
            try:
                self._original_after_cancel(after_id)
            except Exception:
                pass
        self._pending_after.clear()
        self._after_id = None

    def request_close(self):
        """Stop scheduled UI/ROS callbacks and close the Tk root window."""
        self.stop_callbacks()
        try:
            self.root.destroy()
        except Exception:
            pass

    def destroy_nodes(self):
        self.stop_callbacks()
        for node in reversed(self.nodes):
            node.destroy_node()
        self.nodes.clear()

    def shutdown(self):
        self.destroy_nodes()
        if self.rclpy.ok():
            self.rclpy.shutdown()
