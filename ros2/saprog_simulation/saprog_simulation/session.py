"""Own one isolated Gazebo server and an optional low-cost viewer."""

import copy
import importlib
import json
import math
import os
import signal
import subprocess
import tempfile
import threading
import time
import uuid
from pathlib import Path

from .profile import validate_bounds_profile
from .world import GUI_CONFIG, MODEL_NAME, WORLD_NAME, prepare_world


class GazeboSession:
    def __init__(self):
        self.partition = f"saprog_simulation_{uuid.uuid4().hex}"
        # This process is dedicated to simulation. Hardware ROS processes never
        # inherit this environment; only our Gazebo children share the partition.
        os.environ["GZ_PARTITION"] = self.partition
        self.server = self.viewer = self.transport = self.directory = None
        self.log_file = None
        self.joints = {}
        self.types = {}
        self.options = {}
        self.lock = threading.Lock()
        self.stats = {}
        self.stats_at = 0.0
        self.pose_state = {}
        self.closing = threading.Event()

    def _load_transport(self):
        try:
            transport = importlib.import_module("gz.transport13")
            for module, name in (
                ("world_stats", "WorldStatistics"), ("world_control", "WorldControl"),
                ("boolean", "Boolean"),
                ("pose_v", "Pose_V"),
            ):
                self.types[name] = getattr(importlib.import_module(f"gz.msgs10.{module}_pb2"), name)
        except ImportError as error:
            raise RuntimeError(
                "Gazebo Harmonic Python bindings are unavailable. Source ROS Jazzy and install "
                "ros-jazzy-gz-sim-vendor, ros-jazzy-gz-transport-vendor and ros-jazzy-gz-msgs-vendor."
            ) from error
        self.transport = transport.Node()
        options = transport.SubscribeOptions()
        options.msgs_per_sec = 2
        if not self.transport.subscribe(
            self.types["WorldStatistics"], f"/world/{WORLD_NAME}/stats", self._receive_stats, options,
        ):
            raise RuntimeError("Could not subscribe to Gazebo world statistics")
        self.transport.subscribe(
            self.types["Pose_V"], f"/world/{WORLD_NAME}/pose/info", self._receive_pose, options,
        )

    def _receive_pose(self, message):
        for pose in message.pose:
            if pose.name == MODEL_NAME:
                orientation = pose.orientation
                tilt_cos = 1 - 2 * (orientation.x ** 2 + orientation.y ** 2)
                with self.lock:
                    self.pose_state = {
                        "x": pose.position.x, "y": pose.position.y, "height": pose.position.z,
                        "tilt_degrees": math.degrees(math.acos(max(-1, min(1, tilt_cos)))),
                    }
                break

    def _receive_stats(self, message):
        with self.lock:
            self.stats = {
                "paused": message.paused, "sim_time": message.sim_time.sec + message.sim_time.nsec / 1e9,
                "real_time_factor": message.real_time_factor, "iterations": message.iterations,
            }
            self.stats_at = time.monotonic()

    def _launch(self, arguments):
        environment = dict(os.environ, GZ_PARTITION=self.partition, OMP_NUM_THREADS="1")
        # A separate group lets Stop reap the CLI wrapper AND its server/GUI.
        return subprocess.Popen(
            arguments, stdin=subprocess.DEVNULL, stdout=self.log_file, stderr=self.log_file,
            env=environment, start_new_session=True,
        )

    def start(self, options):
        options = copy.deepcopy(options)
        if "bounds_profile" in options:
            options["bounds_profile"] = validate_bounds_profile(options["bounds_profile"])
            if options.get("servo_bounds") is not None:
                raise ValueError("Choose bounds_profile or servo_bounds, not both")
        self.stop()
        if self.closing.is_set():
            raise RuntimeError("Simulation startup cancelled during shutdown")
        self.options = dict(options)
        self.directory = tempfile.TemporaryDirectory(prefix="saprog-gazebo-")
        try:
            world, self.joints = prepare_world(
                options["urdf_path"], self.directory.name,
                step_size=options.get("step_size", 0.004),
                real_time_factor=options.get("real_time_factor", 0.5),
                spawn_height=options.get("spawn_height", 0.3),
                servo_bounds=options.get("servo_bounds"),
                bounds_profile=options.get("bounds_profile"),
            )
            # Keep the resolved inputs alongside the generated world for this
            # experiment. Reset reuses this owned copy, not an edited source file.
            (Path(self.directory.name) / "run-inputs.json").write_text(
                json.dumps(options, indent=2, allow_nan=False), encoding="utf-8",
            )
            self.log_file = open(Path(self.directory.name) / "gazebo.log", "w", encoding="utf-8")
            self._load_transport()
            if self.closing.is_set():
                raise RuntimeError("Simulation startup cancelled during shutdown")
            self.server = self._launch(["gz", "sim", "-s", "-v", "2", str(world)])
            deadline = time.monotonic() + 25
            while time.monotonic() < deadline:
                if self.closing.is_set():
                    raise RuntimeError("Simulation startup cancelled during shutdown")
                if self.server.poll() is not None:
                    raise RuntimeError(f"Gazebo server exited: {self.log_tail()}")
                with self.lock:
                    ready = self.stats_at > 0
                if ready:
                    break
                time.sleep(0.1)
            else:
                raise RuntimeError(f"Gazebo did not publish world statistics: {self.log_tail()}")
            if options.get("viewer", False):
                self.show_viewer()
            self.pause(bool(options.get("paused", False)))
            return "URDF loaded into Gazebo; gravity is enabled"
        except Exception:
            self.stop()
            raise

    def _require_running(self):
        if self.server is None or self.server.poll() is not None:
            raise ValueError("Start the simulation first")

    def pause(self, paused):
        self._require_running()
        request = self.types["WorldControl"]()
        request.pause = paused
        # Initial mesh / physics loading can block Gazebo's update thread for
        # several seconds after its first statistics message. Pause/resume is
        # idempotent: retry briefly, yielding often enough for clean shutdown.
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if self.closing.is_set():
                raise RuntimeError("World control cancelled during shutdown")
            success, response = self.transport.request(
                f"/world/{WORLD_NAME}/control", request,
                self.types["WorldControl"], self.types["Boolean"], 750,
            )
            if success:
                if not response.data:
                    raise RuntimeError("Gazebo rejected world control")
                self.options["paused"] = paused
                return "Simulation paused" if paused else "Simulation running"
        raise RuntimeError("Gazebo did not acknowledge world control within 10 seconds")

    def reset(self):
        self._require_running()
        # Gazebo reset support differs across physics plugins and releases;
        # reimport to restore the complete initial model state consistently.
        return self.start(dict(self.options))

    def show_viewer(self):
        self._require_running()
        if self.viewer is None or self.viewer.poll() is not None:
            config = Path(self.directory.name) / "gui.config"
            config.write_text(GUI_CONFIG, encoding="utf-8")
            self.viewer = self._launch(["gz", "sim", "-g", "-v", "2", "--gui-config", str(config)])
        self.options["viewer"] = True
        return "Gazebo viewer requested"

    def hide_viewer(self):
        self._stop_process(self.viewer)
        self.viewer = None
        self.options["viewer"] = False
        return "Viewer closed; physics continues"

    def status(self):
        running = self.server is not None and self.server.poll() is None
        with self.lock:
            stats = dict(self.stats)
            stats.update(self.pose_state)
            stale = running and time.monotonic() - self.stats_at > 5
        return {
            **stats, "running": running, "stale": stale,
            "viewer": self.viewer is not None and self.viewer.poll() is None,
            "joints": self.joints, "gravity": -9.81,
            "bounds_profile": copy.deepcopy(self.options.get("bounds_profile")) if running else None,
        }

    def log_tail(self):
        if self.directory is None:
            return ""
        try:
            return (Path(self.directory.name) / "gazebo.log").read_text(
                encoding="utf-8", errors="replace",
            )[-2500:]
        except OSError:
            return ""

    @staticmethod
    def _stop_process(process):
        if process is None:
            return
        # Also reap children if the wrapper has already exited.
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            return
        try:
            process.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            pass
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait(timeout=2)

    def stop(self):
        self._stop_process(self.viewer)
        self._stop_process(self.server)
        self.viewer = self.server = None
        if self.transport is not None:
            self.transport.unsubscribe(f"/world/{WORLD_NAME}/stats")
            self.transport.unsubscribe(f"/world/{WORLD_NAME}/pose/info")
        self.transport = None
        if self.log_file is not None:
            self.log_file.close()
            self.log_file = None
        if self.directory is not None:
            self.directory.cleanup()
            self.directory = None
        with self.lock:
            self.stats = {}
            self.stats_at = 0.0
            self.pose_state = {}
        self.joints = {}
        return "Simulation stopped"
