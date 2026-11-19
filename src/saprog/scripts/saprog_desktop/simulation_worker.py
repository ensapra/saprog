"""Gazebo-only playback worker; communicates with Tk through a private session directory."""

import argparse
import json
import signal
import threading
import time
from pathlib import Path

from saprog_desktop.simulation_playback import SimulationPlayback, SimulationRobot
from saprog_desktop.snapshot_files import write_json_atomic


class GazeboPlaybackWorker:
    def __init__(self, directory, description, plugin):
        from gz.msgs10.boolean_pb2 import Boolean
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.entity_pb2 import Entity
        from gz.msgs10.entity_plugin_v_pb2 import EntityPlugin_V
        from gz.msgs10.joint_trajectory_pb2 import JointTrajectory
        from gz.msgs10.scene_pb2 import Scene
        from gz.msgs10.world_control_pb2 import WorldControl
        from gz.msgs10.world_stats_pb2 import WorldStatistics
        from gz.transport13 import Node

        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.node = Node()
        self.message_type = JointTrajectory
        self.publisher = self.node.advertise("/saprog_simulation/targets", JointTrajectory)
        self.feedback = None
        self.feedback_at = time.monotonic()
        self.paused = True
        self.lock = threading.Lock()
        self.robot = SimulationRobot(description)
        self.playback = SimulationPlayback(self.robot, self.send)
        self.node.subscribe(JointTrajectory, "/saprog_simulation/feedback", self.receive)
        self.node.subscribe(WorldStatistics, "/world/saprog_simulation/stats", self.statistics)
        self.last_request = None
        deadline = time.monotonic() + 30
        model = None
        while time.monotonic() < deadline:
            ok, scene = self.node.request("/world/saprog_simulation/scene/info", Empty(), Empty, Scene, 1000)
            if ok:
                model = next((entry for entry in scene.model if entry.name == "saprog"), None)
                if model is not None:
                    break
            time.sleep(0.1)
        if model is None:
            raise RuntimeError("Gazebo did not create the robot")
        request = EntityPlugin_V()
        request.entity.id, request.entity.type = model.id, Entity.MODEL
        request.plugins.add(name="saprog_sim::Actuators", filename=str(plugin))
        ok, response = self.node.request("/world/saprog_simulation/entity/system/add", request,
                                         EntityPlugin_V, Boolean, 5000)
        if not ok or not response.data:
            raise RuntimeError(f"Gazebo actuator attachment failed (transport={ok}, accepted={response.data}); rebuild saprog_simulation")
        # Two paused steps initialize the plugin and physics joint feedback arrays.
        self.node.request("/world/saprog_simulation/control", WorldControl(pause=True, multi_step=2),
                          WorldControl, Boolean, 5000)
        self.feedback_at = time.monotonic()

    def receive(self, message):
        if not message.points:
            return
        now = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
        positions = dict(zip(message.joint_names, message.points[0].positions))
        with self.lock:
            self.feedback = now, positions
            self.feedback_at = time.monotonic()

    def statistics(self, message):
        with self.lock:
            self.paused = message.paused

    def send(self, targets):
        message = self.message_type()
        if targets is not None:
            point = message.points.add()
            for name, (position, speed, effort) in targets.items():
                message.joint_names.append(name)
                point.positions.append(position)
                point.velocities.append(speed)
                point.effort.append(effort)
        if not self.publisher.publish(message):
            raise RuntimeError("Could not publish simulated joint targets")

    def tick(self):
        with self.lock:
            feedback, received_at, paused = self.feedback, self.feedback_at, self.paused
        if feedback is not None:
            self.playback.observe(*feedback)
        path = self.directory / "command.json"
        if path.exists():
            command = json.loads(path.read_text(encoding="utf-8"))
            if command["id"] != self.last_request:
                self.last_request = command["id"]
                try:
                    action = command["action"]
                    if action == "play":
                        # Fully load and validate references/targets before any actuator command.
                        plan = self.robot.load(command["path"], command["kind"], command["snapshots"])
                        self.playback.start(plan)
                    elif action in ("stop", "release"):
                        self.playback.stop(release=action == "release")
                    else:
                        raise ValueError("Unknown simulation action")
                except (OSError, ValueError, KeyError, TypeError) as error:
                    self.playback.message = f"Cannot play: {error}"
        if time.monotonic() - received_at > (10 if feedback is None else 3):
            self.playback.stop(release=True)
            raise RuntimeError("Simulated joint feedback stopped; playback released")
        ready = self.playback.time is not None and set(self.playback.positions) == set(self.robot.joints)
        status = {"ready": ready, "state": self.playback.state, "message": self.playback.message,
                  "paused": paused, "sim_time": self.playback.time, "positions": self.playback.positions,
                  "request": self.last_request}
        write_json_atomic(self.directory / "playback.json", status)
        return ready


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("directory", type=Path)
    parser.add_argument("description", type=Path)
    parser.add_argument("plugin", type=Path)
    parser.add_argument("--ready-file", default="")
    args = parser.parse_args()
    running = True

    def stop(*_):
        nonlocal running
        running = False

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    worker = None
    try:
        worker = GazeboPlaybackWorker(args.directory, args.description, args.plugin)
        deadline = time.monotonic() + 10
        while running:
            ready = worker.tick()
            if ready and args.ready_file:
                Path(args.ready_file).touch()
                args.ready_file = ""
            if not ready and time.monotonic() > deadline:
                raise RuntimeError("Waiting for simulated joint feedback timed out")
            time.sleep(0.05)
    except Exception as error:
        write_json_atomic(args.directory / "playback.json", {"ready": False, "state": "error", "message": str(error)})
        raise
    finally:
        if worker is not None:
            worker.send(None)


if __name__ == "__main__":
    main()
