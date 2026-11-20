"""Validated simulation plans and playback driven exclusively by simulated feedback."""

import json
import math
from pathlib import Path
from xml.etree import ElementTree as ET

from saprog_desktop.robot_view_model import SERVO_JOINT_NAMES
from saprog_desktop.snapshot_units import validate_snapshot
from saprog_desktop.timeline_domain import motion_settings, transition_duration
from saprog_desktop.timeline_storage import (
    load_snapshot_reference,
    load_timeline,
    resolve_timeline_steps,
)

SPEED_UNIT = 0.111 * 2 * math.pi / 60  # AX speed register unit, rad/s.


class SimulationRobot:
    def __init__(self, description):
        self.joints = {}
        self.mimics = {}
        for joint in ET.parse(description).getroot().findall("joint"):
            if joint.attrib["type"] == "fixed":
                continue
            name = joint.attrib["name"]
            limit = joint.find("limit")
            if joint.attrib["type"] != "revolute" or limit is None:
                raise ValueError(f"Simulation requires bounded revolute joints: {name}")
            values = tuple(float(limit.attrib[key]) for key in ("lower", "upper", "velocity", "effort"))
            if not all(math.isfinite(v) for v in values) or values[0] >= values[1] or min(values[2:]) <= 0:
                raise ValueError(f"Invalid simulation limits for {name}; set positive URDF velocity and effort")
            self.joints[name] = values
            mimic = joint.find("mimic")
            if mimic is not None:
                self.mimics[name] = (
                    mimic.attrib["joint"], float(mimic.get("multiplier", "1")), float(mimic.get("offset", "0")),
                )
        self.mapping = {f"servo_{servo}": joint for servo, joint in SERVO_JOINT_NAMES.items()}
        if set(self.mapping.values()) | set(self.mimics) != set(self.joints):
            raise ValueError("URDF joints do not match the configured robot and its mimic joints")
        if any(leader not in self.mapping.values() or not math.isfinite(ratio + offset)
               for leader, ratio, offset in self.mimics.values()):
            raise ValueError("Unsupported mimic chain or nonfinite mimic parameters")
        self.bounds = {servo: self.joints[joint][:2] for servo, joint in self.mapping.items()}

    def targets(self, positions, settings):
        targets = {}
        for servo, name in self.mapping.items():
            low, high, speed, effort = self.joints[name]
            target = positions[servo]
            if not low <= target <= high:
                raise ValueError(f"{servo}: target is outside URDF limits")
            targets[name] = (target, min(speed, settings[servo]["speed"] * SPEED_UNIT),
                             effort * settings[servo]["torque"] / 1023)
        for name, (leader, ratio, offset) in self.mimics.items():
            target, speed, effort = targets[leader]
            low, high, max_speed, max_effort = self.joints[name]
            target = ratio * target + offset
            if not low <= target <= high:
                raise ValueError(f"{name}: coupled target is outside URDF limits")
            targets[name] = (target, min(max_speed, abs(ratio) * speed) or max_speed,
                             min(max_effort, effort))
        return targets

    def load(self, path, kind, snapshots):
        path = Path(path)
        if kind == "snapshot":
            document = json.loads(path.read_text(encoding="utf-8"))
            name, positions = validate_snapshot(document, self.mapping, self.bounds)
            settings = {joint: {"speed": 300, "torque": 1023} for joint in self.mapping}
            return {"name": name, "steps": [{"targets": self.targets(positions, settings),
                                             "mode": "until_reached", "seconds": 0}],
                    "loop": False, "on_end": "on"}
        if kind != "sequence":
            raise ValueError("Select a snapshot or sequence")
        document = load_timeline(path, snapshots, self.mapping, self.bounds)
        steps = resolve_timeline_steps(document["steps"], lambda reference: load_snapshot_reference(
            reference, snapshots, self.mapping,
        ))
        plan = []
        for step in steps:
            if step.get("kind") == "wait":
                plan.append({"targets": None, "mode": "seconds", "seconds": step["seconds"]})
            else:
                settings = motion_settings(step, self.mapping)
                duration = transition_duration(step)
                plan.append({"targets": self.targets(step["snapshot"]["positions"], settings),
                             **duration})
                plan[-1]["mode"] = duration["mode"]
        return {"name": path.stem, "steps": plan, "loop": document.get("loop", False),
                "on_end": document.get("on_end", "on")}


class SimulationPlayback:
    def __init__(self, robot, send):
        self.robot = robot
        self.send = send
        self.plan = None
        self.positions = {}
        self.time = None
        self.index = 0
        self.message = "Ready. Select a snapshot or sequence."
        self.state = "idle"
        self.targets = None
        self._stable_since = None

    def start(self, plan):
        if self.plan is not None:
            raise ValueError("Stop the current playback first")
        if self.time is None or set(self.positions) != set(self.robot.joints):
            raise ValueError("Waiting for complete simulated joint feedback")
        self.plan = plan
        self.index = 0
        self.state = "playing"
        self._enter()

    def _enter(self):
        step = self.plan["steps"][self.index]
        self.started = self.time
        self._stable_since = None
        if step["targets"] is not None:
            self.targets = step["targets"]
            self.send(self.targets)
        self.message = f"{self.plan['name']}: step {self.index + 1}/{len(self.plan['steps'])} ({step['mode']})"

    def observe(self, now, positions):
        if not math.isfinite(now) or set(positions) != set(self.robot.joints) or not all(
            math.isfinite(value) for value in positions.values()
        ):
            return
        previous = self.time
        self.time, self.positions = now, dict(positions)
        if self.plan is None or previous is None or now == previous:
            return
        if now < previous:
            self.stop(release=True)
            self.state, self.message = "error", "World time reset; playback released. Start again."
            return
        step = self.plan["steps"][self.index]
        elapsed = now - self.started
        done = step["mode"] == "seconds" and elapsed >= step["seconds"]
        if step["mode"] == "until_reached":
            arrived = all(abs(positions[name] - target[0]) <= 0.04 for name, target in self.targets.items())
            if not arrived:
                self._stable_since = None
            elif self._stable_since is None:
                self._stable_since = now
            done = self._stable_since is not None and now - self._stable_since >= 0.2
            if not done and elapsed >= 20:
                self.stop(release=True)
                self.state, self.message = "error", "Arrival timed out after 20 simulated seconds; joints released."
                return
        if not done:
            return
        self.index += 1
        if self.index == len(self.plan["steps"]):
            if self.plan["loop"]:
                self.index = 0
            else:
                name, release = self.plan["name"], self.plan["on_end"] == "off"
                self.plan = None
                if release:
                    self.send(None)
                self.state = "complete"
                self.message = f"{name}: complete; joints {'released' if release else 'holding the final target'}."
                return
        self._enter()

    def stop(self, release=False):
        self.plan = None
        self.state = "idle"
        if release or not self.positions:
            self.send(None)
        else:
            hold = {name: (max(limits[0], min(limits[1], self.positions[name])), limits[2], limits[3])
                    for name, limits in self.robot.joints.items()}
            self.send(hold)
        self.message = "Joints released." if release else "Playback stopped; holding the current simulated pose."
