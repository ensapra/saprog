"""Single-leg URDF kinematics in base coordinates; no ROS or hardware commands."""

import copy
import math
import re
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType

import numpy as np
from documents.timeline_domain import mirrored_positions, transition_duration
from documents.timeline_storage import load_snapshot_reference, load_timeline

ROOT = Path(__file__).resolve().parents[3]
DESCRIPTION = ROOT / "urdf"
LEGS = {"front_right": (11, 12, 13), "back_right": (21, 22, 23),
        "front_left": (31, 32, 33), "back_left": (41, 42, 43)}


def vector(text, default="0 0 0"):
    values = np.array([float(v) for v in (text or default).split()])
    if values.shape != (3,) or not np.isfinite(values).all():
        raise ValueError("URDF vectors must contain three finite numbers")
    return values


def rotation(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-12:
        raise ValueError("Joint axis cannot be zero")
    x, y, z = axis / norm
    skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
    return np.eye(3) + math.sin(angle) * skew + (1 - math.cos(angle)) * (skew @ skew)


def origin(element):
    result = np.eye(4)
    if element is not None:
        r, p, y = vector(element.get("rpy"))
        result[:3, :3] = (rotation((0, 0, 1), y) @ rotation((0, 1, 0), p)
                           @ rotation((1, 0, 0), r))
        result[:3, 3] = vector(element.get("xyz"))
    return result


def mesh_path(filename, urdf_path):
    prefix = "package://saprog_description/"
    if filename.startswith(prefix):
        return DESCRIPTION / filename[len(prefix):]
    if filename.startswith("package://"):
        raise ValueError(f"Unsupported mesh package: {filename}")
    return (urdf_path.parent / filename).resolve()


def stl_triangles(path):
    """Read repository binary STL assets without another rendering dependency."""
    data = path.read_bytes()
    if len(data) < 84:
        raise ValueError(f"Invalid binary STL: {path.name}")
    count = int.from_bytes(data[80:84], "little")
    if len(data) != 84 + count * 50 or count == 0:
        raise ValueError(f"Expected binary STL: {path.name}")
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)),
                      ("attribute", "<u2")])
    triangles = np.frombuffer(data, dtype=dtype, offset=84)["vertices"].astype(float)
    if not np.isfinite(triangles).all():
        raise ValueError("Mesh contains non-finite vertices")
    return triangles


@dataclass
class Mapping:
    joint: str
    servo: str
    gain: float
    offset: float


class SingleLegModel:
    def __init__(self, path, leg="front_right", bounds=None):
        self.path, self.leg = Path(path).resolve(), leg
        self.bounds = bounds if bounds is not None else MappingProxyType({})
        if leg not in LEGS:
            raise ValueError("Unknown leg")
        tree = ET.parse(self.path).getroot()
        selected = [copy.deepcopy(j) for j in tree.findall("joint")
                    if j.get("name", "").startswith(leg + "_")]
        ids = LEGS[leg]
        names = (f"{leg}_{ids[0]}", f"{leg}_{ids[1]}", f"{leg}_joint_{ids[2]}")
        by_name = {j.get("name"): j for j in selected}
        if not set((*names, leg + "_leg")) <= by_name.keys():
            raise ValueError(f"URDF is missing the three servo joints or knee for {leg}")
        self.knee = by_name[leg + "_leg"]
        mimic = self.knee.find("mimic")
        if mimic is None or mimic.get("joint") != names[2]:
            raise ValueError("The knee must mimic this leg's crank joint")
        self.multiplier = float(mimic.get("multiplier", "1"))
        self.knee_offset = float(mimic.get("offset", "0"))
        child_links = {j.find("child").get("link") for j in selected}
        roots = {j.find("parent").get("link") for j in selected} - child_links
        if len(roots) != 1:
            raise ValueError("Single leg must have exactly one fixed root link")
        self.root_link = roots.pop()
        self.links = {link.get("name"): copy.deepcopy(link) for link in tree.findall("link")
                      if link.get("name") in child_links}
        if len(self.links) != len(child_links):
            raise ValueError("URDF joint references a missing link")
        self.joints = []
        connected = {self.root_link}
        while selected:
            ready = [j for j in selected if j.find("parent").get("link") in connected]
            if not ready:
                raise ValueError("URDF contains a disconnected or cyclic joint tree")
            for joint in ready:
                if joint.get("type") not in ("revolute", "fixed", "continuous"):
                    raise ValueError("Only fixed and rotary joints are supported")
                self.joints.append(joint)
                connected.add(joint.find("child").get("link"))
                selected.remove(joint)
        # URDF origins and axes are fixed. Parse them once, outside the animation/IK loop.
        self._frames = []
        for joint in self.joints:
            axis = None
            if joint.get("type") != "fixed":
                axis_element = joint.find("axis")
                axis = vector(axis_element.get("xyz") if axis_element is not None else "1 0 0")
                norm = np.linalg.norm(axis)
                if norm < 1e-12:
                    raise ValueError("Joint axis cannot be zero")
                axis /= norm
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                skew_squared = skew @ skew
            else:
                skew = skew_squared = None
            self._frames.append((joint.get("name"), joint.find("parent").get("link"),
                                 joint.find("child").get("link"), origin(joint.find("origin")),
                                 axis, skew, skew_squared))
        self.mappings = []
        for name, servo_id in zip(names, ids):
            servo = f"servo_{servo_id}"
            self.mappings.append(Mapping(name, servo, 1., 0.))
        self.visuals = []
        for name, link in self.links.items():
            for visual in link.findall("visual"):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    continue
                triangles = stl_triangles(mesh_path(mesh.get("filename"), self.path))
                triangles *= vector(mesh.get("scale"), "1 1 1")
                transform = origin(visual.find("origin"))
                triangles = triangles @ transform[:3, :3].T + transform[:3, 3]
                color = visual.find("material/color")
                rgba = [float(c) for c in color.get("rgba").split()] if color is not None else [.6]*3
                self.visuals.append((name, triangles, rgba[:3]))
        self.foot_link = self.knee.find("child").get("link")
        self._foot_chain = []
        by_child = {frame[2]: frame for frame in self._frames}
        ancestor = self.foot_link
        while ancestor != self.root_link:
            frame = by_child[ancestor]
            self._foot_chain.append(frame)
            ancestor = frame[1]
        foot_mesh = [t.reshape(-1, 3) for name, t, _ in self.visuals if name == self.foot_link]
        if not foot_mesh:
            raise ValueError("Lower leg needs an STL visual to estimate the foot point")
        vertices = np.concatenate(foot_mesh)
        distances = np.linalg.norm(vertices, axis=1)
        self.foot = vertices[distances >= np.quantile(distances, .99)].mean(axis=0)
        self.validate()

    def validate(self):
        if len({m.servo for m in self.mappings}) != 3:
            raise ValueError("Map each joint to a different servo")
        for m in self.mappings:
            if not re.fullmatch(r"servo_\d+", m.servo):
                raise ValueError("Mapping needs servo_N names")
            if not all(math.isfinite(v) for v in (m.gain, m.offset)) or abs(m.gain) < .01:
                raise ValueError("Use a nonzero finite calibration gain")
        if (not np.isfinite(self.foot).all() or not math.isfinite(self.multiplier)
                or not math.isfinite(self.knee_offset)):
            raise ValueError("Foot and knee calibration must be finite")

    def joint_angles(self, values):
        angles = {m.joint: m.gain * float(values[m.servo]) + m.offset for m in self.mappings}
        angles[self.knee.get("name")] = self.multiplier * angles[self.mappings[2].joint] + self.knee_offset
        return angles

    def transforms(self, values):
        angles = self.joint_angles(values)
        transforms = {self.root_link: np.eye(4)}
        for name, parent, child, fixed, axis, skew, squared in self._frames:
            local = fixed.copy()
            if axis is not None:
                angle = angles.get(name, 0.)
                local[:3, :3] = fixed[:3, :3] @ (
                    np.eye(3) + math.sin(angle)*skew + (1-math.cos(angle))*squared)
            transforms[child] = transforms[parent] @ local
        return transforms, angles

    def foot_position(self, values):
        transforms, _ = self.transforms(values)
        t = transforms[self.foot_link]
        return t[:3, :3] @ self.foot + t[:3, 3]

    def in_limits(self, values):
        return all(m.servo in self.bounds and
                   self.bounds[m.servo][0]-1e-9 <= values[m.servo] <= self.bounds[m.servo][1]+1e-9
                   for m in self.mappings)

    def foot_kinematics(self, values):
        """Foot position and exact servo Jacobian, including the passive knee coupling."""
        transforms, _ = self.transforms(values)
        foot_transform = transforms[self.foot_link]
        point = foot_transform[:3, :3] @ self.foot + foot_transform[:3, 3]
        jacobian = np.zeros((3, 3))
        driven = {m.joint: (index, m.gain) for index, m in enumerate(self.mappings)}
        for name, parent, _child, fixed, axis, _skew, _squared in self._foot_chain:
            if axis is None:
                continue
            frame = transforms[parent] @ fixed
            derivative = np.cross(frame[:3, :3] @ axis, point-frame[:3, 3])
            if name in driven:
                index, gain = driven[name]
                jacobian[:, index] += derivative*gain
            if name == self.knee.get("name"):
                jacobian[:, 2] += derivative*self.multiplier*self.mappings[2].gain
        return point, jacobian

    def solve(self, target, seed, tolerance=0.0001):
        """Damped numerical IK, all three servos with the URDF knee coupling."""
        target = np.asarray(target, dtype=float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("Foot target must contain three finite coordinates")
        q = np.array([seed[m.servo] for m in self.mappings], dtype=float)
        if any(m.servo not in self.bounds for m in self.mappings):
            raise ValueError("Current EEPROM limits are required before solving a foot path")
        low = np.array([self.bounds[m.servo][0] for m in self.mappings])
        high = np.array([self.bounds[m.servo][1] for m in self.mappings])
        def values(q):
            return {m.servo: float(v) for m, v in zip(self.mappings, q)}
        q = np.clip(q, low, high)
        for _ in range(160):
            pose = values(q)
            point, jac = self.foot_kinematics(pose)
            error = target - point
            if np.linalg.norm(error) <= tolerance and self.in_limits(pose):
                return pose
            delta = jac.T @ np.linalg.solve(jac @ jac.T + np.eye(3) * 1e-7, error)
            q = np.clip(q + np.clip(delta, -.12, .12), low, high)
        raise ValueError("Foot path is unreachable within calibrated limits; reduce stroke or move the centre")

    def vertical_cycle(self, center, stroke, seed, samples=81):
        if not math.isfinite(stroke) or stroke <= 0:
            raise ValueError("Stroke must be positive and finite")
        poses, targets, current = [], [], dict(seed)
        for phase in np.linspace(0, 2 * math.pi, samples):
            target = np.array(center, dtype=float) + [0, 0, stroke / 2 * math.sin(phase)]
            current = self.solve(target, current)
            poses.append(current)
            targets.append(target)
        return poses, targets

    def export_urdf(self, path):
        tree = ET.Element("robot", name="saprog_" + self.leg)
        ET.SubElement(tree, "link", name=self.root_link)
        for link in self.links.values():
            link = copy.deepcopy(link)
            for mesh in link.findall(".//mesh"):
                mesh.set("filename", str(mesh_path(mesh.get("filename"), self.path)))
            tree.append(link)
        for joint in self.joints:
            joint = copy.deepcopy(joint)
            if joint.get("type") != "fixed":
                joint.set("type", "continuous")
                limit = joint.find("limit")
                if limit is not None:
                    limit.attrib.pop("lower", None)
                    limit.attrib.pop("upper", None)
            tree.append(joint)
        mimic = tree.find(f"joint[@name='{self.knee.get('name')}']/mimic")
        mimic.set("multiplier", str(self.multiplier))
        mimic.set("offset", str(self.knee_offset))
        ET.indent(tree)
        ET.ElementTree(tree).write(path, encoding="utf-8", xml_declaration=True)


def sequence_frames(path, servos, seed, bounds=None):
    """Resolve canonical timeline references and return timed selected-leg targets."""
    document = load_timeline(path, ROOT / "snapshots")
    frames, current = [], dict(seed)
    for step in document["steps"]:
        if step.get("kind") == "wait":
            seconds = step["seconds"]
        else:
            duration = transition_duration(step)
            if duration["mode"] != "seconds":
                raise ValueError("Preview requires fixed-second transitions; until-reached needs hardware feedback")
            seconds = duration["seconds"]
            snapshot = load_snapshot_reference(step["snapshot"], ROOT / "snapshots")
            positions = mirrored_positions(snapshot["positions"], step.get("mirror", "None"))
            missing = set(servos) - positions.keys()
            if missing:
                raise ValueError("Snapshot is missing selected-leg servos: " + ", ".join(sorted(missing)))
            current = {s: max(bounds[s][0], min(bounds[s][1], positions[s]))
                       if bounds else positions[s] for s in servos}
        frames.append((float(seconds), dict(current)))
    if sum(seconds for seconds, _ in frames) <= 0:
        raise ValueError("Sequence preview needs a positive duration")
    return frames, document.get("loop", False)
