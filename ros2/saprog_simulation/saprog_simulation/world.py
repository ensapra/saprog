"""Prepare a temporary Gazebo world without changing the source URDF."""

import math
import re
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path
from urllib.parse import unquote, urlparse

from .profile import validate_bounds_profile

WORLD_NAME = "saprog_simulation"
MODEL_NAME = "saprog"


def finite_number(value, name, lower, upper):
    if type(value) not in (float, int) or not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"{name} must be between {lower} and {upper}")
    return float(value)


def resolve_mesh(filename, urdf_path):
    """Resolve exporter package URIs even though urdf/ is outside ros2/."""
    parsed = urlparse(filename)
    if parsed.scheme in ("package", "model"):
        # The checked-in description package is intentionally not rebuilt.
        local = urdf_path.parent.parent
        package_xml = local / "package.xml"
        if package_xml.is_file() and ET.parse(package_xml).findtext("name") == parsed.netloc:
            root = local
        else:
            from ament_index_python.packages import get_package_share_directory

            root = Path(get_package_share_directory(parsed.netloc))
        path = root / unquote(parsed.path.lstrip("/"))
    elif parsed.scheme == "file":
        path = Path(unquote(parsed.path))
    elif parsed.scheme:
        raise ValueError(f"Unsupported mesh URI: {filename}")
    else:
        path = urdf_path.parent / filename
    path = path.resolve()
    if not path.is_file():
        raise ValueError(f"Mesh not found: {path}")
    return path.as_uri()


def child(parent, tag, text=None, **attributes):
    element = ET.SubElement(parent, tag, attributes)
    if text is not None:
        element.text = str(text)
    return element


def apply_servo_bounds(source, bounds):
    """Apply explicit bounds, never saved URDF angle stops as a fallback."""
    joints = {joint.get("name"): joint for joint in source.findall("joint")}
    ranges = {}
    for name, joint in joints.items():
        if joint.get("type") not in ("revolute", "continuous") or joint.find("mimic") is not None:
            continue
        match = re.search(r"(?:^|_)(\d+)$", name)
        if match is None:
            continue
        alias = f"servo_{match[1]}"
        if not isinstance(bounds, dict) or alias not in bounds:
            raise ValueError(f"Explicit servo bounds are required for {alias}")
        pair = bounds[alias]
        if (not isinstance(pair, (list, tuple)) or len(pair) != 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in pair)
                or pair[0] >= pair[1]):
            raise ValueError(f"Invalid bounds for {alias}")
        ranges[name] = pair
    for name, joint in joints.items():
        mimic = joint.find("mimic")
        if mimic is not None and mimic.get("joint") in ranges:
            gain, offset = float(mimic.get("multiplier", "1")), float(mimic.get("offset", "0"))
            ranges[name] = sorted(gain*v+offset for v in ranges[mimic.get("joint")])
    for name, (lower, upper) in ranges.items():
        joint = joints[name]
        joint.set("type", "revolute")
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit", effort="2", velocity="3")
        limit.set("lower", str(lower))
        limit.set("upper", str(upper))


def prepare_world(urdf_path, directory, *, step_size=0.004, real_time_factor=0.5, spawn_height=0.3,
                  servo_bounds=None, bounds_profile=None):
    """Convert with installed SDFormat, preserving geometry, inertia and mimic joints."""
    step_size = finite_number(step_size, "Physics step", 0.001, 0.01)
    real_time_factor = finite_number(real_time_factor, "Simulation speed", 0.1, 4.0)
    spawn_height = finite_number(spawn_height, "Spawn height", 0.05, 2.0)
    if bounds_profile is not None:
        if servo_bounds is not None:
            raise ValueError("Choose bounds_profile or servo_bounds, not both")
        servo_bounds = validate_bounds_profile(bounds_profile)["servo_bounds"]
    urdf_path = Path(urdf_path).expanduser().resolve()
    source = ET.parse(urdf_path).getroot()
    if source.tag != "robot":
        raise ValueError("Choose a URDF robot document")
    apply_servo_bounds(source, servo_bounds)
    if bounds_profile is not None:
        mapped = []
        for joint in source.findall("joint"):
            if joint.get("type") in ("revolute", "continuous") and joint.find("mimic") is None:
                match = re.search(r"(?:^|_)(\d+)$", joint.get("name", ""))
                mapped.append(f"servo_{match[1]}" if match else joint.get("name"))
        if len(mapped) != 12 or set(mapped) != set(servo_bounds):
            raise ValueError("URDF must map exactly once to every simulation profile servo")
    for mesh in source.iter("mesh"):
        mesh.set("filename", resolve_mesh(mesh.attrib["filename"], urdf_path))
    prepared = Path(directory) / "robot.urdf"
    ET.ElementTree(source).write(prepared, encoding="utf-8", xml_declaration=True)
    converted = subprocess.run(
        ["gz", "sdf", "-p", str(prepared)], capture_output=True, text=True, timeout=30, check=False,
    )
    if converted.returncode:
        raise RuntimeError(f"URDF import failed: {converted.stderr[-2000:]}")
    model = ET.fromstring(converted.stdout).find("model")
    if model is None:
        raise ValueError("SDFormat did not produce a robot model")
    model.set("name", MODEL_NAME)
    pose = model.find("pose")
    if pose is None:
        pose = child(model, "pose")
    pose.text = f"0 0 {spawn_height} 0 0 0"
    static = model.find("static")
    if static is None:
        static = child(model, "static")
    static.text = "false"
    for link in model.findall("link"):
        gravity = link.find("gravity")
        if gravity is None:
            gravity = child(link, "gravity")
        gravity.text = "true"
    joints = {}
    for joint in source.findall("joint"):
        if joint.get("type") not in ("revolute", "continuous", "prismatic"):
            continue
        name = joint.attrib["name"]
        if joint.find("mimic") is not None:
            # Let Bullet Featherstone enforce the imported bilateral constraint.
            imported = model.find(f"joint[@name='{name}']/axis/mimic")
            if imported is None:
                raise RuntimeError(f"Installed SDFormat lost the mimic relationship for {name}")
            continue
        limit = joint.find("limit")
        lower = float(limit.get("lower", "-3.141592653589793")) if limit is not None else -math.pi
        upper = float(limit.get("upper", "3.141592653589793")) if limit is not None else math.pi
        match = re.search(r"(?:^|_)(\d+)$", name)
        alias = f"servo_{match[1]}" if match else name
        if alias in joints:
            raise ValueError(f"Ambiguous servo mapping: {alias}")
        joints[alias] = {"name": name, "lower": lower, "upper": upper}
    # No rendering/sensor system on the server. Scene data is only consumed if
    # an optional GUI connects. Keep the original collision meshes and inertia.
    sdf = ET.Element("sdf", version="1.11")
    world = child(sdf, "world", name=WORLD_NAME)
    child(world, "gravity", "0 0 -9.81")
    physics = child(world, "physics", name="lightweight", type="ignored")
    child(physics, "max_step_size", step_size)
    child(physics, "real_time_factor", real_time_factor)
    child(physics, "real_time_update_rate", real_time_factor / step_size)
    plugin = child(world, "plugin", filename="gz-sim-physics-system", name="gz::sim::systems::Physics")
    child(child(plugin, "engine"), "filename", "gz-physics-bullet-featherstone-plugin")
    child(world, "plugin", filename="gz-sim-user-commands-system", name="gz::sim::systems::UserCommands")
    broadcaster = child(world, "plugin", filename="gz-sim-scene-broadcaster-system",
                        name="gz::sim::systems::SceneBroadcaster")
    child(broadcaster, "state_hertz", 15)
    scene = child(world, "scene")
    child(scene, "shadows", "false")
    child(scene, "ambient", "0.6 0.6 0.6 1")
    child(scene, "background", "0.14 0.18 0.23 1")
    light = child(world, "light", name="sun", type="directional")
    child(light, "direction", "-0.5 0.2 -1")
    child(light, "cast_shadows", "false")
    ground = child(world, "model", name="ground")
    child(ground, "static", "true")
    link = child(ground, "link", name="ground_link")
    for kind in ("collision", "visual"):
        plane = child(child(child(link, kind, name=f"ground_{kind}"), "geometry"), "plane")
        child(plane, "normal", "0 0 1")
        child(plane, "size", "10 10")
    world.append(model)
    destination = Path(directory) / "world.sdf"
    ET.ElementTree(sdf).write(destination, encoding="utf-8", xml_declaration=True)
    return destination, joints


GUI_CONFIG = """<?xml version="1.0"?>
<window><width>800</width><height>600</height><dialog_on_exit>false</dialog_on_exit></window>
<plugin filename="MinimalScene" name="3D View">
  <gz-gui><title>Simulation</title><property type="bool" key="showTitleBar">false</property>
  <property type="string" key="state">docked</property></gz-gui>
  <engine>ogre2</engine><scene>scene</scene>
  <ambient_light>0.6 0.6 0.6</ambient_light><background_color>0.14 0.18 0.23</background_color>
  <camera_pose>0.9 -0.9 0.6 0 0.35 2.35</camera_pose>
</plugin>
<plugin filename="GzSceneManager" name="Scene Manager"/>
<plugin filename="InteractiveViewControl" name="View control"/>
"""
