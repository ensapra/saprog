"""Timeline documents reference snapshots and store per-transition motion settings."""

import copy
import json
import math
from pathlib import Path

from snapshot_files import write_json_atomic
from snapshot_units import validate_snapshot

CONTROL_MODES = ("Global Control", "Leg Control", "Custom Control")
MOTION_STEP = 0
WAIT_STEP = 1
UNTIL_REACHED = 0
FIXED_SECONDS = 1
LEGS = {
    "Front left": (31, 32, 33),
    "Front right": (11, 12, 13),
    "Back left": (41, 42, 43),
    "Back right": (21, 22, 23),
}


def wait_seconds(step):
    value = step.get("seconds")
    if type(value) not in (int, float):
        raise ValueError("Wait duration must be a number of seconds")
    try:
        valid = math.isfinite(value) and value >= 0
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError("Wait duration must be finite and non-negative")
    return value


def transition_duration(step):
    duration = step.get("duration", {"mode": "until_reached", "seconds": 1.0})
    if not isinstance(duration, dict) or duration.get("mode") not in ("until_reached", "seconds"):
        raise ValueError("Transition duration must be until_reached or seconds")
    seconds = wait_seconds({"seconds": duration.get("seconds", 1.0)})
    return {"mode": duration["mode"], "seconds": seconds}


def timeline_rows(steps):
    """Map visible rows to entries while keeping transition/snapshot pairs intact."""
    rows = []
    for index, step in enumerate(steps):
        kinds = ("wait",) if step.get("kind") == "wait" else ("transition", "snapshot")
        rows.extend((index, kind) for kind in kinds)
    return rows


def control_groups(mode, joints):
    if mode == "Global Control":
        return {"All servos": list(joints)}
    if mode == "Custom Control":
        return {joint: [joint] for joint in joints}
    if mode != "Leg Control":
        raise ValueError("Unknown control mode")
    groups = {
        label: [f"servo_{i}" for i in ids if f"servo_{i}" in joints] for label, ids in LEGS.items()
    }
    assigned = {joint for names in groups.values() for joint in names}
    groups.update({joint: [joint] for joint in joints if joint not in assigned})
    return {label: names for label, names in groups.items() if names}


def regroup_motion(step, mode, joints=None):
    """Expand current settings, then average only when combining unequal values."""
    joints = list(joints if joints is not None else step["snapshot"]["positions"])
    resolved = motion_settings(step, joints)
    result = copy.deepcopy(step)
    result["control_mode"] = mode
    result["overrides"] = {}
    for names in control_groups(mode, joints).values():
        pair = {
            key: round(sum(resolved[j][key] for j in names) / len(names))
            for key in ("speed", "torque")
        }
        if mode == "Global Control":
            result["motion"] = pair
        else:
            for joint in names:
                result["overrides"][joint] = dict(pair)
    result["motion"] = {key: result["motion"][key] for key in ("speed", "torque")}
    return result


def _motion_pair(value):
    """Validate the common shape used by global settings and joint overrides."""
    if not isinstance(value, dict):
        raise ValueError("Motion settings must be objects")
    for key in ("speed", "torque"):
        if type(value.get(key)) is not int or not 1 <= value[key] <= 1023:
            raise ValueError(f"{key} must be an integer from 1 to 1023")
    if value.get("enabled", True) is not True:
        raise ValueError(
            "Legacy torque-off transitions are no longer supported; use a timeline with torque on"
        )
    return {"speed": value["speed"], "torque": value["torque"]}


def motion_settings(step, joints):
    """Resolve each joint's override, falling back to global motion settings."""
    if step.get("control_mode", "Global Control") not in CONTROL_MODES:
        raise ValueError("Unknown control mode")
    global_motion = _motion_pair(step.get("motion"))
    overrides = step.get("overrides", {})
    if not isinstance(overrides, dict) or set(overrides) - set(joints):
        raise ValueError("Unknown servo in transition overrides")
    return {
        joint: _motion_pair(overrides[joint]) if joint in overrides else dict(global_motion)
        for joint in joints
    }


def lifecycle_settings(document):
    on_end = document.get("on_end", "on")
    if on_end not in ("on", "off"):
        raise ValueError("On end must be 'on' or 'off'")
    return {"on_end": on_end}


def make_snapshot_reference(path, directory):
    """Return a path relative to the snapshot directory when possible."""
    path = Path(path).resolve()
    directory = Path(directory).resolve()
    try:
        return path.relative_to(directory).as_posix()
    except ValueError:
        return str(path)


def reference_for_legacy_snapshot(snapshot, directory, joints=None, bounds=None):
    """Find a file equivalent to an embedded legacy pose, rejecting ambiguity."""
    name, positions = validate_snapshot(snapshot, joints, bounds)
    matches = []
    for path in Path(directory).glob("*.json"):
        try:
            with path.open(encoding="utf-8") as stream:
                candidate = json.load(stream)
            candidate_name, candidate_positions = validate_snapshot(candidate, joints, bounds)
        except (OSError, ValueError, TypeError, KeyError):
            continue
        if candidate_name != name or candidate_positions.keys() != positions.keys():
            continue
        if all(math.isclose(candidate_positions[j], positions[j], rel_tol=0, abs_tol=1e-9) for j in positions):
            matches.append(path)
    if not matches:
        raise ValueError(f"No matching snapshot pose named '{name}' is present in {directory}")
    # Duplicate files with the same pose are interchangeable; choose in stable order.
    return make_snapshot_reference(sorted(matches, key=lambda path: path.name)[0], directory)


def load_snapshot_reference(reference, directory, joints=None, bounds=None):
    """Read and validate the current contents of a referenced snapshot file."""
    if not isinstance(reference, str) or not reference.strip():
        raise ValueError("Snapshot reference must be a non-empty path")
    path = Path(reference)
    if not path.is_absolute():
        path = (Path(directory) / path).resolve()
        try:
            path.relative_to(Path(directory).resolve())
        except ValueError as exc:
            raise ValueError("Snapshot reference must stay inside the snapshot directory") from exc
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    name, positions = validate_snapshot(document, joints, bounds)
    return {"schema_version": 1, "name": name, "positions": positions}


def resolve_timeline_steps(steps, snapshot_resolver):
    """Resolve references to current snapshots for editing or playback."""
    resolved = copy.deepcopy(steps)
    for step in resolved:
        if step.get("kind") != "wait" and isinstance(step.get("snapshot"), str):
            step["snapshot"] = snapshot_resolver(step["snapshot"])
    return resolved


def timeline_action_data(steps, joints, loop=False, run_end_action=True, end_torque_enabled=True):
    """Flatten validated timeline steps into the ROS action's row-major goal fields."""
    data = {
        "joint_names": list(joints),
        "step_kinds": [],
        "duration_modes": [],
        "durations": [],
        "positions": [],
        "speeds": [],
        "torque_limits": [],
        "loop": bool(loop),
        "run_end_action": bool(run_end_action and not loop),
        "end_torque_enabled": bool(end_torque_enabled),
    }
    for step in steps:
        if step.get("kind") == "wait":
            # The ROS action uses one fixed-width row per step, including waits.
            data["step_kinds"].append(WAIT_STEP)
            data["duration_modes"].append(FIXED_SECONDS)
            data["durations"].append(wait_seconds(step))
            data["positions"].extend([0.0] * len(joints))
            data["speeds"].extend([1] * len(joints))
            data["torque_limits"].extend([1] * len(joints))
            continue
        duration = transition_duration(step)
        settings = motion_settings(step, joints)
        data["step_kinds"].append(MOTION_STEP)
        data["duration_modes"].append(
            FIXED_SECONDS if duration["mode"] == "seconds" else UNTIL_REACHED
        )
        data["durations"].append(duration["seconds"])
        data["positions"].extend(step["snapshot"]["positions"][joint] for joint in joints)
        data["speeds"].extend(settings[joint]["speed"] for joint in joints)
        data["torque_limits"].extend(settings[joint]["torque"] for joint in joints)
    return data


def validate_timeline(document, joints=None, bounds=None, snapshot_resolver=None):
    """Validate file structure, referenced poses, and motion before playback/save."""
    if (
        not isinstance(document, dict)
        or document.get("kind") != "saprog_timeline"
        or type(document.get("schema_version")) is not int
        or document["schema_version"] not in (1, 2)
    ):
        raise ValueError("Unsupported timeline file")
    version = document["schema_version"]
    lifecycle_settings(document)
    if type(document.get("loop", False)) is not bool:
        raise ValueError("Loop must be true or false")
    steps = document.get("steps")
    if not isinstance(steps, list) or not steps:
        raise ValueError("Add at least one snapshot or wait")
    for step in steps:
        if isinstance(step, dict) and step.get("kind") == "wait":
            wait_seconds(step)
            continue
        if not isinstance(step, dict):
            raise ValueError("Each step needs a snapshot")
        snapshot_value = step.get("snapshot")
        if version == 1 and isinstance(snapshot_value, dict):
            snapshot = snapshot_value
        elif version == 2 and isinstance(snapshot_value, str) and snapshot_resolver is not None:
            snapshot = snapshot_resolver(snapshot_value)
        else:
            raise ValueError("Each step needs a valid snapshot reference")
        _, positions = validate_snapshot(snapshot, joints, bounds)
        names = list(positions)
        motion_settings(step, names)
        transition_duration(step)
    return copy.deepcopy(steps)


def save_timeline(path, steps, on_end="on", loop=False, snapshot_resolver=None):
    document = {"kind": "saprog_timeline", "schema_version": 2, "steps": steps}
    document.update(on_end=on_end, loop=loop)
    validate_timeline(document, snapshot_resolver=snapshot_resolver)
    write_json_atomic(path, document)
