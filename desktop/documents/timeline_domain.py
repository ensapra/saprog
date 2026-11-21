"""Pure timeline validation and control-group rules."""

import copy
import math

from .snapshot_units import LEG_JOINTS, validate_snapshot

CONTROL_MODES = ("Global Control", "Leg Control", "Custom Control")
MOTION_STEP = 0
WAIT_STEP = 1
UNTIL_REACHED = 0
FIXED_SECONDS = 1
MIRROR_MODES = ("None", "Horizontal", "Vertical", "Horizontal + Vertical")


def mirror_mode(step):
    mode = step.get("mirror", "None")
    if mode not in MIRROR_MODES:
        raise ValueError("Mirror mode must be None, Horizontal, Vertical, or Horizontal + Vertical")
    return mode


def mirrored_positions(positions, mode, *, inverse=False):
    """Reflect pose targets by exchanging corresponding legs across an axis."""
    if mode not in MIRROR_MODES:
        raise ValueError("Unknown snapshot mirror mode")
    result = dict(positions)
    legs = {
        "front_left": ("servo_31", "servo_32", "servo_33"),
        "front_right": ("servo_11", "servo_12", "servo_13"),
        "back_left": ("servo_41", "servo_42", "servo_43"),
        "back_right": ("servo_21", "servo_22", "servo_23"),
    }
    pairs = []
    if mode in ("Horizontal", "Horizontal + Vertical"):
        pairs.extend((("front_left", "front_right"), ("back_left", "back_right")))
    if mode in ("Vertical", "Horizontal + Vertical"):
        pairs.extend((("front_left", "back_left"), ("front_right", "back_right")))
    for first, second in reversed(pairs) if inverse else pairs:
        for left, right in zip(legs[first], legs[second]):
            if left in result and right in result:
                result[left], result[right] = result[right], result[left]
    return result


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
    groups = {label.removesuffix(" Leg").capitalize(): [joint for joint in names if joint in joints]
              for label, names in LEG_JOINTS}
    assigned = {joint for names in groups.values() for joint in names}
    groups.update({joint: [joint] for joint in joints if joint not in assigned})
    return {label: names for label, names in groups.items() if names}


def _motion_pair(value):
    if not isinstance(value, dict):
        raise ValueError("Motion settings must be objects")
    for key in ("speed", "torque"):
        if type(value.get(key)) is not int or not 1 <= value[key] <= 1023:
            raise ValueError(f"{key} must be an integer from 1 to 1023")
    if value.get("enabled", True) is not True:
        raise ValueError("Legacy torque-off transitions are no longer supported; use a timeline with torque on")
    return {"speed": value["speed"], "torque": value["torque"]}


def motion_settings(step, joints):
    if step.get("control_mode", "Global Control") not in CONTROL_MODES:
        raise ValueError("Unknown control mode")
    global_motion = _motion_pair(step.get("motion"))
    overrides = step.get("overrides", {})
    if not isinstance(overrides, dict) or set(overrides) - set(joints):
        raise ValueError("Unknown servo in transition overrides")
    return {joint: _motion_pair(overrides[joint]) if joint in overrides else dict(global_motion) for joint in joints}


def regroup_motion(step, mode, joints=None):
    joints = list(joints if joints is not None else step["snapshot"]["positions"])
    resolved = motion_settings(step, joints)
    result = copy.deepcopy(step)
    result["control_mode"] = mode
    result["overrides"] = {}
    for names in control_groups(mode, joints).values():
        pair = {key: round(sum(resolved[j][key] for j in names) / len(names)) for key in ("speed", "torque")}
        if mode == "Global Control":
            result["motion"] = pair
        else:
            for joint in names:
                result["overrides"][joint] = dict(pair)
    result["motion"] = {key: result["motion"][key] for key in ("speed", "torque")}
    return result


def lifecycle_settings(document):
    on_end = document.get("on_end", "on")
    if on_end not in ("on", "off"):
        raise ValueError("On end must be 'on' or 'off'")
    return {"on_end": on_end}


def validate_timeline(document, joints=None, bounds=None, snapshot_resolver=None):
    if (not isinstance(document, dict) or document.get("kind") != "saprog_timeline"
            or type(document.get("schema_version")) is not int or document["schema_version"] not in (1, 2)):
        raise ValueError("Unsupported timeline file")
    version = document["schema_version"]
    lifecycle_settings(document)
    if type(document.get("loop", False)) is not bool:
        raise ValueError("Loop must be true or false")
    if type(document.get("smooth_playback", False)) is not bool:
        raise ValueError("Smooth playback must be true or false")
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
        motion_settings(step, list(positions))
        transition_duration(step)
        mirror_mode(step)
    return copy.deepcopy(steps)
