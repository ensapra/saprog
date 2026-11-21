"""Radian validation shared by snapshot editors and playback."""

import math

AX_RANGE = 5 * math.pi / 3
AX_TICK_RADIANS = AX_RANGE / 1023
LEG_JOINTS = (
    ("Front Left Leg", ("servo_31", "servo_32", "servo_33")),
    ("Front Right Leg", ("servo_11", "servo_12", "servo_13")),
    ("Back Left Leg", ("servo_41", "servo_42", "servo_43")),
    ("Back Right Leg", ("servo_21", "servo_22", "servo_23")),
)


def joint_bounds(ids, zeros, directions, lower, upper):
    """Convert measured encoder limits to ordered radian limits per joint."""
    if not ids or any(len(values) != len(ids) for values in (zeros, directions, lower, upper)):
        raise ValueError("Read hardware limits for every configured servo first")
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate servo IDs")
    bounds = {}
    for servo, zero, direction, low, high in zip(ids, zeros, directions, lower, upper):
        if (
            not 0 <= servo <= 253
            or not 0 <= zero <= 1023
            or not 0 <= low < high <= 1023
            or direction not in (-1, 1)
        ):
            raise ValueError(f"Invalid limits/calibration for servo_{servo}")
        endpoints = [(raw - zero) * direction * AX_RANGE / 1023 for raw in (low, high)]
        bounds[f"servo_{servo}"] = tuple(sorted(endpoints))
    return bounds


def to_radians(value, bounds, unit):
    """Validate a radian target and clamp it to its joint limits."""
    if unit != "radians":
        raise ValueError("Snapshot targets must use radians")
    try:
        finite = math.isfinite(value)
    except OverflowError:
        finite = False
    if not finite:
        raise ValueError("Target must be finite")
    return clamp_to_bounds(value, bounds)


def clamp_to_bounds(value, bounds):
    """Return a value constrained to a pair of inclusive bounds."""
    low, high = bounds
    return max(low, min(high, value))


def validate_snapshot(document, expected_names=None, bounds=None):
    """Return a name and radian targets without modifying the input document.

    Hardware callers supply joint names and limits. Offline timeline editing may
    omit them to check a radian document's structure, but cannot certify reachability.
    """
    if not isinstance(document, dict):
        raise ValueError("Unsupported snapshot schema")
    version = document.get("schema_version")
    if type(version) is not int or version not in (1, 2):
        raise ValueError("Unsupported snapshot schema")
    # Version 1 stores radians implicitly; version 2 declares its units explicitly.
    unit = document.get("units", "radians" if version == 1 else None)
    if unit != "radians":
        raise ValueError("Snapshot targets must use radians")
    name = document.get("name")
    positions = document.get("positions")
    if not isinstance(name, str) or not name.strip() or not isinstance(positions, dict):
        raise ValueError("Snapshot needs a name and positions object")
    if expected_names is None:
        expected_names = list(positions)
        if not expected_names:
            raise ValueError("Snapshot has no servos")
    else:
        expected_names = list(expected_names)
    if set(positions) != set(expected_names):
        raise ValueError("Snapshot joint set does not match the configured robot")
    if not bounds:
        bounds = {joint: (-math.inf, math.inf) for joint in expected_names}
    result = {}
    for joint in expected_names:
        value = positions[joint]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"Invalid position for {joint}")
        try:
            result[joint] = to_radians(value, bounds[joint], unit)
        except (ValueError, OverflowError) as error:
            raise ValueError(f"{joint}: {error}") from error
    return name.strip(), result
