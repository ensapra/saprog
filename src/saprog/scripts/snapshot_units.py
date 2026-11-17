"""Unit conversion shared by snapshot editors and playback. ROS uses radians."""
import math

AX_RANGE = 5 * math.pi / 3


def joint_bounds(ids, zeros, directions, lower, upper):
    if not ids or any(len(values) != len(ids) for values in (zeros, directions, lower, upper)):
        raise ValueError("Read hardware limits for every configured servo first")
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate servo IDs")
    bounds = {}
    for servo, zero, direction, low, high in zip(ids, zeros, directions, lower, upper):
        if not 0 <= servo <= 253 or not 0 <= zero <= 1023 or not 0 <= low < high <= 1023 or direction not in (-1, 1):
            raise ValueError(f"Invalid limits/calibration for servo_{servo}")
        endpoints = [(raw - zero) * direction * AX_RANGE / 1023 for raw in (low, high)]
        bounds[f"servo_{servo}"] = tuple(sorted(endpoints))
    return bounds


def to_radians(value, bounds, unit):
    low, high = bounds
    if not math.isfinite(value):
        raise ValueError("Target must be finite")
    if unit == "normalized":
        if not -1 <= value <= 1:
            raise ValueError("Normalized target must be between -1 and 1")
        return low + (value + 1) * (high - low) / 2
    if unit != "radians" or not low <= value <= high:
        raise ValueError(f"Radian target must be between {low:.6g} and {high:.6g}")
    return value


def from_radians(value, bounds, unit):
    if unit == "radians":
        return value
    low, high = bounds
    return 2 * (value - low) / (high - low) - 1


def validate_snapshot(document, expected_names, bounds):
    if not isinstance(document, dict) or type(document.get("schema_version")) is not int or document["schema_version"] not in (1, 2):
        raise ValueError("Unsupported snapshot schema")
    unit = "radians" if document["schema_version"] == 1 else document.get("units")
    if unit not in ("normalized", "radians"):
        raise ValueError("Snapshot must specify normalized or radians units")
    name, positions = document.get("name"), document.get("positions")
    if not isinstance(name, str) or not name.strip() or not isinstance(positions, dict):
        raise ValueError("Snapshot needs a name and positions object")
    if set(positions) != set(expected_names):
        raise ValueError("Snapshot joint set does not match the configured robot")
    result = {}
    for joint in expected_names:
        value = positions[joint]
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"Invalid position for {joint}")
        try:
            result[joint] = to_radians(value, bounds[joint], unit)
        except ValueError as error:
            raise ValueError(f"{joint}: {error}") from error
    return name.strip(), result
