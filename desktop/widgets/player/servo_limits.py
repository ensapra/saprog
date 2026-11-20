"""Validate hardware readings and atomically store calibrated servo bounds."""

import json
import math
import os
import tempfile
from datetime import datetime, timezone
from pathlib import Path

LIMITS_PATH = Path(__file__).resolve().parents[3] / "urdf/config/measured_servo_limits.json"
TICK_RADIANS = math.radians(300) / 1023


def hardware_report(message, expected_servos):
    """Reject incomplete or invalid reads before replacing any cached limits."""
    try:
        items = json.loads(message)["servos"]
        if not isinstance(items, list):
            raise ValueError("Expected a servo list")
        servos = []
        names = set()
        for item in items:
            fields = ("servo_id", "zero_position", "direction", "raw_lower", "raw_upper")
            values = [item[key] for key in fields]
            if any(type(value) is not int for value in values):
                raise ValueError("Servo calibration and limits must be integers")
            servo_id, zero, direction, low, high = values
            name = f"servo_{servo_id}"
            if (name in names or not 0 <= servo_id <= 253 or not 0 <= zero <= 1023
                    or direction not in (-1, 1) or not 0 <= low < high <= 1023):
                raise ValueError(f"Invalid limits or calibration for {name}")
            names.add(name)
            lower, upper = sorted((
                (low - zero) * direction * TICK_RADIANS,
                (high - zero) * direction * TICK_RADIANS,
            ))
            servos.append(dict(zip(fields, values), lower_radians=lower, upper_radians=upper))
        if names != set(expected_servos):
            raise ValueError("Hardware limits do not cover all configured servos")
    except (KeyError, TypeError, AttributeError) as error:
        raise ValueError("Invalid servo limit response") from error
    return {
        "read_at_utc": datetime.now(timezone.utc).isoformat(),
        "source": "/saprog/read_servo_limits (servo EEPROM)",
        "servos": servos,
    }


def report_bounds(report):
    return {
        f"servo_{item['servo_id']}": (item["lower_radians"], item["upper_radians"])
        for item in report["servos"]
    }


def save_report(report, path=None):
    """Preserve URDF metadata and leave the last valid file intact on failure."""
    path = Path(path) if path is not None else LIMITS_PATH
    try:
        previous = json.loads(path.read_text(encoding="utf-8"))
        old_servos = {item["servo_id"]: item for item in previous.get("servos", [])}
    except (OSError, ValueError, TypeError, KeyError, AttributeError):
        previous, old_servos = {}, {}
    merged = dict(previous, **report)
    merged["servos"] = [
        dict(old_servos.get(item["servo_id"], {}), **item) for item in report["servos"]
    ]
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, suffix=".tmp", delete=False,
        ) as stream:
            temporary = Path(stream.name)
            json.dump(merged, stream, indent=2, allow_nan=False)
            stream.write("\n")
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
