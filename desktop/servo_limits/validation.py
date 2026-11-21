"""Validate a complete EEPROM report and convert encoder bounds to logical radians."""

import json
from datetime import datetime, timezone

from documents.snapshot_units import joint_bounds


def hardware_report(message, expected_servos):
    try:
        items = json.loads(message)["servos"]
        if not isinstance(items, list):
            raise ValueError("Expected a servo list")
        fields = ("servo_id", "zero_position", "direction", "raw_lower", "raw_upper")
        rows = [[item[key] for key in fields] for item in items]
        if any(type(value) is not int for row in rows for value in row):
            raise ValueError("Servo calibration and limits must be integers")
        if {f"servo_{row[0]}" for row in rows} != set(expected_servos):
            raise ValueError("Hardware limits do not cover all configured servos")
        bounds = joint_bounds(*zip(*rows))
        servos = [dict(zip(fields, row), lower_radians=bounds[f"servo_{row[0]}"][0],
                       upper_radians=bounds[f"servo_{row[0]}"][1]) for row in rows]
    except (KeyError, TypeError, AttributeError) as error:
        raise ValueError("Invalid servo limit response") from error
    return {"read_at_utc": datetime.now(timezone.utc).isoformat(),
            "source": "/saprog/read_servo_limits (servo EEPROM)", "servos": servos}


def report_bounds(report):
    return {f"servo_{item['servo_id']}": (item["lower_radians"], item["upper_radians"])
            for item in report["servos"]}
