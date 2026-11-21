"""Synthetic EEPROM responses for offline tests; never used by application code."""

import json

from servo_limits import ServoLimits, get_servo_limits, hardware_report, report_bounds
from widgets.servo_layout import SERVO_NAMES


def limits_message(raw_lower=100, raw_upper=924):
    return json.dumps({"servos": [
        {"servo_id": int(name[6:]), "zero_position": 512, "direction": 1,
         "raw_lower": raw_lower, "raw_upper": raw_upper} for name in SERVO_NAMES]})


def fixture_bounds():
    return report_bounds(hardware_report(limits_message(), SERVO_NAMES))


def prime_limits(manager, message=None):
    service = manager if isinstance(manager, ServoLimits) else get_servo_limits(manager)
    service.pending = "fixture-limits"
    service.result("fixture-limits", True, message or limits_message())
    return service
