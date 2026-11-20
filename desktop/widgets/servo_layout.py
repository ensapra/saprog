"""Canonical servo names and leg positions shared by desktop widgets."""

LEG_LAYOUT = (
    ("Front Right Leg", ("servo_11", "servo_12", "servo_13"), 0, 1),
    ("Back Right Leg", ("servo_21", "servo_22", "servo_23"), 1, 1),
    ("Front Left Leg", ("servo_31", "servo_32", "servo_33"), 0, 0),
    ("Back Left Leg", ("servo_41", "servo_42", "servo_43"), 1, 0),
)

LEG_SERVOS = tuple((name, servos) for name, servos, _row, _column in LEG_LAYOUT)
SERVO_NAMES = tuple(servo for _leg, servos in LEG_SERVOS for servo in servos)

__all__ = ["LEG_LAYOUT", "LEG_SERVOS", "SERVO_NAMES"]
