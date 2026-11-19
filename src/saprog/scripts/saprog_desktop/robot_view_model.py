"""ROS-independent contracts for mapping live robot state into the URDF view."""

import math
import time

# Servo IDs map to movable joints in the current robot description.
SERVO_JOINT_NAMES = {
    11: "front_right_11",
    12: "front_right_12",
    13: "front_right_joint_13",
    21: "back_right_21",
    22: "back_right_22",
    23: "back_right_joint_23",
    31: "front_left_31",
    32: "front_left_32",
    33: "front_left_joint_33",
    41: "back_left_41",
    42: "back_left_42",
    43: "back_left_joint_43",
}

# Logical servo angles whose rotation is opposite to the supplied URDF axes.
# Display correction only; backend calibration and commands retain their signs.
REVERSED_DISPLAY_SERVOS = frozenset({11, 12, 13, 22, 23, 31, 33, 43})

IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)
ORIENTATION_MAX_AGE_SECONDS = 0.5


class OrientationReference:
    """Return the latest sensor-relative quaternion, or identity when stale."""

    def __init__(self, max_age=ORIENTATION_MAX_AGE_SECONDS, clock=time.monotonic):
        self.max_age = max_age
        self.clock = clock
        self.quaternion = IDENTITY_QUATERNION
        self.received_at = None

    def update(self, quaternion):
        values = tuple(float(value) for value in quaternion)
        if len(values) != 4 or not all(map(math.isfinite, values)):
            return False
        if sum(value * value for value in values) < 1e-12:
            return False
        self.quaternion = values
        self.received_at = self.clock()
        return True

    def current(self):
        if self.received_at is None or self.clock() - self.received_at > self.max_age:
            return IDENTITY_QUATERNION
        return self.quaternion
