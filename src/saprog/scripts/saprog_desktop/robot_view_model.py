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

IDENTITY_QUATERNION = (0.0, 0.0, 0.0, 1.0)
# Apply +90 degrees about Z after the existing 180-degree X mounting correction.
IMU_MOUNTING_QUATERNION = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
ORIENTATION_MAX_AGE_SECONDS = 0.5


def apply_imu_mounting_rotation(quaternion):
    """Compose sensor attitude with the fixed X/Z mounting correction."""
    x, y, z, w = quaternion
    mx, my, mz, mw = IMU_MOUNTING_QUATERNION
    return (
        w * mx + x * mw + y * mz - z * my,
        w * my - x * mz + y * mw + z * mx,
        w * mz + x * my - y * mx + z * mw,
        w * mw - x * mx - y * my - z * mz,
    )


class OrientationReference:
    """Hold the latest valid quaternion; report freshness independently."""

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
        return self.quaternion

    def is_fresh(self):
        """Whether an orientation sample has arrived within the freshness limit."""
        return self.received_at is not None and self.clock() - self.received_at <= self.max_age
