"""Planar four-bar reference geometry; all defaults are uncalibrated examples."""

import math
from dataclasses import asdict, dataclass, fields, replace


@dataclass(frozen=True)
class LinkageConfig:
    # SI units. O is the fixed crank shaft; D is the fixed knee pivot.
    pivot_x: float = 0.0
    pivot_z: float = -0.075
    crank_radius: float = 0.017
    rod_length: float = 0.080
    knee_radius: float = 0.025
    foot_length: float = 0.150
    foot_offset: float = math.radians(-100)
    crank_zero: float = 0.8803420273988994
    branch: int = 1
    servo_lower: float = 0.2661499152601324
    servo_upper: float = 0.8803420273988994
    initial_servo: float = 0.4
    crank_mass: float = 0.013
    rod_mass: float = 0.010
    leg_mass: float = 0.058
    torque_limit: float = 0.5
    speed_limit: float = 1.0
    kp: float = 8.0
    kd: float = 0.2
    foot_load: float = 0.0
    real_time_factor: float = 0.5

    @classmethod
    def from_dict(cls, values):
        if not isinstance(values, dict):
            raise ValueError("Linkage configuration must be an object")
        known = {field.name for field in fields(cls)}
        if set(values) - known:
            raise ValueError(f"Unknown linkage settings: {', '.join(sorted(set(values) - known))}")
        config = cls(**values)
        if (
            type(config.initial_servo) in (int, float)
            and math.isfinite(config.initial_servo)
            and config.servo_lower < config.servo_upper
        ):
            config = replace(
                config,
                initial_servo=max(
                    config.servo_lower, min(config.servo_upper, config.initial_servo),
                ),
            )
        config.validate()
        return config

    def to_dict(self):
        return asdict(self)

    def validate(self):
        for name, value in asdict(self).items():
            if type(value) not in (int, float) or not math.isfinite(value):
                raise ValueError(f"{name} must be a finite number")
        if type(self.branch) is not int or self.branch not in (-1, 1):
            raise ValueError("Assembly branch must be +1 or -1")
        for name in ("crank_radius", "rod_length", "knee_radius", "foot_length"):
            if not 0.001 <= getattr(self, name) <= 0.5:
                raise ValueError(f"{name} must be between 1 and 500 mm")
        if not 0.001 <= math.hypot(self.pivot_x, self.pivot_z) <= 0.5:
            raise ValueError("Fixed pivots must be between 1 and 500 mm apart")
        if not -math.pi <= self.servo_lower < self.servo_upper <= math.pi:
            raise ValueError("Servo limits must be ordered and within -pi to pi")
        if not self.servo_lower <= self.initial_servo <= self.servo_upper:
            raise ValueError("Initial servo angle must be inside the travel limits")
        for name in ("foot_offset", "crank_zero"):
            if abs(getattr(self, name)) > 2 * math.pi:
                raise ValueError(f"{name} must be within -360 to 360 degrees")
        for name in ("crank_mass", "rod_mass", "leg_mass"):
            if not 0.001 <= getattr(self, name) <= 5:
                raise ValueError(f"{name} must be between 0.001 and 5 kg")
        for name, lower, upper in (
            ("torque_limit", 0.01, 10), ("speed_limit", 0.01, 10),
            ("kp", 0.01, 100), ("kd", 0.001, 10),
            ("foot_load", 0, 50), ("real_time_factor", 0.1, 2),
        ):
            if not lower <= getattr(self, name) <= upper:
                raise ValueError(f"{name} must be between {lower} and {upper}")
        # Check exact extrema of the pin-to-knee distance over the whole travel.
        # Endpoint checks alone can miss an unreachable interval or a toggle.
        low, high = self.crank_zero - self.servo_upper, self.crank_zero - self.servo_lower
        angles = [low, high]
        direction = math.atan2(self.pivot_z, self.pivot_x)
        for k in range(math.floor((low - direction) / math.pi),
                       math.ceil((high - direction) / math.pi) + 1):
            angle = direction + k * math.pi
            if low <= angle <= high:
                angles.append(angle)
        for angle in angles:
            distance = math.hypot(
                self.pivot_x - self.crank_radius * math.cos(angle),
                self.pivot_z - self.crank_radius * math.sin(angle),
            )
            if not abs(self.rod_length - self.knee_radius) + 1e-5 < distance < (
                self.rod_length + self.knee_radius - 1e-5
            ):
                raise ValueError("Linkage cannot close over the full servo range, or reaches a toggle")


def reference_pose(config, servo):
    """Circle intersection chooses one fixed assembly branch, never a mimic ratio."""
    theta = config.crank_zero - servo
    a = (config.crank_radius * math.cos(theta), config.crank_radius * math.sin(theta))
    d = (config.pivot_x, config.pivot_z)
    dx, dz = d[0] - a[0], d[1] - a[1]
    distance = math.hypot(dx, dz)
    if distance < 1e-12:
        raise ValueError("Coincident linkage pivots")
    along = (config.rod_length**2 - config.knee_radius**2 + distance**2) / (2 * distance)
    height_squared = config.rod_length**2 - along**2
    if height_squared <= 0:
        raise ValueError("Linkage is unreachable or at a toggle")
    height = config.branch * math.sqrt(height_squared)
    b = (a[0] + (along * dx - height * dz) / distance,
         a[1] + (along * dz + height * dx) / distance)
    gamma = math.atan2(b[1] - a[1], b[0] - a[0])
    beta = math.atan2(b[1] - d[1], b[0] - d[0])
    foot_angle = beta + config.foot_offset
    f = (d[0] + config.foot_length * math.cos(foot_angle),
         d[1] + config.foot_length * math.sin(foot_angle))
    return {"O": (0.0, 0.0), "A": a, "B": b, "D": d, "F": f,
            "theta": theta, "gamma": gamma, "beta": beta}
