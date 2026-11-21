"""Command/state contract shared by the ROS node and headless tests."""

import math

from .geometry import LinkageConfig
from .physics import TIMESTEP, LegPhysics


def finite(value, label):
    if type(value) not in (float, int) or not math.isfinite(value):
        raise ValueError(f"{label} must be a finite number")
    return float(value)


class LegTestController:
    def __init__(self):
        self.physics = None
        self.paused = False
        self.sweep = None
        self.accumulator = 0.0
        self.message = "Stopped. Configure the linkage and press Start."
        self.error = ""

    def command(self, command):
        if not isinstance(command, dict):
            raise ValueError("Leg test command must be an object")
        action = command.get("action")
        if action == "start":
            if self.physics is not None:
                raise ValueError("Stop the rig before changing geometry")
            config = LinkageConfig.from_dict(command.get("config", {}))
            self.physics = LegPhysics(config)
            self.paused, self.sweep, self.accumulator = False, None, 0.0
            self.message = "Running with example geometry; validate dimensions against the physical leg."
        elif action == "stop":
            self.physics, self.sweep, self.paused = None, None, False
            self.accumulator = 0.0
            self.message = "Stopped."
        else:
            if self.physics is None:
                raise ValueError("Start the single-leg test first")
            if action in ("pause", "resume"):
                self.paused = action == "pause"
                self.message = "Paused." if self.paused else "Running."
            elif action == "reset":
                self.physics.reset()
                self.sweep, self.accumulator = None, 0.0
                self.message = "Reset to the closed reference pose."
            elif action == "target":
                target = finite(command.get("angle"), "Target")
                target = self.clamp_angle(target)
                self.physics.target = target
                self.sweep = None
                self.message = "Servo target accepted."
            elif action == "sweep":
                low = finite(command.get("lower"), "Sweep lower limit")
                high = finite(command.get("upper"), "Sweep upper limit")
                period = finite(command.get("period"), "Sweep period")
                low, high = self.clamp_angle(low), self.clamp_angle(high)
                if low > high or not 1 <= period <= 120:
                    raise ValueError("Sweep needs ordered limits and a period of 1 to 120 seconds")
                self.sweep = (low, high, period, float(self.physics.data.time))
                self.message = "Sweeping the crank; motion is limited by the motor model."
            elif action == "stop_sweep":
                self.sweep = None
                cfg = self.physics.config
                self.physics.target = max(cfg.servo_lower, min(cfg.servo_upper,
                    float(self.physics.data.qpos[self.physics.q_index])))
                self.message = "Sweep stopped; holding current servo position."
            else:
                raise ValueError("Unknown single-leg test action")
        self.error = ""
        return self.message

    def clamp_angle(self, angle):
        cfg = self.physics.config
        return max(cfg.servo_lower, min(cfg.servo_upper, angle))

    def advance(self, elapsed):
        if self.physics is None or self.paused:
            return
        self.accumulator += min(0.05, max(0, elapsed)) * self.physics.config.real_time_factor
        try:
            while self.accumulator >= TIMESTEP:
                target = self.physics.target
                if self.sweep is not None:
                    low, high, period, start = self.sweep
                    phase = (float(self.physics.data.time) - start) / period
                    target = low + (high - low) * (1 - math.cos(2 * math.pi * phase)) / 2
                self.physics.step(target)
                self.accumulator -= TIMESTEP
        except (RuntimeError, ValueError) as error:
            self.paused, self.sweep, self.accumulator = True, None, 0.0
            self.error = str(error)

    def state(self):
        state = {"running": self.physics is not None, "paused": self.paused,
                 "sweeping": self.sweep is not None, "message": self.message, "error": self.error}
        if self.physics is not None:
            state.update(self.physics.state())
            state["config"] = self.physics.config.to_dict()
        return state
