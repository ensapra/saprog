"""Validated controller configuration and bounded refresh sequencing."""

from dataclasses import dataclass
import math
import time
from types import MappingProxyType

from .snapshot_units import joint_bounds

PARAMETERS = (
    "servo_ids", "zero_positions", "directions", "target_tolerance",
    "lower_limits", "upper_limits",
)


@dataclass(frozen=True)
class SnapshotConfiguration:
    ids: tuple
    joints: tuple
    bounds: dict
    tolerance: float


def decode_configuration(values):
    """Validate six ROS parameter values and return a complete immutable result."""
    if len(values) != len(PARAMETERS):
        raise ValueError("controller returned incomplete configuration")
    ids, zeros, directions = (tuple(v.integer_array_value) for v in values[:3])
    tolerance = values[3].double_value
    lower, upper = (tuple(v.integer_array_value) for v in values[4:6])
    if isinstance(tolerance, bool) or not math.isfinite(tolerance) or tolerance < 0:
        raise ValueError("controller returned invalid target tolerance")
    bounds = MappingProxyType(joint_bounds(ids, zeros, directions, lower, upper))
    joints = tuple(f"servo_{servo}" for servo in ids)
    return SnapshotConfiguration(ids, joints, bounds, tolerance)


class SnapshotConfigurationClient:
    """Refresh limits, then read parameters, with deadlines and stale callback guards.

    Inject ROS clients/future factories so this lifecycle can be tested without ROS.
    Call poll() from the owning UI scheduler; on_result receives (config, error).
    """

    def __init__(self, limits_client, parameter_client, trigger_type, on_result,
                 timeout=8.0, retry_delay=2.0, clock=time.monotonic):
        self.limits_client = limits_client
        self.parameter_client = parameter_client
        self.trigger_type = trigger_type
        self.on_result = on_result
        self.timeout, self.retry_delay, self.clock = timeout, retry_delay, clock
        self._future = None
        self._started = 0.0
        self._retry_at = 0.0
        self._stage = "idle"
        self._identity = object()
        self.closed = False
        self.error = "Waiting to read hardware limits"

    def refresh(self):
        if self.closed:
            return False
        self._cancel_current()
        self._identity = object()
        self._stage = "limits"
        self._retry_at = 0.0
        self.error = "Refreshing hardware limits..."
        return True

    def close(self):
        self.closed = True
        self._identity = object()
        self._cancel_current()

    def _cancel_current(self):
        future, self._future = self._future, None
        if future is not None:
            try:
                future.cancel()
            except Exception:
                pass

    def _fail(self, message):
        self._cancel_current()
        self._stage = "idle"
        self.error = str(message)
        self._retry_at = self.clock() + self.retry_delay
        self.on_result(None, self.error)

    def poll(self):
        if self.closed:
            return
        now = self.clock()
        if self._stage == "idle":
            if now < self._retry_at:
                return
            self._stage = "limits"
        if self._stage == "ready":
            return
        if self._future is not None:
            if not self._future.done():
                if now - self._started >= self.timeout:
                    self._fail(f"{self._stage} request timed out")
                return
            identity, future, stage = self._identity, self._future, self._stage
            self._future = None
            try:
                response = future.result()
                if stage == "limits":
                    if not response.success:
                        raise RuntimeError(response.message or "hardware limit refresh failed")
                    self._stage = "parameters"
                    self.error = "Reading controller parameters..."
                else:
                    config = decode_configuration(response.values)
                    if identity is self._identity and not self.closed:
                        self._stage = "ready"
                        self.on_result(config, None)
                    return
            except Exception as exc:
                self._fail(exc)
                return
        if self._future is not None or now < self._retry_at:
            return
        try:
            if self._stage == "limits":
                if not self.limits_client.service_is_ready():
                    self.error = "Waiting for hardware limit refresh service"
                    return
                request = self.trigger_type.Request()
                self._future = self.limits_client.call_async(request)
                self.error = "Reading hardware limits from servos..."
            elif self._stage == "parameters":
                ready = getattr(self.parameter_client, "parameters_are_ready", None)
                if ready is None:
                    ready = self.parameter_client.services_are_ready
                if not ready():
                    self.error = getattr(self.parameter_client, "limits_error", "Waiting for parameters")
                    return
                self._future = self.parameter_client.get_parameters(list(PARAMETERS))
            else:
                return
            self._started = now
        except Exception as exc:
            self._fail(exc)
