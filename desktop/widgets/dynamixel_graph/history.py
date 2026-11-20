"""Bounded, validated sample history for the DYNAMIXEL graph."""

import math
import time
from collections import deque

from widgets.servo_layout import LEG_SERVOS

SERVO_COLORS = (
    "#70b5f5", "#f2a65a", "#7fd1ae", "#e883a8", "#b49aff", "#e6d36a",
    "#58c5d0", "#fa806e", "#9bc76b", "#d58bdc", "#f0a0ce", "#a7c6ed",
)
DATA_TYPES = ("temperature", "voltage", "speed")
HISTORY_SECONDS = 300.0
MAX_SAMPLES = 18000


class TelemetryHistory:
    """Store up to five minutes of finite samples for each servo and metric."""

    def __init__(self):
        self.servo_names = tuple(name for _leg, names in LEG_SERVOS for name in names)
        self.samples = {
            kind: {name: deque(maxlen=MAX_SAMPLES) for name in self.servo_names}
            for kind in DATA_TYPES
        }

    def add(self, servo_name, data_type, value, timestamp=None):
        """Append a valid sample, returning its numeric value or ``None``."""
        if servo_name not in self.samples["temperature"] or data_type not in self.samples:
            return None
        try:
            numeric_value = float(value)
            stamp = time.monotonic() if timestamp is None else float(timestamp)
        except (TypeError, ValueError, OverflowError):
            return None
        if not math.isfinite(numeric_value) or not math.isfinite(stamp):
            return None

        points = self.samples[data_type][servo_name]
        points.append((stamp, numeric_value))
        cutoff = stamp - HISTORY_SECONDS
        while points and points[0][0] < cutoff:
            points.popleft()
        return numeric_value
