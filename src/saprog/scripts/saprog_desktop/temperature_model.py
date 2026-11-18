"""Pure bounded history and time-window rules for temperature plots."""

from collections import deque
import math


HISTORY_SECONDS = 300.0
STALE_SECONDS = 2.5
MAX_POINTS = 18000


class TemperatureHistory:
    def __init__(self, history_seconds=HISTORY_SECONDS, stale_seconds=STALE_SECONDS):
        self.history_seconds = history_seconds
        self.stale_seconds = stale_seconds
        self.points = {}
        self.selected = set()
        self.select_new = True
        self.window_seconds = history_seconds

    def add_servo(self, name):
        if name not in self.points:
            self.points[name] = deque(maxlen=MAX_POINTS)
            if self.select_new:
                self.selected.add(name)

    def add_sample(self, name, stamp, value):
        if not name or not math.isfinite(value) or not math.isfinite(stamp):
            return False
        self.add_servo(name)
        self.points[name].append((stamp, value))
        return True

    def select_all(self, selected):
        self.select_new = bool(selected)
        self.selected = set(self.points) if selected else set()

    def set_window(self, seconds):
        try:
            seconds = float(seconds)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(seconds) or not 1 <= seconds <= self.history_seconds:
            return False
        self.window_seconds = seconds
        return True

    def zoom(self, factor):
        if not math.isfinite(factor) or factor <= 0:
            return self.window_seconds
        self.window_seconds = min(self.history_seconds, max(1.0, self.window_seconds * factor))
        return self.window_seconds

    def visible(self, now):
        cutoff_history = now - self.history_seconds
        cutoff_window = now - self.window_seconds
        visible, fresh = [], 0
        for name, points in self.points.items():
            while points and points[0][0] < cutoff_history:
                points.popleft()
            recent = bool(points) and now - points[-1][0] <= self.stale_seconds
            fresh += int(recent)
            if name in self.selected:
                visible.append((name, [(stamp, value) for stamp, value in points if stamp >= cutoff_window]))
        return visible, fresh
