"""UI-independent snapshot arrival and optional torque-release coordinator."""

import math

from .snapshot_arrival import arrival_progress

FRESH_SECONDS = 2.5
STABLE_SAMPLES = 3
STABLE_SECONDS = 1.0
ARRIVAL_TIMEOUT = 30.0


class PlaybackCoordinator:
    """Track one command through fresh telemetry, stable arrival, and release.

    The caller owns command publication and supplies its completion timestamp.
    ``observe`` accepts only complete, increasing, fresh samples; ``tick``
    returns ``waiting``, ``arrived``, ``release`` or ``failed``. Timeline dwell
    belongs to timeline execution, not this snapshot coordinator.
    """

    def __init__(self, joints, positions, tolerance, *, started_at, sent_stamp,
                 release_after=False, freshness=FRESH_SECONDS,
                 arrival_timeout=ARRIVAL_TIMEOUT, stable_samples=STABLE_SAMPLES,
                 stable_seconds=STABLE_SECONDS):
        if not joints or set(joints) != set(positions):
            raise ValueError("Playback targets must include every configured joint")
        if not _finite_number(tolerance) or tolerance < 0:
            raise ValueError("Target tolerance must be finite and nonnegative")
        if any(not _finite_number(value) for value in positions.values()):
            raise ValueError("Playback targets must be finite")
        self.joints = tuple(joints)
        self.targets = dict(positions)
        self.tolerance = tolerance
        self.started_at = started_at
        self.sent_stamp = sent_stamp
        self.deadline = started_at + arrival_timeout
        self.release_after = release_after
        self.freshness = freshness
        self.stable_samples_required = stable_samples
        self.stable_seconds_required = stable_seconds
        self.last_sample_stamp = 0
        self.stable_count = 0
        self.stable_since = None
        self.stable_first_stamp = 0
        self.state = "waiting"
        self.detail = "waiting for fresh readings from every configured joint"
        self._progress = {"targets": self.targets, "last_progress": started_at}

    def reset_stability(self):
        self.stable_count, self.stable_since, self.stable_first_stamp = 0, None, 0

    def observe(self, positions, *, received_at, stamp, controller_now, now=None):
        if self.state != "waiting":
            return self.state
        if (set(positions) != set(self.joints) or received_at < 0
                or any(not _finite_number(value) for value in positions.values())):
            self.reset_stability()
            self.detail = "waiting for fresh readings from every configured joint"
            return self.state
        age = (controller_now - stamp) / 1_000_000_000 if stamp else float("inf")
        if (received_at < self.started_at or received_at < 0
                or (now is not None and now - received_at > self.freshness)
                or stamp <= self.sent_stamp
                or stamp <= self.last_sample_stamp or age < 0 or age > self.freshness):
            self.reset_stability()
            self.detail = "waiting for fresh telemetry recorded after the command"
            return self.state
        self.last_sample_stamp = stamp
        within, stalled, self.detail = arrival_progress(
            self._progress, positions, self.joints, self.tolerance, received_at
        )
        if stalled:
            self.detail = f"No progress toward targets for 8 seconds: {self.detail}"
            self.state = "failed"
            return self.state
        if not within:
            self.reset_stability()
            return self.state
        if self.stable_since is None:
            self.stable_since, self.stable_first_stamp = received_at, stamp
        self.stable_count += 1
        source_span = (stamp - self.stable_first_stamp) / 1_000_000_000
        if (self.stable_count >= self.stable_samples_required
                and received_at - self.stable_since >= self.stable_seconds_required
                and source_span >= self.stable_seconds_required):
            self.state = "release" if self.release_after else "arrived"
        return self.state

    def tick(self, now):
        if self.state == "waiting" and now >= self.deadline:
            self.state = "failed"
            self.detail += "; timed out. Torque unchanged."
        return self.state

    def publication_failed(self, reason):
        self.state = "failed"
        self.detail = f"Command publication failed; torque was not released: {reason}"


def _finite_number(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
