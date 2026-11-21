"""Offline timeline editing using the shared document rules and atomic storage."""

import copy
from pathlib import Path

from documents.snapshot_units import clamp_to_bounds
from documents.timeline_domain import (
    CONTROL_MODES,
    MIRROR_MODES,
    control_groups,
    mirror_mode,
    mirrored_positions,
    motion_settings,
    regroup_motion,
    timeline_rows,
    transition_duration,
    wait_seconds,
)
from documents.timeline_presenter import TimelineEditorPresenter
from documents.timeline_storage import (
    load_snapshot_reference,
    load_timeline,
    make_snapshot_reference,
    save_timeline,
)

__all__ = [
    "CONTROL_MODES", "MIRROR_MODES", "TimelineDocument", "control_groups", "motion_settings",
    "timeline_rows", "transition_duration", "mirror_mode",
]


class TimelineDocument:
    """Own a local document without controller, hardware, or playback dependencies."""

    def __init__(self, snapshot_directory, bounds=None):
        self.presenter = TimelineEditorPresenter()
        self.snapshot_directory = Path(snapshot_directory).expanduser().resolve()
        self.bounds = bounds
        self.path = None
        self.on_end = "on"
        self.loop = False
        self.smooth_playback = False

    @property
    def steps(self):
        return self.presenter.steps

    @property
    def dirty(self):
        return self.presenter.dirty

    def new(self, snapshot_directory=None):
        self.presenter.replace([])
        self.path = None
        self.on_end, self.loop = "on", False
        self.smooth_playback = False
        if snapshot_directory is not None:
            self.snapshot_directory = Path(snapshot_directory).expanduser().resolve()

    def snapshot(self, index):
        snapshot = load_snapshot_reference(
            self.steps[index]["snapshot"], self.snapshot_directory, bounds=self.bounds,
        )
        snapshot["positions"] = mirrored_positions(snapshot["positions"], mirror_mode(self.steps[index]))
        if self.bounds:
            snapshot["positions"] = {
                joint: clamp_to_bounds(value, self.bounds[joint])
                if joint in self.bounds else value
                for joint, value in snapshot["positions"].items()
            }
        return snapshot

    def add_snapshot(self, path):
        reference = make_snapshot_reference(path, self.snapshot_directory)
        load_snapshot_reference(reference, self.snapshot_directory, bounds=self.bounds)
        return self.presenter.add_snapshot(reference)

    def add_wait(self, index, seconds=1.0):
        wait_seconds({"seconds": seconds})
        return self.presenter.add_wait(index, seconds)

    def update_wait(self, index, seconds):
        step = copy.deepcopy(self.steps[index])
        step["seconds"] = wait_seconds({"seconds": seconds})
        self.presenter.update_step(index, step)

    def update_duration(self, index, mode, seconds):
        step = copy.deepcopy(self.steps[index])
        step["duration"] = transition_duration({"duration": {"mode": mode, "seconds": seconds}})
        self.presenter.update_step(index, step)

    def update_mirror(self, index, mode):
        if mode not in ("None", "Horizontal", "Vertical", "Horizontal + Vertical"):
            raise ValueError("Unknown snapshot mirror mode")
        step = copy.deepcopy(self.steps[index])
        step["mirror"] = mode
        mirror_mode(step)
        self.presenter.update_step(index, step)

    def set_control_mode(self, index, mode):
        joints = list(self.snapshot(index)["positions"])
        self.presenter.update_step(index, regroup_motion(self.steps[index], mode, joints))

    def update_motion(self, index, mode, group, key, value):
        if key not in ("speed", "torque") or type(value) is not int or not 1 <= value <= 1023:
            raise ValueError("Speed and torque must be integers from 1 to 1023")
        step = copy.deepcopy(self.steps[index])
        joints = list(self.snapshot(index)["positions"])
        names = control_groups(mode, joints)[group]
        resolved = motion_settings(step, joints)
        pair = {
            setting: round(sum(resolved[joint][setting] for joint in names) / len(names))
            for setting in ("speed", "torque")
        }
        pair[key] = value
        step["control_mode"] = mode
        if mode == "Global Control":
            step["motion"], step["overrides"] = pair, {}
        else:
            step.setdefault("overrides", {}).update({joint: dict(pair) for joint in names})
        motion_settings(step, joints)
        self.presenter.update_step(index, step)

    def set_lifecycle(self, on_end, loop):
        if on_end not in ("on", "off") or type(loop) is not bool:
            raise ValueError("Invalid timeline end settings")
        if (on_end, loop) != (self.on_end, self.loop):
            self.on_end, self.loop = on_end, loop
            self.presenter.dirty = True

    def load(self, path, snapshot_directory=None):
        directory = Path(snapshot_directory or self.snapshot_directory).expanduser().resolve()
        document = load_timeline(path, directory, bounds=self.bounds)
        # Commit only after all references, settings and legacy migration validate.
        self.presenter.replace(document["steps"])
        self.snapshot_directory = directory
        self.path = Path(path).resolve()
        self.on_end, self.loop = document.get("on_end", "on"), document.get("loop", False)
        self.smooth_playback = document.get("smooth_playback", False)

    def set_smooth_playback(self, enabled):
        if type(enabled) is not bool:
            raise ValueError("Smooth playback must be true or false")
        if enabled != self.smooth_playback:
            self.smooth_playback = enabled
            self.presenter.dirty = True

    def save(self, path=None, *, absolute_references=False):
        destination = Path(path).resolve() if path else self.path
        if destination is None:
            raise ValueError("Choose a timeline file first")
        steps = copy.deepcopy(self.steps)
        if absolute_references:
            # Mixed-folder documents must resolve to the same targets for all
            # readers, regardless of their configured snapshot directory.
            for step in steps:
                if step.get("kind") != "wait":
                    step["snapshot"] = str(
                        (self.snapshot_directory / step["snapshot"]).resolve()
                    )
        save_timeline(
            destination, steps, self.on_end, self.loop,
            smooth_playback=self.smooth_playback,
            snapshot_resolver=lambda reference: load_snapshot_reference(
                reference, self.snapshot_directory, bounds=self.bounds,
            ),
        )
        # Commit reference changes only after the atomic write succeeds.
        self.presenter.replace(steps)
        self.path = destination
