"""Own timeline editor document state and whole-entry edit operations."""

import copy


class TimelineEditorPresenter:
    """Keep editable steps and dirty state independent of Tk and ROS objects."""

    def __init__(self, steps=None):
        self.steps = copy.deepcopy(steps) if steps is not None else []
        self.dirty = False

    def replace(self, steps, dirty=False):
        self.steps = copy.deepcopy(steps)
        self.dirty = dirty

    def update_step(self, index, step):
        value = copy.deepcopy(step)
        if value != self.steps[index]:
            self.steps[index] = value
            self.dirty = True

    def add_snapshot(self, reference, speed=1023, torque=1023):
        self.steps.append({"snapshot": reference, "motion": {"speed": speed, "torque": torque}, "overrides": {}})
        self.dirty = True
        return len(self.steps) - 1

    def add_wait(self, index, seconds=1.0):
        index = max(0, min(index, len(self.steps)))
        self.steps.insert(index, {"kind": "wait", "seconds": seconds})
        self.dirty = True
        return index

    def move(self, index, delta):
        target = index + delta
        if not 0 <= index < len(self.steps) or not 0 <= target < len(self.steps):
            return None
        self.steps[index], self.steps[target] = self.steps[target], self.steps[index]
        self.dirty = True
        return target

    def duplicate(self, index):
        if not 0 <= index < len(self.steps):
            return None
        target = index + 1
        self.steps.insert(target, copy.deepcopy(self.steps[index]))
        self.dirty = True
        return target

    def remove(self, index):
        if not 0 <= index < len(self.steps):
            return None
        del self.steps[index]
        self.dirty = True
        return min(index, len(self.steps) - 1) if self.steps else None

    def mark_saved(self):
        self.dirty = False
