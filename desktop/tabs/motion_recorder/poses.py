"""Pose insertion, editing, deletion and cyclic leg copying."""

from documents.snapshot_units import clamp_to_bounds


class PoseEditor:
    """Pose insertion, editing, deletion and cyclic leg copying.

    The tab supplies shared session state and UI callbacks; this helper owns
    only this workflow and never substitutes for the QWidget.
    """

    def __init__(self, tab):
        self.tab = tab

    def _append_pose(self, pose, captured_at=None, *, recorded=False, label=None):
        """Append a pose and add its summary to the keyframe list."""
        tab = self.tab
        timestamp = tab.clock() if captured_at is None else captured_at
        pose = tab._clamp_pose(pose)
        joints = tuple(pose)
        tab.recorded_joints.update(joints)
        frame = {
            "positions": dict(pose), "captured_at": timestamp, "recorded": recorded,
            "label": label,
        }
        tab.frames.append(frame)
        tab.timeline_entries.append(frame)
        row = tab.pose_list.rowCount()
        tab.pose_list.insertRow(row)
        tab._set_pose_row(row, frame, len(tab.frames))
        tab.pose_list.selectRow(row)
        tab._update_controls()
        return True

    def _clamp_pose(self, pose):
        tab = self.tab
        bounds = tab.playback.bounds or {}
        return {
            joint: clamp_to_bounds(value, bounds[joint]) if joint in bounds else value
            for joint, value in pose.items()
        }

    def _edit_pose_at(self, row, column):
        """Open the double-clicked pose through the shared edit action."""
        tab = self.tab
        tab.pose_list.setCurrentCell(row, column)
        tab.edit_selected_pose()

    def edit_selected_pose(self):
        """Open the selected pose in the workspace's Static Animator."""
        tab = self.tab
        if tab.recording or tab.playback.owner is tab:
            return False
        row = tab.pose_list.currentRow()
        if not 0 <= row < len(tab.timeline_entries):
            return False
        frame = tab.timeline_entries[row]
        if "positions" not in frame:
            tab.status.setText("Select a pose row to edit, not a wait step.")
            return False

        tab.pose_edit_requested.emit(tab, frame)
        return True

    def apply_pose_edit(self, frame, positions):
        """Apply targets to the original step even if the selection has changed."""
        tab = self.tab
        if tab.recording or tab.playback.owner is tab:
            raise ValueError("Stop recording or playback before applying pose changes")
        row = next((i for i, entry in enumerate(tab.timeline_entries) if entry is frame), None)
        if row is None:
            raise ValueError("This pose is no longer in the Motion Recorder timeline")
        frame["positions"] = tab._clamp_pose(positions)
        frame["positions_changed"] = True
        tab.recorded_joints.update(frame["positions"])
        tab._refresh_pose_list()
        tab.pose_list.selectRow(row)
        tab.status.setText(f"Updated pose {row + 1}.")
        tab._update_controls()
        return True

    def remove_selected_leg_data(self):
        """Remove the selected leg's targets from a pose without deleting its step."""
        tab = self.tab
        if tab.recording or tab.playback.owner is tab:
            return False
        row = tab.pose_list.currentRow()
        if not 0 <= row < len(tab.timeline_entries):
            return False
        frame = tab.timeline_entries[row]
        if "positions" not in frame:
            tab.status.setText("Select a pose row to remove leg data.")
            return False
        joints = set(tab._selected_joints())
        positions = frame["positions"]
        removed = joints & positions.keys()
        if not removed:
            tab.status.setText("The selected leg has no data in this pose.")
            return False
        if not positions.keys() - joints:
            tab.status.setText("Remove the entire step to delete its last leg.")
            return False
        frame["positions"] = {
            joint: value for joint, value in positions.items() if joint not in joints
        }
        frame["positions_changed"] = True
        tab.recorded_joints = {
            joint for pose in tab.frames for joint in pose["positions"]
        }
        tab._refresh_pose_list()
        tab.pose_list.selectRow(row)
        tab._update_controls()
        tab.status.setText(f"Removed the selected leg's data from pose {row + 1}.")
        return True

    def remove_selected_pose(self):
        """Remove the selected pose or wait step from the timeline."""
        tab = self.tab
        if tab.recording:
            tab.status.setText("Stop recording before removing a pose.")
            return False
        if tab.playback.owner is tab:
            tab.status.setText("Stop playback before removing a pose.")
            return False
        index = tab.pose_list.currentRow()
        if not 0 <= index < len(tab.timeline_entries):
            tab.status.setText("Select a timeline step to remove.")
            return False
        entry = tab.timeline_entries.pop(index)
        if "positions" in entry:
            tab.frames = [frame for frame in tab.frames if frame is not entry]
        else:
            tab.status.setText(f"Removed wait step {index + 1}.")
        if "positions" in entry:
            tab.status.setText(f"Removed pose {index + 1}.")
        tab.recorded_joints = {
            joint for frame in tab.frames for joint in frame["positions"]
        }
        tab._refresh_pose_list()
        tab._update_controls()
        return True

    def copy_leg(self):
        """Copy every pose of the selected leg, in timeline order."""
        tab = self.tab
        if tab.recording or tab.playback.owner is tab or not tab.frames:
            return False
        joints = tab._selected_joints()
        if any(any(joint not in frame["positions"] for joint in joints) for frame in tab.frames):
            tab.status.setText("The selected leg must have all three targets in every pose to copy it.")
            return False
        tab.copied_leg = [tuple(frame["positions"][joint] for joint in joints) for frame in tab.frames]
        tab.status.setText(f"Copied {tab.leg_picker.currentText()} from {len(tab.frames)} poses.")
        tab._update_controls()
        return True

    def paste_leg(self):
        """Paste copied targets into the selected leg, shifting pose indices cyclically."""
        tab = self.tab
        if tab.recording or tab.playback.owner is tab or tab.copied_leg is None:
            return False
        if len(tab.copied_leg) != len(tab.frames) or not tab.frames:
            tab.status.setText("The number of poses changed since copying; copy the leg again.")
            return False
        joints = tab._selected_joints()
        offset = tab.paste_offset.value()
        clamped = False
        for index, values in enumerate(tab.copied_leg):
            frame = tab.frames[(index + offset) % len(tab.frames)]
            targets = dict(zip(joints, values))
            bounded = tab._clamp_pose(targets)
            clamped = clamped or bounded != targets
            frame["positions"].update(bounded)
            frame["positions_changed"] = True
        tab.recorded_joints.update(joints)
        tab._refresh_pose_list()
        tab._update_controls()
        tab.status.setText(
            f"Pasted into {tab.leg_picker.currentText()} with offset {offset} poses."
            + (" Some targets were clamped to servo limits." if clamped else "")
        )
        return True
