"""Snapshot import and timeline load/save actions."""

import copy
import re
from datetime import datetime
from pathlib import Path

from documents.timeline_domain import mirror_mode, mirrored_positions
from documents.timeline_storage import load_snapshot_reference, save_timeline
from documents.timeline_storage import load_timeline as load_timeline_document
from PySide6.QtWidgets import QFileDialog, QMessageBox


class TimelineFiles:
    """Snapshot import and timeline load/save actions.

    The tab supplies shared session state and UI callbacks; this helper owns
    only this workflow and never substitutes for the QWidget.
    """

    def __init__(self, tab):
        self.tab = tab

    def add_snapshot_pose(self, path=None):
        """Add the selected-leg targets from a saved snapshot as a keyframe."""
        tab = self.tab
        if tab.recording or tab.base_loop_locked or tab.playback.owner is tab:
            return False
        if path is None:
            path, _filter = QFileDialog.getOpenFileName(
                tab, "Add pose from snapshot", str(tab.snapshot_directory),
                "Snapshot JSON (*.json)",
            )
        if not path:
            return False
        try:
            snapshot = load_snapshot_reference(
                str(path), tab.snapshot_directory, bounds=tab.playback.bounds,
            )
            joints = tab._selected_joints()
            missing = [joint for joint in joints if joint not in snapshot["positions"]]
            if missing:
                raise ValueError(
                    "Snapshot is missing selected-leg servos: " + ", ".join(missing)
                )
            tab._append_pose(
                {joint: snapshot["positions"][joint] for joint in joints},
                label=snapshot["name"],
            )
        except (OSError, ValueError, TypeError, KeyError) as error:
            tab.status.setText(f"Could not add snapshot pose: {error}")
            return False
        tab.status.setText(f"Added snapshot pose: {snapshot['name']}.")
        return True

    def _confirm_timeline_replacement(self):
        tab = self.tab
        if not tab.timeline_entries:
            return True
        return QMessageBox.question(
            tab,
            "Replace current timeline?",
            "Discard the current timeline from the Motion Recorder?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No,
        ) == QMessageBox.StandardButton.Yes

    def new_timeline(self):
        """Start a blank timeline after confirming replacement of current content."""
        tab = self.tab
        if tab.recording or tab.playback.owner is tab:
            tab.status.setText("Stop recording or playback before starting a new timeline.")
            return False
        if not tab._confirm_timeline_replacement():
            return False
        tab.frames.clear()
        tab.timeline_entries.clear()
        tab.recorded_joints.clear()
        tab.base_loop_locked = False
        tab.timeline_path = None
        tab.timeline_options = {"loop": True, "on_end": "on"}
        tab.smooth_playback.setChecked(False)
        tab._refresh_pose_list()
        tab._update_controls()
        tab.status.setText("New timeline. Capture or import poses to add steps.")
        return True

    def load_timeline(self, path=None):
        """Load a timeline while retaining its waits and per-step settings."""
        tab = self.tab
        if tab.recording or tab.playback.owner is tab:
            tab.status.setText("Stop recording or playback before loading a timeline.")
            return False
        if path is None:
            path, _filter = QFileDialog.getOpenFileName(
                tab, "Load timeline", str(tab.sequence_directory),
                "Timeline JSON (*.json)",
            )
        if not path:
            return False
        if not tab._confirm_timeline_replacement():
            return False
        try:
            document = load_timeline_document(
                path, tab.snapshot_directory, bounds=tab.playback.bounds,
            )
            entries = []
            frames = []
            for step in document["steps"]:
                if step.get("kind") == "wait":
                    entries.append(copy.deepcopy(step))
                    continue
                snapshot = load_snapshot_reference(
                    step["snapshot"], tab.snapshot_directory, bounds=tab.playback.bounds,
                )
                positions = mirrored_positions(snapshot["positions"], mirror_mode(step))
                frame = {
                    "positions": tab._clamp_pose(positions),
                    "captured_at": tab.clock(),
                    "recorded": False,
                    "label": snapshot["name"],
                    "timeline_step": copy.deepcopy(step),
                }
                entries.append(frame)
                frames.append(frame)
        except (OSError, ValueError, TypeError, KeyError) as error:
            tab.status.setText(f"Could not load timeline: {error}")
            return False

        tab.timeline_entries = entries
        tab.frames = frames
        tab.recorded_joints = {
            joint for frame in frames for joint in frame["positions"]
        }
        tab.base_loop_locked = False
        tab.timeline_path = Path(path).expanduser().resolve()
        tab.timeline_options = {
            "loop": document.get("loop", False),
            "on_end": document.get("on_end", "on"),
        }
        tab.smooth_playback.setChecked(document.get("smooth_playback", False))
        tab._refresh_pose_list()
        if tab.timeline_entries:
            tab.pose_list.selectRow(0)
        tab._update_controls()
        wait_count = sum(entry.get("kind") == "wait" for entry in entries)
        tab.status.setText(
            f"Loaded {len(frames)} poses and {wait_count} wait steps from {tab.timeline_path.name}."
        )
        return True

    def save_timeline_as(self):
        """Choose a new path for the current timeline."""
        tab = self.tab
        path, _filter = QFileDialog.getSaveFileName(
            tab, "Save timeline as",
            str(tab.sequence_directory / "recorded_motion.json"),
            "Timeline JSON (*.json)",
        )
        return tab.save_timeline(path) if path else False

    def save_timeline(self, path=None):
        """Save the current timeline without discarding its step settings."""
        tab = self.tab
        if tab.recording:
            tab.stop_recording()
        if not tab.timeline_entries:
            tab.status.setText("Add at least one timeline step before saving.")
            return False
        if path is None:
            path = tab.timeline_path
            if path is None:
                path, _filter = QFileDialog.getSaveFileName(
                    tab, "Save timeline",
                    str(tab.sequence_directory / "recorded_motion.json"),
                    "Timeline JSON (*.json)",
                )
                if not path:
                    return False
        destination = Path(path).expanduser()
        if not destination.suffix:
            destination = destination.with_suffix(".json")
        destination = destination.resolve()
        folder_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", destination.stem).strip("._")
        if not folder_name:
            tab.status.setText("Choose a timeline file name with letters or numbers.")
            return False
        unique_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        pose_directory = tab.snapshot_directory / "recordings" / f"{folder_name}_{unique_id}"
        try:
            steps = tab._write_steps(
                tab.timeline_entries, pose_directory, tab.snapshot_directory,
            )
            save_timeline(
                destination, steps,
                on_end=tab.timeline_options["on_end"],
                loop=tab.timeline_options["loop"],
                smooth_playback=tab.smooth_playback.isChecked(),
                snapshot_resolver=lambda reference: load_snapshot_reference(
                    reference, tab.snapshot_directory, bounds=tab.playback.bounds,
                ),
            )
        except (OSError, ValueError, TypeError, KeyError) as error:
            tab.status.setText(f"Could not save recorded timeline: {error}")
            return False
        for entry, step in zip(tab.timeline_entries, steps):
            if "positions" in entry:
                entry["timeline_step"] = copy.deepcopy(step)
                entry.pop("positions_changed", None)
            else:
                entry.clear()
                entry.update(copy.deepcopy(step))
        tab.timeline_path = destination
        tab._update_controls()
        tab.status.setText(f"Saved {len(tab.frames)} poses as {destination}.")
        return True
