"""Playback ownership, temporary preview files and step selection."""

import tempfile
from pathlib import Path


class MotionPreview:
    """Playback ownership, temporary preview files and step selection.

    The tab supplies shared session state and UI callbacks; this helper owns
    only this workflow and never substitutes for the QWidget.
    """

    def __init__(self, tab):
        self.tab = tab

    def _playback_joints(self):
        """Return joints currently controlled by this tab's recorded playback."""
        tab = self.tab
        if tab.playback.owner is not tab:
            return set()
        return {
            joint
            for step in tab.playback.steps
            if isinstance(step.get("snapshot"), dict)
            for joint in step["snapshot"]["positions"]
        }

    def _playback_status(self, message):
        """Surface playback results and release its temporary snapshots when stopped."""
        tab = self.tab
        tab.status.setText(message)
        if tab.playback.owner is not tab:
            tab._highlight_playback_step(None)
            tab._cleanup_playback_files()
            if tab.overlay_recording:
                tab._discard_overlay(f"Overlay recording stopped with playback: {message}")
        tab._update_controls()

    def _cleanup_playback_files(self):
        """Remove temporary preview poses after playback no longer needs them."""
        tab = self.tab
        tab._preview_row = None
        if tab._playback_temp_dir is not None:
            tab._playback_temp_dir.cleanup()
            tab._playback_temp_dir = None

    def _playback_step_changed(self, index):
        """Map a single-step preview back to its original timeline row."""
        tab = self.tab
        tab._highlight_playback_step(tab._preview_row if tab._preview_row is not None else index)

    def play_recorded_motion(self):
        """Preview the current timeline so another leg can be recorded alongside it."""
        tab = self.tab
        return tab._play_motion()

    def play_selected_step(self):
        """Play just the selected timeline step once using its saved settings."""
        tab = self.tab
        row = tab.pose_list.currentRow()
        if not 0 <= row < len(tab.timeline_entries):
            tab.status.setText("Select a timeline step to play.")
            return False
        return tab._play_motion(row)

    def _play_motion(self, row=None):
        """Run the timeline or one step through the shared playback lifecycle."""
        tab = self.tab
        if tab.manager is None:
            tab.status.setText("ROS manager is unavailable.")
            return False
        if tab.playback.owner is tab:
            tab.status.setText("Recorded motion is already playing.")
            return False
        if tab.recording or not tab.timeline_entries:
            tab.status.setText("Stop recording and add a timeline step before playback.")
            return False
        preview = tempfile.TemporaryDirectory(prefix="saprog_motion_preview_")
        try:
            steps = tab._write_steps(tab.timeline_entries, Path(preview.name))
            if row is not None:
                steps = [steps[row]]
            tab._preview_row = row
            tab._playback_temp_dir = preview
            tab.playback.start_steps(
                tab, steps, repeat=row is None and tab.timeline_options["loop"],
                smooth=tab.smooth_playback.isChecked(),
            )
        except (OSError, RuntimeError, ValueError, TypeError, KeyError) as error:
            preview.cleanup()
            tab._cleanup_playback_files()
            tab.status.setText(f"Could not play recorded motion: {error}")
            tab._update_controls()
            return False
        tab.status.setText(
            f"Playing selected step {row + 1} once." if row is not None else
            "Playing timeline in a loop. Select another leg and record it on top."
            if tab.timeline_options["loop"] else "Playing timeline once."
        )
        tab._update_controls()
        return True

    def stop_playback(self):
        """Stop the recorded motion preview."""
        tab = self.tab
        if tab.playback.owner is not tab:
            return False
        tab.playback.stop("Recorded motion playback stopped.")
        return True
