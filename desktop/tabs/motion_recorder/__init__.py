"""Capture manually guided joint motion and save it as a timeline.

The tab coordinates focused helpers for presentation, robot feedback, pose
editing, recording, preview playback and document actions. Its action methods
remain stable for Qt signals and workspace integrations.
"""

import time
from pathlib import Path

from PySide6.QtCore import QTimer, Signal
from PySide6.QtWidgets import QWidget

from tabs.controller.playback import ControllerPlayback

from .constants import STATE_TIMEOUT as STATE_TIMEOUT
from .constants import STATE_TOPIC as STATE_TOPIC
from .feedback import JointFeedback
from .files import QFileDialog as QFileDialog
from .files import QMessageBox as QMessageBox
from .files import TimelineFiles
from .poses import PoseEditor
from .preview import MotionPreview
from .recording import MotionRecording
from .storage import write_steps
from .view import RecorderView

TAB_TITLE = "Motion Recorder"
TAB_ORDER = 36


class MotionRecorderTab(QWidget):
    """Coordinate motion recorder workflows and own their shared session state."""

    pose_edit_requested = Signal(object, object)

    def __init__(
        self, ros_manager=None, parent=None, *, snapshot_directory=None,
        sequence_directory=None, clock=time.monotonic,
    ):
        super().__init__(parent)
        workspace = Path(__file__).resolve().parents[3]
        self.manager = ros_manager
        self.snapshot_directory = Path(
            snapshot_directory or workspace / "snapshots"
        ).expanduser().resolve()
        self.sequence_directory = Path(
            sequence_directory or workspace / "sequences"
        ).expanduser().resolve()
        self.clock = clock
        self.state_token = None
        self.positions = {}
        self.position_updated_at = {}
        self.torque_states = {}
        self.pending_torque = {}
        self.torque_failures = set()
        self.recorded_joints = set()
        self.frames = []
        self.timeline_entries = []
        self.timeline_path = None
        self.timeline_options = {"loop": True, "on_end": "on"}
        self.base_loop_locked = False
        self.recording = False
        self.overlay_recording = False
        self.overlay_samples = {}
        self.copied_leg = None
        self.playing_row = None
        self._preview_row = None
        self.overlay_last_index = None
        self.last_capture_at = None
        self.playback = ControllerPlayback(
            self.manager, self.snapshot_directory, self, clock=clock,
        )
        self.playback.status_changed.connect(self._playback_status)
        self.playback.step_ready.connect(self._overlay_step_ready)
        self.playback.step_changed.connect(self._playback_step_changed)
        self._playback_temp_dir = None

        self._feedback = JointFeedback(self)
        self._poses = PoseEditor(self)
        self._recording = MotionRecording(self)
        self._preview = MotionPreview(self)
        self._files = TimelineFiles(self)
        self._view = RecorderView(self)
        self._view.build()

        self.record_timer = QTimer(self)
        self.record_timer.setInterval(50)
        self.record_timer.timeout.connect(self._record_tick)
        self.stop_playback_button.setEnabled(False)
        self._feedback.subscribe()
        self._selection_changed()

    def showEvent(self, event):
        super().showEvent(event)
        self.playback.limits.refresh()

    def _selected_joints(self):
        """Return the configured servos for the selected leg."""
        return tuple(self.leg_picker.currentData() or ())

    def _selection_changed(self, _index=None):
        """Refresh pose controls when the selected leg changes."""
        self._update_pose_status()
        self._update_controls()

    def receive(self, topic, message):
        """Cache valid servo positions and torque feedback."""
        return self._feedback.receive(topic, message)

    def _state_is_fresh(self, joints=None):
        """Check that every requested joint has reported a recent position."""
        return self._feedback._state_is_fresh(joints)

    def _capture_joints(self):
        """Return recorded joints plus the currently selected leg."""
        return self.recorded_joints | set(self._selected_joints())

    def _playback_joints(self):
        """Return joints currently controlled by this tab's recorded playback."""
        return self._preview._playback_joints()

    def _playback_status(self, message):
        """Surface playback results and release its temporary snapshots when stopped."""
        return self._preview._playback_status(message)

    def _cleanup_playback_files(self):
        """Remove temporary preview poses after playback no longer needs them."""
        return self._preview._cleanup_playback_files()

    def _update_pose_status(self):
        """Show the latest selected-leg positions and torque states."""
        return self._view._update_pose_status()

    def _update_controls(self):
        """Enable only actions supported by the current connection and state."""
        return self._view._update_controls()

    def set_leg_torque(self, enabled):
        """Request torque state changes for every servo in the selected leg."""
        return self._feedback.set_leg_torque(enabled)

    def command_result(self, command_id, success, message):
        """Apply acknowledged torque states and report any failed servo request."""
        return self._feedback.command_result(command_id, success, message)

    def _capture_current_pose(self, captured_at=None, *, recorded=False):
        """Append a composed pose from the selected and previously recorded legs."""
        return self._recording._capture_current_pose(captured_at, recorded=recorded)

    def _append_pose(self, pose, captured_at=None, *, recorded=False, label=None):
        """Append a pose and add its summary to the keyframe list."""
        return self._poses._append_pose(pose, captured_at, recorded=recorded, label=label)

    def _clamp_pose(self, pose):
        return self._poses._clamp_pose(pose)

    def _set_pose_row(self, row, frame, pose_number):
        return self._view._set_pose_row(row, frame, pose_number)

    def _set_wait_row(self, row, step):
        return self._view._set_wait_row(row, step)

    def add_snapshot_pose(self, path=None):
        """Add the selected-leg targets from a saved snapshot as a keyframe."""
        return self._files.add_snapshot_pose(path)

    def _edit_pose_at(self, row, column):
        """Open the double-clicked pose through the shared edit action."""
        return self._poses._edit_pose_at(row, column)

    def edit_selected_pose(self):
        """Open the selected pose in the workspace's Static Animator."""
        return self._poses.edit_selected_pose()

    def apply_pose_edit(self, frame, positions):
        """Apply targets to the original step even if the selection has changed."""
        return self._poses.apply_pose_edit(frame, positions)

    def _confirm_timeline_replacement(self):
        return self._files._confirm_timeline_replacement()

    def new_timeline(self):
        """Start a blank timeline after confirming replacement of current content."""
        return self._files.new_timeline()

    def load_timeline(self, path=None):
        """Load a timeline while retaining its waits and per-step settings."""
        return self._files.load_timeline(path)

    def remove_selected_leg_data(self):
        """Remove the selected leg's targets from a pose without deleting its step."""
        return self._poses.remove_selected_leg_data()

    def remove_selected_pose(self):
        """Remove the selected pose or wait step from the timeline."""
        return self._poses.remove_selected_pose()

    def _refresh_pose_list(self):
        """Rebuild pose and wait rows after timeline edits."""
        return self._view._refresh_pose_list()

    def _playback_step_changed(self, index):
        """Map a single-step preview back to its original timeline row."""
        return self._preview._playback_step_changed(index)

    def _highlight_playback_step(self, index):
        """Mark the active pose or wait without changing the user's selection."""
        return self._view._highlight_playback_step(index)

    def copy_leg(self):
        """Copy every pose of the selected leg, in timeline order."""
        return self._poses.copy_leg()

    def paste_leg(self):
        """Paste copied targets into the selected leg, shifting pose indices cyclically."""
        return self._poses.paste_leg()

    def capture_pose(self):
        """Capture a manually positioned leg as a timeline pose."""
        return self._recording.capture_pose()

    def start_recording(self):
        """Record the base loop or overlay another leg on an existing loop."""
        return self._recording.start_recording()

    def start_overlay_recording(self):
        """Capture a different leg once per step of the playing base loop."""
        return self._recording.start_overlay_recording()

    def _record_tick(self):
        """Capture the latest pose at the selected sample interval."""
        return self._recording._record_tick()

    def _overlay_step_ready(self, index):
        """Capture each loop pose after its playback command is acknowledged."""
        return self._recording._overlay_step_ready(index)

    def _capture_overlay_step(self, index):
        """Store selected-leg positions for the given fixed timeline step."""
        return self._recording._capture_overlay_step(index)

    def _finish_overlay_recording(self):
        """Merge a full loop of leg samples without adding timeline steps."""
        return self._recording._finish_overlay_recording()

    def _discard_overlay(self, message):
        """Stop an incomplete leg overlay without changing the base timeline."""
        return self._recording._discard_overlay(message)

    def stop_recording(self):
        """Stop timed capture and retain the final available pose."""
        return self._recording.stop_recording()

    def play_recorded_motion(self):
        """Preview the current timeline so another leg can be recorded alongside it."""
        return self._preview.play_recorded_motion()

    def play_selected_step(self):
        """Play just the selected timeline step once using its saved settings."""
        return self._preview.play_selected_step()

    def _play_motion(self, row=None):
        """Run the timeline or one step through the shared playback lifecycle."""
        return self._preview._play_motion(row)

    def stop_playback(self):
        """Stop the recorded motion preview."""
        return self._preview.stop_playback()

    def closeEvent(self, event):
        """Stop timers and release playback resources when this tab closes."""
        self.record_timer.stop()
        self.playback.close()
        self._cleanup_playback_files()
        super().closeEvent(event)

    def _write_steps(self, entries, pose_directory, reference_directory=None):
        return write_steps(
            entries, pose_directory, reference_directory,
            sample_interval=self.sample_interval.value(),
            speed=self.speed.value(), strength=self.strength.value(),
        )

    def save_timeline_as(self):
        """Choose a new path for the current timeline."""
        return self._files.save_timeline_as()

    def save_timeline(self, path=None):
        """Save the current timeline without discarding its step settings."""
        return self._files.save_timeline(path)


def create_tab(ros_manager=None):
    """Build the hardware-backed motion recorder."""
    return MotionRecorderTab(ros_manager=ros_manager)
