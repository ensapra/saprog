"""Timed base capture and acknowledged per-step leg overlays."""

from documents.snapshot_units import clamp_to_bounds


class MotionRecording:
    """Timed base capture and acknowledged per-step leg overlays.

    The tab supplies shared session state and UI callbacks; this helper owns
    only this workflow and never substitutes for the QWidget.
    """

    def __init__(self, tab):
        self.tab = tab

    def _capture_current_pose(self, captured_at=None, *, recorded=False):
        """Append a composed pose from the selected and previously recorded legs."""
        tab = self.tab
        joints = tab._capture_joints()
        if not tab._state_is_fresh(joints):
            return False
        pose = {joint: tab.positions[joint] for joint in sorted(joints)}
        return tab._append_pose(pose, captured_at, recorded=recorded)

    def capture_pose(self):
        """Capture a manually positioned leg as a timeline pose."""
        tab = self.tab
        if tab.recording or tab.playback.owner is tab:
            tab.status.setText("Stop recording or playback before adding a pose.")
            return False
        if tab.base_loop_locked:
            tab.status.setText("The first recording fixed the timeline length; use a loop overlay.")
            return False
        if not tab._capture_current_pose():
            tab.status.setText("Cannot capture: waiting for fresh feedback from every servo in the leg.")
            return False
        tab.status.setText(f"Captured pose {len(tab.frames)}.")
        return True

    def start_recording(self):
        """Record the base loop or overlay another leg on an existing loop."""
        tab = self.tab
        if tab.base_loop_locked or tab.playback.owner is tab:
            return tab.start_overlay_recording()
        if tab.recording:
            return False
        if tab.playback.owner is tab:
            tab.status.setText("Stop playback before recording the base leg.")
            return False
        if not tab._state_is_fresh(tab._capture_joints()):
            tab.status.setText("Cannot record: waiting for fresh feedback from every servo in the leg.")
            return False
        if any(tab.torque_states.get(joint) is not False for joint in tab._selected_joints()):
            tab.status.setText("Request torque off for the selected leg and wait for acknowledgement.")
            return False
        now = tab.clock()
        if not tab._capture_current_pose(now, recorded=True):
            return False
        tab.recording = True
        tab.last_capture_at = now
        tab.record_timer.start()
        tab.status.setText("Recording. Move the released leg by hand; torque will remain off.")
        tab._update_controls()
        return True

    def start_overlay_recording(self):
        """Capture a different leg once per step of the playing base loop."""
        tab = self.tab
        if tab.recording:
            return False
        if tab._preview_row is not None:
            tab.status.setText("Play the full recorded loop before recording another leg.")
            return False
        if tab.playback.owner is not tab:
            tab.status.setText("Play the recorded loop before recording another leg over it.")
            return False
        joints = tab._selected_joints()
        if tab._playback_joints() & set(joints):
            tab.status.setText("Select a leg that is not part of the playing loop.")
            return False
        if not tab._state_is_fresh(joints):
            tab.status.setText("Cannot record: waiting for fresh feedback from every servo in the leg.")
            return False
        if any(tab.torque_states.get(joint) is not False for joint in joints):
            tab.status.setText("Request torque off for the selected leg and wait for acknowledgement.")
            return False
        tab.overlay_samples = {}
        tab.overlay_last_index = None
        tab.overlay_recording = True
        tab.recording = True
        if tab.playback.phase in ("wait", "arrival"):
            tab._capture_overlay_step(tab.playback.index)
        if not tab.overlay_recording:
            return True
        tab.status.setText(
            "Recording this leg over one complete loop. The timeline length will not change."
        )
        tab._update_controls()
        return True

    def _record_tick(self):
        """Capture the latest pose at the selected sample interval."""
        tab = self.tab
        if not tab.recording:
            return
        if tab.overlay_recording:
            return
        now = tab.clock()
        if not tab._state_is_fresh(tab._capture_joints()):
            tab.stop_recording()
            tab.status.setText(
                "Recording stopped because feedback became stale for a recorded leg."
            )
            return
        if now - tab.last_capture_at < tab.sample_interval.value():
            return
        if tab._capture_current_pose(now, recorded=True):
            tab.last_capture_at = now

    def _overlay_step_ready(self, index):
        """Capture each loop pose after its playback command is acknowledged."""
        tab = self.tab
        if tab.overlay_recording:
            tab._capture_overlay_step(index)

    def _capture_overlay_step(self, index):
        """Store selected-leg positions for the given fixed timeline step."""
        tab = self.tab
        if tab.playback.owner is not tab:
            tab._discard_overlay("Overlay recording stopped because playback ended.")
            return
        if index == tab.overlay_last_index or not 0 <= index < len(tab.timeline_entries):
            return
        frame = tab.timeline_entries[index]
        if "positions" not in frame:
            return
        joints = tab._selected_joints()
        if not tab._state_is_fresh(joints):
            tab._discard_overlay(
                "Overlay recording stopped because feedback for the selected leg became stale."
            )
            return
        tab.overlay_samples[index] = {
            joint: clamp_to_bounds(tab.positions[joint], tab.playback.bounds[joint])
            if tab.playback.bounds and joint in tab.playback.bounds else tab.positions[joint]
            for joint in joints
        }
        tab.overlay_last_index = index
        if len(tab.overlay_samples) == len(tab.frames):
            tab._finish_overlay_recording()

    def _finish_overlay_recording(self):
        """Merge a full loop of leg samples without adding timeline steps."""
        tab = self.tab
        tab.record_timer.stop()
        tab.recording = False
        tab.overlay_recording = False
        for index, pose in tab.overlay_samples.items():
            frame = tab.timeline_entries[index]
            frame["positions"].update(pose)
            frame["positions_changed"] = True
        tab.recorded_joints.update(tab._selected_joints())
        tab.overlay_samples = {}
        tab.overlay_last_index = None
        tab._refresh_pose_list()
        if tab.playback.owner is tab:
            tab.playback.stop("Leg overlay captured; playback stopped.")
        else:
            tab._cleanup_playback_files()
        tab.status.setText(
            f"Recorded one loop over {len(tab.frames)} timeline poses. "
            "Play the updated loop to layer another leg."
        )
        tab._update_controls()

    def _discard_overlay(self, message):
        """Stop an incomplete leg overlay without changing the base timeline."""
        tab = self.tab
        tab.record_timer.stop()
        tab.recording = False
        tab.overlay_recording = False
        tab.overlay_samples = {}
        tab.overlay_last_index = None
        tab.status.setText(message)
        tab._update_controls()

    def stop_recording(self):
        """Stop timed capture and retain the final available pose."""
        tab = self.tab
        if not tab.recording:
            return False
        if tab.overlay_recording:
            tab._discard_overlay(
                "Overlay recording stopped before a full loop; no poses were added."
            )
            return True
        tab.record_timer.stop()
        tab.recording = False
        tab._capture_current_pose(recorded=True)
        tab.base_loop_locked = True
        tab.status.setText(
            f"Base timeline set to {len(tab.frames)} poses. Play it to record other legs over one loop."
        )
        tab._update_controls()
        return True
