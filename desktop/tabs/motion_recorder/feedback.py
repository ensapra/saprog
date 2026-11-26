"""Joint feedback subscription, freshness and torque acknowledgements."""

import math

from widgets.servo_layout import SERVO_NAMES

from .constants import STATE_TIMEOUT, STATE_TOPIC


class JointFeedback:
    """Joint feedback subscription, freshness and torque acknowledgements.

    The tab supplies shared session state and UI callbacks; this helper owns
    only this workflow and never substitutes for the QWidget.
    """

    def __init__(self, tab):
        self.tab = tab

    def subscribe(self):
        """Subscribe for the lifetime of the tab and release on destruction."""
        tab = self.tab
        if tab.manager is not None:
            tab.manager.message_received.connect(tab.receive)
            tab.manager.command_result.connect(tab.command_result)
            try:
                tab.state_token = tab.manager.request_topic(
                    STATE_TOPIC, "sensor_msgs/msg/JointState",
                    package="saprog_dynamixel", executable="dynamixel_control_node",
                )
                manager_ref = [tab.manager]
                tab.manager.destroyed.connect(
                    lambda _obj=None: manager_ref.__setitem__(0, None)
                )
                tab.destroyed.connect(
                    lambda _obj=None: manager_ref[0].release(tab.state_token)
                    if manager_ref[0] is not None and tab.state_token else None
                )
            except (AttributeError, RuntimeError, ValueError) as error:
                tab.status.setText(f"Could not subscribe to joint feedback: {error}")
                tab.manager = None
        else:
            tab.status.setText("ROS manager is unavailable.")

    def receive(self, topic, message):
        """Cache valid servo positions and torque feedback."""
        tab = self.tab
        if topic != STATE_TOPIC or not isinstance(message, dict):
            return
        names = message.get("name", ())
        positions = message.get("position", ())
        efforts = message.get("effort", ())
        updated = False
        for index, joint in enumerate(names):
            if not isinstance(joint, str) or joint not in SERVO_NAMES:
                continue
            if index < len(positions):
                try:
                    value = float(positions[index])
                except (TypeError, ValueError, OverflowError):
                    continue
                if math.isfinite(value):
                    tab.positions[joint] = value
                    tab.position_updated_at[joint] = tab.clock()
                    updated = True
            if index < len(efforts):
                try:
                    tab.torque_states[joint] = bool(float(efforts[index]))
                except (TypeError, ValueError, OverflowError):
                    pass
        if updated:
            tab._update_pose_status()
            tab._update_controls()

    def _state_is_fresh(self, joints=None):
        """Check that every requested joint has reported a recent position."""
        tab = self.tab
        now = tab.clock()
        joints = tab._selected_joints() if joints is None else joints
        return all(
            joint in tab.positions
            and now - tab.position_updated_at.get(joint, float("-inf")) <= STATE_TIMEOUT
            for joint in joints
        )

    def set_leg_torque(self, enabled):
        """Request torque state changes for every servo in the selected leg."""
        tab = self.tab
        if tab.manager is None:
            tab.status.setText("ROS manager is unavailable.")
            return False
        if tab.recording or tab.pending_torque:
            return False
        joints = tab._selected_joints()
        if not joints:
            tab.status.setText("Select a leg first.")
            return False
        if tab._playback_joints() & set(joints):
            tab.status.setText("Stop recorded playback before changing torque on the playing leg.")
            return False
        tab.torque_failures.clear()
        try:
            for joint in joints:
                command_id = tab.manager.set_servo_torque(joint, enabled)
                tab.pending_torque[command_id] = (joint, enabled)
        except (RuntimeError, ValueError, TypeError, AttributeError) as error:
            tab.status.setText(f"Could not request torque change for the selected leg: {error}")
            tab._update_controls()
            return False
        state = "on" if enabled else "off"
        tab.status.setText(f"Requesting torque {state} for {len(joints)} servos…")
        tab._update_controls()
        return True

    def command_result(self, command_id, success, message):
        """Apply acknowledged torque states and report any failed servo request."""
        tab = self.tab
        pending = tab.pending_torque.pop(command_id, None)
        if pending is None:
            return
        joint, enabled = pending
        if success:
            tab.torque_states[joint] = enabled
        else:
            tab.torque_failures.add(joint)
        if not tab.pending_torque:
            if tab.torque_failures:
                failed = ", ".join(sorted(tab.torque_failures))
                tab.status.setText(f"Torque change failed for {failed}: {message}")
            else:
                state = "on" if enabled else "off"
                tab.status.setText(f"Torque {state} acknowledged for the selected leg.")
            tab._update_pose_status()
            tab._update_controls()
