"""Pure serializer for the saprog PlayTimeline action goal."""

from saprog_desktop.timeline_domain import (
    FIXED_SECONDS,
    MOTION_STEP,
    UNTIL_REACHED,
    WAIT_STEP,
    motion_settings,
    transition_duration,
    wait_seconds,
)


def timeline_action_data(steps, joints, loop=False, run_end_action=True, end_torque_enabled=True):
    """Return goal fields matching PlayTimeline.action, with row-major joint data."""
    data = {
        "joint_names": list(joints),
        "step_kinds": [],
        "duration_modes": [],
        "durations": [],
        "positions": [],
        "speeds": [],
        "torque_limits": [],
        "loop": bool(loop),
        "run_end_action": bool(run_end_action and not loop),
        "end_torque_enabled": bool(end_torque_enabled),
    }
    for step in steps:
        if step.get("kind") == "wait":
            data["step_kinds"].append(WAIT_STEP)
            data["duration_modes"].append(FIXED_SECONDS)
            data["durations"].append(wait_seconds(step))
            data["positions"].extend([0.0] * len(joints))
            data["speeds"].extend([1] * len(joints))
            data["torque_limits"].extend([1] * len(joints))
            continue
        duration = transition_duration(step)
        settings = motion_settings(step, joints)
        data["step_kinds"].append(MOTION_STEP)
        data["duration_modes"].append(FIXED_SECONDS if duration["mode"] == "seconds" else UNTIL_REACHED)
        data["durations"].append(duration["seconds"])
        data["positions"].extend(step["snapshot"]["positions"][joint] for joint in joints)
        data["speeds"].extend(settings[joint]["speed"] for joint in joints)
        data["torque_limits"].extend(settings[joint]["torque"] for joint in joints)
    return data
