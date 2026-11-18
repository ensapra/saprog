"""Compatibility imports for timeline rules, storage and goal serialization."""

from saprog_desktop.timeline_action import timeline_action_data
from saprog_desktop.timeline_domain import (
    CONTROL_MODES,
    FIXED_SECONDS,
    MOTION_STEP,
    UNTIL_REACHED,
    WAIT_STEP,
    control_groups,
    lifecycle_settings,
    motion_settings,
    regroup_motion,
    timeline_rows,
    transition_duration,
    validate_timeline,
    wait_seconds,
)
from saprog_desktop.timeline_storage import (
    load_snapshot_reference,
    load_timeline,
    make_snapshot_reference,
    reference_for_legacy_snapshot,
    resolve_timeline_steps,
    save_timeline,
)

# Older callers used the servo IDs directly; derive them from the shared joint
# membership so the Creator and Timeline cannot disagree about leg assignment.
from saprog_desktop.snapshot_units import LEG_JOINTS

LEGS = {
    label.removesuffix(" Leg").capitalize(): tuple(int(joint.removeprefix("servo_")) for joint in joints)
    for label, joints in LEG_JOINTS
}
