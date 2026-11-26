"""Serialize recorder frames to timeline steps without depending on Qt."""

import copy

from documents.snapshot_files import write_json_atomic
from documents.timeline_domain import mirror_mode, mirrored_positions
from documents.timeline_storage import make_snapshot_reference


def write_steps(entries, pose_directory, reference_directory=None, *,
                sample_interval, speed, strength):
    """Write changed pose files while retaining loaded timeline step settings."""
    steps = []
    pose_number = 0
    for index, entry in enumerate(entries):
        if "positions" not in entry:
            steps.append(copy.deepcopy(entry))
            continue
        frame = entry
        source_step = frame.get("timeline_step")
        if source_step is not None and not frame.get("positions_changed"):
            steps.append(copy.deepcopy(source_step))
            continue

        pose_number += 1
        pose_directory.mkdir(parents=True, exist_ok=True)
        pose_name = f"pose_{pose_number:04d}"
        pose_path = pose_directory / f"{pose_name}.json"
        positions = frame["positions"]
        if source_step is not None:
            positions = mirrored_positions(positions, mirror_mode(source_step), inverse=True)
        write_json_atomic(pose_path, {
            "schema_version": 1,
            "name": pose_name,
            "positions": positions,
        })
        reference = (
            make_snapshot_reference(pose_path, reference_directory)
            if reference_directory is not None
            else str(pose_path.resolve())
        )
        if source_step is not None:
            step = copy.deepcopy(source_step)
            step["snapshot"] = reference
            if "overrides" in step:
                step["overrides"] = {
                    joint: settings for joint, settings in step["overrides"].items()
                    if joint in positions
                }
            steps.append(step)
        else:
            next_entry = entries[index + 1] if index + 1 < len(entries) else None
            seconds = sample_interval
            if (
                next_entry is not None and "positions" in next_entry
                and frame["recorded"] and next_entry["recorded"]
            ):
                seconds = next_entry["captured_at"] - frame["captured_at"]
            steps.append({
                "snapshot": reference,
                "motion": {"speed": speed, "torque": strength},
                "duration": {"mode": "seconds", "seconds": max(0.05, seconds)},
            })
    return steps
