"""Read radian snapshots using the shared, ROS-independent validation rules."""

import json
from pathlib import Path

from documents.snapshot_units import validate_snapshot


def load_keyframe(path, servos, bounds=None):
    with Path(path).open("r", encoding="utf-8") as stream:
        document = json.load(stream)
    positions = document.get("positions") if isinstance(document, dict) else None
    if isinstance(positions, dict) and set(positions) - set(servos):
        raise ValueError("Keyframe contains unknown servos")
    return validate_snapshot(document, bounds=bounds or None)
