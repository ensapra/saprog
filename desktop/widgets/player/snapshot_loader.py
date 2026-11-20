"""Read radian snapshots using the shared, ROS-independent validation rules."""

import json
import sys
from pathlib import Path

_SCRIPTS = str(Path(__file__).resolve().parents[3] / "src" / "saprog" / "scripts")
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

from saprog_desktop.snapshot_units import validate_snapshot  # noqa: E402


def load_keyframe(path, servos, bounds=None):
    with Path(path).open("r", encoding="utf-8") as stream:
        document = json.load(stream)
    positions = document.get("positions") if isinstance(document, dict) else None
    if isinstance(positions, dict) and set(positions) - set(servos):
        raise ValueError("Keyframe contains unknown servos")
    return validate_snapshot(document, bounds=bounds or None)
