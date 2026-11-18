"""Snapshot Player preferences stored outside the installed package."""

import json
import os
from pathlib import Path

from .snapshot_files import write_json_atomic


def player_settings_path():
    """Return the per-user settings path, independent of snapshot storage."""
    base = os.environ.get("APPDATA") if os.name == "nt" else os.environ.get("XDG_CONFIG_HOME")
    return (Path(base) if base else Path.home() / ".config") / "saprog" / "snapshot_player.json"


def load_torque_strength(path):
    """Load a valid AX torque strength; use full strength for invalid settings."""
    try:
        settings = json.loads(Path(path).read_text(encoding="utf-8"))
        value = settings.get("torque_strength") if isinstance(settings, dict) else None
        if type(value) is int and 0 <= value <= 1023:
            return value
    except (OSError, ValueError, UnicodeError):
        pass
    return 1023


def save_torque_strength(path, value):
    """Atomically save the Player's per-window torque strength preference."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    write_json_atomic(path, {"torque_strength": value})
