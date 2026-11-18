"""Filesystem operations shared by the snapshot and timeline windows.

Validation belongs to the document models; this module only handles storage.
"""
import json
import os
from pathlib import Path
import tempfile


def write_json_atomic(path, document):
    """Replace a JSON file only after the complete document is safely written.

    The caller creates the parent directory and decides whether to overwrite.
    A temporary file beside the destination keeps replacement on one filesystem.
    """
    path = Path(path)
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="w", encoding="utf-8", dir=path.parent, suffix=".tmp", delete=False
        ) as stream:
            temporary = Path(stream.name)
            json.dump(document, stream, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        # Close first: Windows cannot replace a file while its handle is open.
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)


def default_snapshot_directory():
    """Find the workspace from either source scripts or installed executables."""
    for parent in Path(__file__).resolve().parents:
        if (parent / "src" / "saprog" / "config" / "saprog.yaml").is_file():
            return str(parent / "snapshots")
    return str(Path.cwd() / "snapshots")
