"""Timeline reference storage and transactional document load/migration."""

import copy
import json
import math
from pathlib import Path

from .snapshot_files import write_json_atomic
from .snapshot_units import validate_snapshot
from .timeline_domain import validate_timeline


def make_snapshot_reference(path, directory):
    path, directory = Path(path).resolve(), Path(directory).resolve()
    try:
        return path.relative_to(directory).as_posix()
    except ValueError:
        return str(path)


def load_snapshot_reference(reference, directory, joints=None, bounds=None):
    if not isinstance(reference, str) or not reference.strip():
        raise ValueError("Snapshot reference must be a non-empty path")
    path = Path(reference)
    if not path.is_absolute():
        path = (Path(directory) / path).resolve()
        try:
            path.relative_to(Path(directory).resolve())
        except ValueError as exc:
            raise ValueError("Snapshot reference must stay inside the snapshot directory") from exc
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    name, positions = validate_snapshot(document, joints, bounds)
    return {"schema_version": 1, "name": name, "positions": positions}


def reference_for_legacy_snapshot(snapshot, directory, joints=None, bounds=None):
    name, positions = validate_snapshot(snapshot, joints, bounds)
    matches = []
    for path in Path(directory).glob("*.json"):
        try:
            with path.open(encoding="utf-8") as stream:
                candidate = json.load(stream)
            candidate_name, candidate_positions = validate_snapshot(candidate, joints, bounds)
        except (OSError, ValueError, TypeError, KeyError):
            continue
        if candidate_name != name or candidate_positions.keys() != positions.keys():
            continue
        if all(math.isclose(candidate_positions[j], positions[j], rel_tol=0, abs_tol=1e-9) for j in positions):
            matches.append(path)
    if not matches:
        raise ValueError(f"No matching snapshot pose named '{name}' is present in {directory}")
    return make_snapshot_reference(sorted(matches, key=lambda item: item.name)[0], directory)


def resolve_timeline_steps(steps, snapshot_resolver):
    resolved = copy.deepcopy(steps)
    for step in resolved:
        if step.get("kind") != "wait" and isinstance(step.get("snapshot"), str):
            step["snapshot"] = snapshot_resolver(step["snapshot"])
    return resolved


def load_timeline(path, directory, joints=None, bounds=None):
    """Read, migrate and fully validate before returning a timeline document."""
    path = Path(path)
    document = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("Unsupported timeline file")

    def resolve(reference):
        return load_snapshot_reference(reference, directory, joints, bounds)

    candidate = copy.deepcopy(document)
    if candidate.get("schema_version") == 1:
        migrated_steps = []
        for step in candidate.get("steps", []):
            if isinstance(step, dict) and isinstance(step.get("snapshot"), dict):
                step = {**step, "snapshot": reference_for_legacy_snapshot(step["snapshot"], directory, joints, bounds)}
            migrated_steps.append(step)
        candidate["steps"] = migrated_steps
        candidate["schema_version"] = 2
    validate_timeline(candidate, joints, bounds, resolve)
    return candidate


def save_timeline(path, steps, on_end="on", loop=False, snapshot_resolver=None):
    document = {"kind": "saprog_timeline", "schema_version": 2, "steps": steps,
                "on_end": on_end, "loop": loop}
    validate_timeline(document, snapshot_resolver=snapshot_resolver)
    write_json_atomic(path, document)
