"""Arrival diagnostics; called only for fresh, complete, post-command samples."""

from .snapshot_units import AX_TICK_RADIANS
NO_PROGRESS_SECONDS = 8.0


def arrival_progress(playback, positions, joints, tolerance, now):
    # Goal positions are rounded to the nearest encoder tick by the controller.
    effective = tolerance + AX_TICK_RADIANS / 2
    errors = {joint: abs(positions[joint] - playback["targets"][joint]) for joint in joints}
    remaining = {joint: error for joint, error in errors.items() if error > effective}
    if not remaining:
        return True, False, f"All joints within {effective:.3f} rad; checking stable readings"
    best = playback.setdefault("best_errors", {})
    progress = False
    for joint, error in remaining.items():
        if joint not in best or error < best[joint] - AX_TICK_RADIANS / 2:
            best[joint] = error
            progress = True
    if progress:
        playback["last_progress"] = now
    stalled = now - playback.get("last_progress", now) >= NO_PROGRESS_SECONDS
    worst = sorted(remaining, key=remaining.get, reverse=True)[:3]
    detail = ", ".join(f"{joint}: {remaining[joint]:.3f} rad away" for joint in worst)
    return False, stalled, f"{len(remaining)} joint(s) outside {effective:.3f} rad: {detail}"
