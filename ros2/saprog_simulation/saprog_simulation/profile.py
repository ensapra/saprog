"""Explicit offline bounds input; never consult hardware or its session cache."""

import copy
import math

SERVO_NAMES = {f"servo_{leg}{joint}" for leg in (1, 2, 3, 4) for joint in (1, 2, 3)}


def validate_bounds_profile(document):
    """Return an owned, JSON-safe profile with complete, ordered radian bounds.

    This is the bounds portion of the planned simulation configuration. The
    provenance describes assumptions or an explicitly imported hardware report;
    it does not make these limits safe for hardware publication.
    """
    if not isinstance(document, dict):
        raise ValueError("Simulation bounds profile must be an object")
    if set(document) != {"schema_version", "name", "provenance", "units", "servo_bounds"}:
        raise ValueError("Unexpected or missing simulation bounds profile fields")
    if type(document["schema_version"]) is not int or document["schema_version"] != 1:
        raise ValueError("Unsupported simulation bounds profile version")
    if document["units"] != "radians":
        raise ValueError("Simulation bounds must use radians")
    for key in ("name", "provenance"):
        if not isinstance(document[key], str) or not document[key].strip():
            raise ValueError(f"Simulation bounds profile requires {key}")
    bounds = document["servo_bounds"]
    if not isinstance(bounds, dict) or set(bounds) != SERVO_NAMES:
        raise ValueError("Simulation bounds must cover exactly the twelve configured servos")
    for servo, pair in bounds.items():
        if (not isinstance(pair, (list, tuple)) or len(pair) != 2
                or any(type(value) not in (int, float) or not math.isfinite(value) for value in pair)
                or pair[0] >= pair[1]):
            raise ValueError(f"Invalid simulation bounds for {servo}: expected finite lower < upper")
        # Until explicit joint initialization is implemented, the imported model
        # starts at zero. Reject a profile incompatible with that starting state.
        if not pair[0] <= 0 <= pair[1]:
            raise ValueError(f"Initial zero pose outside simulation bounds for {servo}: {pair}")
    result = copy.deepcopy(document)
    result["servo_bounds"] = {servo: list(pair) for servo, pair in bounds.items()}
    return result
