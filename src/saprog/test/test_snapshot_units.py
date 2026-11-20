import unittest

from saprog_desktop.snapshot_units import joint_bounds, to_radians, validate_snapshot


class SnapshotUnitsTest(unittest.TestCase):
    def test_asymmetric_limits_and_reversed_direction(self):
        bounds = joint_bounds([13, 33], [512, 512], [1, -1], [350, 570], [440, 660])
        self.assertLess(bounds['servo_13'][1], 0)
        self.assertLess(bounds['servo_33'][1], 0)
        for limits in bounds.values():
            for value in (limits[0], sum(limits) / 2, limits[1]):
                self.assertEqual(to_radians(value, limits, "radians"), value)
            for value in (limits[0] - 0.01, limits[1] + 0.01):
                with self.assertRaises(ValueError):
                    to_radians(value, limits, "radians")

    def test_legacy_and_new_snapshots(self):
        bounds = {"servo_13": (-0.8, -0.3)}
        for version, units, value in (
            (1, None, -0.55),
            (2, "radians", -0.55),
        ):
            doc = dict(
                schema_version=version, units=units, name="pose", positions={"servo_13": value}
            )
            if units is None:
                doc.pop("units")
            self.assertAlmostEqual(validate_snapshot(doc, bounds, bounds)[1]["servo_13"], -0.55)
        for value in (-1.01, 1.01, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                validate_snapshot(
                    dict(
                        schema_version=2,
                        units="radians",
                        name="pose",
                        positions={"servo_13": value},
                    ),
                    bounds,
                    bounds,
                )
        with self.assertRaises(ValueError):
            validate_snapshot(
                dict(schema_version=2, name="pose", positions={"servo_13": 0}), bounds, bounds
            )

    def test_other_units_are_rejected_even_with_measured_limits(self):
        for version in (1, 2):
            for units in ("normalized", "degrees", None):
                with self.subTest(version=version, units=units):
                    with self.assertRaisesRegex(ValueError, "must use radians"):
                        validate_snapshot({
                            "schema_version": version, "units": units, "name": "pose",
                            "positions": {"servo_11": 0.0},
                        }, bounds={"servo_11": (-0.2, 0.6)})

if __name__ == "__main__":
    unittest.main()
