import unittest

from saprog_desktop.snapshot_units import from_radians, joint_bounds, to_radians, validate_snapshot


class SnapshotUnitsTest(unittest.TestCase):
    def test_asymmetric_limits_and_reversed_direction(self):
        bounds = joint_bounds([13, 33], [512, 512], [1, -1], [350, 570], [440, 660])
        for limits in bounds.values():
            for normalized in (-1, -0.3, 0, 0.7, 1):
                radians = to_radians(normalized, limits, "normalized")
                self.assertAlmostEqual(from_radians(radians, limits, "normalized"), normalized)
            self.assertEqual(to_radians(-1, limits, "normalized"), limits[0])
            self.assertEqual(to_radians(1, limits, "normalized"), limits[1])
            self.assertAlmostEqual(to_radians(0, limits, "normalized"), sum(limits) / 2)

    def test_legacy_and_new_snapshots(self):
        bounds = {"servo_13": (-0.8, -0.3)}
        for version, units, value in (
            (1, None, -0.55),
            (2, "normalized", 0),
            (2, "radians", -0.55),
        ):
            doc = dict(
                schema_version=version, units=units, name="pose", positions={"servo_13": value}
            )
            self.assertAlmostEqual(validate_snapshot(doc, bounds, bounds)[1]["servo_13"], -0.55)
        for value in (-1.01, 1.01, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                validate_snapshot(
                    dict(
                        schema_version=2,
                        units="normalized",
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

if __name__ == "__main__":
    unittest.main()
