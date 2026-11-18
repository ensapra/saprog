import tempfile
import unittest
from pathlib import Path

from saprog_desktop.player_preferences import load_torque_strength, save_torque_strength


class PlayerSettingsTest(unittest.TestCase):
    def test_first_use_defaults_to_full_strength_and_changes_survive_reload(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "saprog" / "snapshot_player.json"
            self.assertEqual(load_torque_strength(path), 1023)
            for strength in (512, 0, 1023):
                save_torque_strength(path, strength)
                self.assertEqual(load_torque_strength(path), strength)
            self.assertEqual(list(path.parent.iterdir()), [path])

    def test_invalid_settings_fall_back_to_full_strength(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "settings.json"
            for text in (
                "broken",
                "[]",
                "{}",
                '{"torque_strength": -1}',
                '{"torque_strength": 1024}',
                '{"torque_strength": true}',
                '{"torque_strength": "512"}',
                '{"torque_strength": 1.5}',
            ):
                path.write_text(text, encoding="utf-8")
                self.assertEqual(load_torque_strength(path), 1023)


if __name__ == "__main__":
    unittest.main()
