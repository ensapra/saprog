from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from snapshot_arrival import arrival_progress
from snapshot_player import SnapshotPlayer


class ArrivalTest(unittest.TestCase):
    def test_encoder_rounding_and_off_target(self):
        playback = {"targets": {"servo_11": 0}}
        self.assertTrue(arrival_progress(playback, {"servo_11": .102}, ["servo_11"], .1, 1)[0])
        self.assertFalse(arrival_progress(playback, {"servo_11": .11}, ["servo_11"], .1, 2)[0])

    def test_no_motion_is_reported_not_success(self):
        playback = {"targets": {"servo_13": -.75}}
        self.assertFalse(arrival_progress(playback, {"servo_13": -.4}, ["servo_13"], .1, 0)[1])
        within, stalled, detail = arrival_progress(playback, {"servo_13": -.4}, ["servo_13"], .1, 8)
        self.assertFalse(within)
        self.assertTrue(stalled)
        self.assertIn("servo_13", detail)

    def test_progress_extends_stall_deadline(self):
        playback = {"targets": {"servo_13": -.75}}
        arrival_progress(playback, {"servo_13": -.4}, ["servo_13"], .1, 0)
        arrival_progress(playback, {"servo_13": -.5}, ["servo_13"], .1, 7)
        self.assertFalse(arrival_progress(playback, {"servo_13": -.5}, ["servo_13"], .1, 9)[1])

    def test_player_completes_fresh_stable_samples_near_limit(self):
        app = SnapshotPlayer.__new__(SnapshotPlayer)
        app.joints = ["servo_11"]
        app.bounds = {"servo_11": (0, 1)}
        app.target_tolerance = .1
        app.status = NS(set=lambda text: None)
        finished = []
        app._finish_playback = finished.append
        app.playback = dict(phase="waiting", targets={"servo_11": 0},
                           deadline=30, sent_stamp=1_000_000_000,
                           last_sample_stamp=0, stable_count=0,
                           stable_since=None, stable_first_stamp=0, release_after=False)
        for now in (2, 3, 4):
            app.node = NS(get_clock=lambda: NS(now=lambda: NS(nanoseconds=now * 1_000_000_000)))
            message = NS(name=["servo_11"], position=[-.002],
                         header=NS(stamp=NS(sec=now, nanosec=0)))
            with patch("snapshot_player.time.monotonic", return_value=now):
                app._on_state(message)
                app._playback_tick()
        self.assertEqual(len(finished), 1)
        self.assertIn("reached and stable", finished[0])


if __name__ == "__main__":
    unittest.main()
