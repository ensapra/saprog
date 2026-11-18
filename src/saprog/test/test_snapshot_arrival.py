import unittest
from types import SimpleNamespace as NS

from saprog_desktop.snapshot_arrival import arrival_progress
from snapshot_player import SnapshotPlayer


class ArrivalTest(unittest.TestCase):
    def test_encoder_rounding_and_off_target(self):
        playback = {"targets": {"servo_11": 0}}
        self.assertTrue(arrival_progress(playback, {"servo_11": 0.102}, ["servo_11"], 0.1, 1)[0])
        self.assertFalse(arrival_progress(playback, {"servo_11": 0.11}, ["servo_11"], 0.1, 2)[0])

    def test_no_motion_is_reported_not_success(self):
        playback = {"targets": {"servo_13": -0.75}}
        self.assertFalse(arrival_progress(playback, {"servo_13": -0.4}, ["servo_13"], 0.1, 0)[1])
        within, stalled, detail = arrival_progress(
            playback, {"servo_13": -0.4}, ["servo_13"], 0.1, 8
        )
        self.assertFalse(within)
        self.assertTrue(stalled)
        self.assertIn("servo_13", detail)

    def test_progress_extends_stall_deadline(self):
        playback = {"targets": {"servo_13": -0.75}}
        arrival_progress(playback, {"servo_13": -0.4}, ["servo_13"], 0.1, 0)
        arrival_progress(playback, {"servo_13": -0.5}, ["servo_13"], 0.1, 7)
        self.assertFalse(arrival_progress(playback, {"servo_13": -0.5}, ["servo_13"], 0.1, 9)[1])

    def test_player_sends_once_and_unlocks_without_telemetry(self):
        app = SnapshotPlayer.__new__(SnapshotPlayer)
        app.joints = ["servo_11"]
        app.torque_strength = NS(get=lambda: 600)
        app.moving_speed = NS(get=lambda: 250)
        app.hold_after_load = NS(get=lambda: False)
        app.status = NS(set=lambda text: None)
        app.dependencies = NS(JointState=lambda: NS(header=NS()))
        app.node = NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: None)))
        locks, messages = [], []
        app._set_playback_controls = locks.append
        app.command_pub = NS(publish=messages.append)
        for target in (0.1, 0.2):
            app._start_playback("pose", {"servo_11": target})
            app._playback_tick()
            self.assertIsNone(app.playback)
            app._playback_tick()
        self.assertEqual([message.position for message in messages], [[0.1], [0.2]])
        self.assertEqual(locks, [True, False, True, False])

    def test_player_publication_failure_unlocks(self):
        app = SnapshotPlayer.__new__(SnapshotPlayer)
        app.joints = ["servo_11"]
        app.torque_strength = NS(get=lambda: 600)
        app.moving_speed = NS(get=lambda: 250)
        statuses, locks = [], []
        app.status = NS(set=statuses.append)
        app.dependencies = NS(JointState=lambda: NS(header=NS()))
        app.node = NS(get_clock=lambda: NS(now=lambda: NS(to_msg=lambda: None)))
        app._set_playback_controls = locks.append

        def fail(message):
            raise RuntimeError("publisher unavailable")

        app.command_pub = NS(publish=fail)
        app._start_playback("pose", {"servo_11": 0.1})
        app._playback_tick()
        self.assertIsNone(app.playback)
        self.assertEqual(locks, [True, False])
        self.assertIn("publication failed", statuses[-1])

if __name__ == "__main__":
    unittest.main()
