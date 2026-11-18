"""UI recovery after lost service responses and failed action submissions."""

import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

from snapshot_player import SnapshotPlayer
from timeline_window import TimelineWindow
from saprog_desktop.torque import TorqueBatch


class AsyncFailuresTest(unittest.TestCase):
    def test_torque_timeout_unlocks_player_and_ignores_late_response(self):
        app = SnapshotPlayer.__new__(SnapshotPlayer)
        app.joints = ["servo_11", "servo_12"]
        app._update_buttons = Mock()
        app.status = NS(set=Mock())
        now = [0.0]
        class Request:
            def __init__(self):
                self.data = None

        class Future:
            def __init__(self, result=None):
                self.value, self.callback = result, None

            def done(self):
                return self.value is not None

            def result(self):
                return self.value

            def cancel(self):
                pass

            def add_done_callback(self, callback):
                self.callback = callback
                if self.done():
                    callback(self)

        success = Future(NS(success=True, message=""))
        late = Future()
        clients = {}
        for joint, future in zip(app.joints, (success, late)):
            client = Mock()
            client.service_is_ready.return_value = True
            client.call_async.return_value = future
            clients[joint] = client
        adapter = TorqueBatch(clients, Request, clock=lambda: now[0])
        adapter.start(False)
        batch = {"pending": set(app.joints), "failed": {}, "adapter": adapter}
        app.torque_batch = batch
        now[0] = 7.9
        app._check_torque_timeout()
        self.assertIs(app.torque_batch, batch)
        now[0] = 8.0
        app._check_torque_timeout()
        self.assertIsNone(app.torque_batch)
        self.assertEqual(set(batch["failed"]), {"servo_12"})
        self.assertIn("state unknown", app.status.set.call_args.args[0])
        app._update_buttons.assert_called_once()

        new_batch = {"pending": set(app.joints), "failed": {}, "adapter": adapter}
        app.torque_batch = new_batch
        late.value = NS(success=True, message="late")
        late.callback(late)
        self.assertIs(app.torque_batch, new_batch)
        self.assertEqual(new_batch["pending"], set(app.joints))

    def test_failed_timeline_submission_unlocks_editor_for_retry(self):
        app = TimelineWindow.__new__(TimelineWindow)
        app.steps = [{"kind": "wait", "seconds": 0.0}]
        app.run_steps = None
        app.run_index = 0
        app.joints, app.bounds = ["servo_11"], {"servo_11": (-1, 1)}
        app.loop = NS(get=lambda: False)
        app.on_end = NS(get=lambda: "Torque on")
        app._ready = lambda: True
        app.apply = lambda: True
        app._update_buttons = Mock()
        app.status = NS(set=Mock())
        app.close_after_cancel = False
        app.timeline_client = NS(send_goal_async=Mock(side_effect=RuntimeError("offline")))
        app.timeline_goal_handle = app.timeline_goal_future = None
        with patch("timeline_window.snapshot_player.PlayTimeline", NS(Goal=NS), create=True):
            app.play_timeline()
        self.assertIsNone(app.run_steps)
        self.assertIsNone(app.timeline_goal_future)
        self.assertIn("offline", app.status.set.call_args.args[0])
        # A second attempt must reach the client again, rather than stay locked.
        with patch("timeline_window.snapshot_player.PlayTimeline", NS(Goal=NS), create=True):
            app.play_timeline()
        self.assertEqual(app.timeline_client.send_goal_async.call_count, 2)


if __name__ == "__main__":
    unittest.main()
