"""UI recovery after lost service responses and failed action submissions."""

import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock

from saprog_desktop.torque import TorqueBatch
from snapshot_player import SnapshotPlayer
from snapshot_window import SnapshotWindow


class FakeSetBool:
    class Request:
        def __init__(self):
            self.data = None


class AsyncFailuresTest(unittest.TestCase):
    def test_creator_release_supersedes_pending_fix_and_queued_pose(self):
        app = SnapshotWindow.__new__(SnapshotWindow)
        app.joints = ["servo_11"]
        app.external_busy = lambda: True
        app.stop_external_operation = None
        app.playback_state = {"phase": "sending"}
        app._set_playback_controls = Mock()
        old_token = object()
        app.pending = {"servo_11": (old_token, 0, True)}
        old_adapter = Mock()
        app.torque_adapters = {"servo_11": old_adapter}
        client = Mock()
        client.service_is_ready.return_value = True
        app.services = {"servo_11": client}
        app.dependencies = NS(SetBool=FakeSetBool)
        app.fixed, app.torque_state = {"servo_11"}, {}
        app.status = NS(set=Mock())
        app._update_joint_buttons = Mock()
        app.release_all()
        self.assertIsNone(app.playback_state)
        old_adapter.close.assert_called_once()
        self.assertFalse(client.call_async.call_args.args[0].data)
        self.assertFalse(app.pending["servo_11"][2])
        app._apply_torque_result("servo_11", True, old_token, "success")
        self.assertNotIn("servo_11", app.fixed)
        app.release_all()
        self.assertEqual(client.call_async.call_count, 2)

    def test_player_release_retries_and_sends_to_available_services(self):
        app = SnapshotPlayer.__new__(SnapshotPlayer)
        app.joints = ["servo_11", "servo_12"]
        ready, unavailable = Mock(), Mock()
        ready.service_is_ready.return_value = True
        unavailable.service_is_ready.return_value = False
        app.torque_clients = dict(zip(app.joints, (ready, unavailable)))
        app.dependencies = NS(SetBool=FakeSetBool)
        app.torque_adapter = TorqueBatch(app.torque_clients, FakeSetBool)
        app.torque_batch = None
        app.playback = {"phase": "sending"}
        app.status = NS(set=Mock())
        app._update_buttons = Mock()
        for _ in range(2):
            app.disable_all_torque()
            self.assertIsNone(app.playback)
            self.assertFalse(ready.call_async.call_args.args[0].data)
            self.assertIn("not_sent", app.torque_adapter.outcomes["servo_12"])
        self.assertEqual(ready.call_async.call_count, 2)
        unavailable.call_async.assert_not_called()

    def test_torque_timeout_unlocks_player_and_ignores_late_response(self):
        app = SnapshotPlayer.__new__(SnapshotPlayer)
        app.joints = ["servo_11", "servo_12"]
        app._update_buttons = Mock()
        app.status = NS(set=Mock())
        now = [0.0]
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
        adapter = TorqueBatch(clients, FakeSetBool, clock=lambda: now[0])
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

if __name__ == "__main__":
    unittest.main()
