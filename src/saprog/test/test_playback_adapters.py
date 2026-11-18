import unittest
from unittest.mock import Mock

from saprog_desktop.playback import PlaybackCoordinator
from saprog_desktop.snapshot_config import decode_configuration
from saprog_desktop.snapshot_config import SnapshotConfigurationClient
from saprog_desktop.snapshot_units import AX_RANGE, AX_TICK_RADIANS
from saprog_desktop.torque import TorqueBatch


class Parameter:
    def __init__(self, kind, value):
        setattr(self, f"{kind}_value", value)


def config_values(ids=(1, 2), zeros=(512, 512), directions=(1, -1),
                  tolerance=0.05, lower=(100, 120), upper=(900, 880)):
    return [Parameter("integer_array", ids), Parameter("integer_array", zeros),
            Parameter("integer_array", directions), Parameter("double", tolerance),
            Parameter("integer_array", lower), Parameter("integer_array", upper)]


class Future:
    def __init__(self, result=None):
        self.value, self.callback = result, None
        self.was_cancelled = False

    def done(self):
        return self.value is not None

    def result(self):
        if isinstance(self.value, Exception):
            raise self.value
        return self.value

    def cancel(self):
        self.was_cancelled = True

    def add_done_callback(self, callback):
        self.callback = callback
        if self.done():
            callback(self)


class PlaybackAdapterTests(unittest.TestCase):
    def test_config_decodes_complete_calibration_and_limits(self):
        result = decode_configuration(config_values())
        self.assertEqual(result.joints, ("servo_1", "servo_2"))
        self.assertEqual(result.tolerance, 0.05)
        self.assertAlmostEqual(result.bounds["servo_1"][0], (100 - 512) * AX_RANGE / 1023)
        self.assertAlmostEqual(AX_TICK_RADIANS, AX_RANGE / 1023)

    def test_config_rejects_short_arrays_bad_ids_calibration_limits_and_tolerance(self):
        cases = [config_values()[:-1], config_values(ids=(1, 1)),
                 config_values(zeros=(1024, 512)), config_values(directions=(0, -1)),
                 config_values(lower=(900, 120), upper=(100, 880)),
                 config_values(ids=()), config_values(zeros=(512,)),
                 config_values(tolerance=float("nan")), config_values(tolerance=-0.1)]
        for values in cases:
            with self.subTest(values=values), self.assertRaises(ValueError):
                decode_configuration(values)

    def test_configuration_refreshes_limits_before_atomic_parameter_commit(self):
        now = [0.0]
        events, results = [], []
        limits = Mock()
        limits.service_is_ready.return_value = True
        limits.call_async.side_effect = lambda request: (events.append("limits"),
                                                           Future(Mock(success=True, message="ok")))[1]
        parameters = Mock()
        parameters.parameters_are_ready.return_value = True
        parameters.get_parameters.side_effect = lambda names: (events.append(tuple(names)),
                                                                 Future(Mock(values=config_values())))[1]
        trigger = type("Trigger", (), {"Request": type("Request", (), {})})
        client = SnapshotConfigurationClient(limits, parameters, trigger,
                                             lambda config, error: results.append((config, error)),
                                             clock=lambda: now[0])
        client.refresh()
        client.poll()
        client.poll()
        client.poll()
        self.assertEqual(events[0], "limits")
        self.assertEqual(events[1], ("servo_ids", "zero_positions", "directions",
                                     "target_tolerance", "lower_limits", "upper_limits"))
        self.assertEqual(results[0][0].joints, ("servo_1", "servo_2"))
        self.assertIsNone(results[0][1])

    def test_configuration_timeout_retries_and_stale_or_closed_response_is_ignored(self):
        now = [0.0]
        results = []
        pending = Future()
        limits = Mock()
        limits.service_is_ready.return_value = True
        limits.call_async.return_value = pending
        params = Mock()
        params.parameters_are_ready.return_value = True
        trigger = type("Trigger", (), {"Request": type("Request", (), {})})
        client = SnapshotConfigurationClient(limits, params, trigger,
                                             lambda config, error: results.append((config, error)),
                                             timeout=2, retry_delay=3, clock=lambda: now[0])
        client.refresh()
        client.poll()
        now[0] = 2
        client.poll()
        self.assertIsNone(results[-1][0])
        self.assertIn("timed out", results[-1][1])
        pending.value = Mock(success=True, message="ok")
        # The old response cannot reach the parameter stage or commit a config.
        client.poll()
        self.assertEqual(len(results), 1)
        client.close()
        self.assertTrue(client.closed)

    def test_failed_limit_refresh_retries_after_backoff(self):
        now = [0.0]
        results = []
        limits = Mock()
        limits.service_is_ready.return_value = True
        limits.call_async.side_effect = [Future(Mock(success=False, message="no servo")),
                                         Future(Mock(success=True, message="ok"))]
        params = Mock()
        params.parameters_are_ready.return_value = True
        params.get_parameters.return_value = Future(Mock(values=config_values()))
        trigger = type("Trigger", (), {"Request": type("Request", (), {})})
        client = SnapshotConfigurationClient(limits, params, trigger,
                                             lambda config, error: results.append((config, error)),
                                             retry_delay=3, clock=lambda: now[0])
        client.refresh()
        client.poll()
        client.poll()
        self.assertIn("no servo", results[-1][1])
        self.assertEqual(params.get_parameters.call_count, 0)
        now[0] = 3
        client.poll()
        client.poll()
        client.poll()
        self.assertIsNotNone(results[-1][0])

    def test_refresh_replaces_and_cancels_an_inflight_identity(self):
        old, current = Future(), Future(Mock(success=True, message="fresh"))
        limits, params, results = Mock(), Mock(), []
        limits.service_is_ready.return_value = True
        limits.call_async.side_effect = (old, current)
        params.parameters_are_ready.return_value = True
        params.get_parameters.return_value = Future(Mock(values=config_values()))
        trigger = type("Trigger", (), {"Request": type("Request", (), {})})
        client = SnapshotConfigurationClient(limits, params, trigger,
                                             lambda config, error: results.append((config, error)))
        client.refresh()
        client.poll()
        client.refresh()
        self.assertTrue(old.was_cancelled)
        old.value = Mock(success=True, message="late old response")
        client.poll()
        client.poll()
        client.poll()
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0][0].joints, ("servo_1", "servo_2"))

    def test_close_cancels_configuration_request_and_ignores_late_reply(self):
        future, results = Future(), []
        limits, params = Mock(), Mock()
        limits.service_is_ready.return_value = True
        limits.call_async.return_value = future
        trigger = type("Trigger", (), {"Request": type("Request", (), {})})
        client = SnapshotConfigurationClient(limits, params, trigger,
                                             lambda config, error: results.append(config))
        client.refresh()
        client.poll()
        client.close()
        self.assertTrue(future.was_cancelled)
        future.value = Mock(success=True, message="late")
        client.poll()
        self.assertEqual(results, [])

    def test_parameter_deadline_cancels_and_retries_from_fresh_limits(self):
        now, results = [0.0], []
        parameters_future = Future()
        limits = Mock()
        limits.service_is_ready.return_value = True
        limits.call_async.return_value = Future(Mock(success=True, message="ok"))
        params = Mock()
        params.services_are_ready.return_value = True
        params.get_parameters.return_value = parameters_future
        trigger = type("Trigger", (), {"Request": type("Request", (), {})})
        client = SnapshotConfigurationClient(limits, params, trigger,
                                             lambda config, error: results.append((config, error)),
                                             timeout=2, retry_delay=3, clock=lambda: now[0])
        client.refresh()
        client.poll()
        client.poll()
        now[0] = 2
        client.poll()
        self.assertTrue(parameters_future.was_cancelled)
        self.assertIn("parameters request timed out", results[-1][1])
        parameters_future.value = Mock(values=config_values())
        client.poll()
        self.assertEqual(len(results), 1)

    def test_arrival_requires_fresh_complete_ordered_samples_and_one_second(self):
        targets = {"servo_1": 0.0}
        coordinator = PlaybackCoordinator(tuple(targets), targets, 0.01,
                                          started_at=1.0, sent_stamp=100)
        self.assertEqual(coordinator.observe({}, received_at=1.1, stamp=101,
                                             controller_now=200_000_000), "waiting")
        self.assertEqual(coordinator.observe(targets, received_at=1.1, stamp=100,
                                             controller_now=200_000_000), "waiting")
        self.assertEqual(coordinator.observe(targets, received_at=1.1, stamp=101,
                                             controller_now=200_000_000), "waiting")
        self.assertEqual(coordinator.observe(targets, received_at=1.2, stamp=101,
                                             controller_now=200_000_000), "waiting")
        self.assertEqual(coordinator.observe(targets, received_at=1.6, stamp=600_000_101,
                                             controller_now=700_000_000), "waiting")
        self.assertEqual(coordinator.observe(targets, received_at=2.2, stamp=1_100_000_101,
                                             controller_now=1_200_000_000), "waiting")
        self.assertEqual(coordinator.observe(targets, received_at=2.7, stamp=1_700_000_101,
                                             controller_now=1_800_000_000), "arrived")

    def test_stall_timeout_publication_failure_and_release_after_stability(self):
        c = PlaybackCoordinator(("j",), {"j": 1.0}, 0.0, started_at=0,
                                 sent_stamp=1, arrival_timeout=3)
        self.assertEqual(c.observe({"j": 0.0}, received_at=0.1, stamp=2,
                                   controller_now=10_000_000), "waiting")
        self.assertEqual(c.tick(3), "failed")
        c.publication_failed("offline")
        self.assertIn("offline", c.detail)
        stalled = PlaybackCoordinator(("j",), {"j": 0.0}, 0.0,
                                      started_at=0, sent_stamp=1)
        self.assertEqual(stalled.observe({"j": 1.0}, received_at=0.1, stamp=2,
                                         controller_now=10_000_000), "waiting")
        self.assertEqual(stalled.observe({"j": 1.0}, received_at=8.2,
                                         stamp=8_100_000_002,
                                         controller_now=8_200_000_000), "failed")
        self.assertIn("No progress", stalled.detail)
        c = PlaybackCoordinator(("j",), {"j": 0.0}, 0.0, started_at=0,
                                sent_stamp=1, release_after=True)
        for sec in (0.1, 0.6, 1.2):
            state = c.observe({"j": 0.0}, received_at=sec,
                              stamp=int((sec + 1) * 1e9),
                              controller_now=int((sec + 1.1) * 1e9))
        self.assertEqual(state, "release")


class TorqueAdapterTests(unittest.TestCase):
    class Request:
        def __init__(self):
            self.data = None

    def make_client(self, future, ready=True):
        client = Mock()
        client.service_is_ready.return_value = ready
        client.call_async.return_value = future
        return client

    def test_partial_batch_reports_outcomes_and_ignores_late_callbacks(self):
        now = [0.0]
        slow = Future()
        clients = {"a": self.make_client(Future(Mock(success=True, message=""))),
                   "b": self.make_client(slow), "c": self.make_client(Future(), ready=False)}
        adapter = TorqueBatch(clients, self.Request, clock=lambda: now[0])
        outcomes = adapter.start(False)
        self.assertEqual(outcomes["a"], "success")
        self.assertEqual(outcomes["c"], "not_sent: service unavailable")
        now[0] = 9
        self.assertEqual(adapter.poll()["b"], "timeout: torque state unknown")
        slow.value = Mock(success=True, message="")
        slow.callback(slow)
        self.assertEqual(adapter.outcomes["b"], "timeout: torque state unknown")

    def test_failed_response_and_close_are_explicit(self):
        failure = Future(Mock(success=False, message="bus error"))
        recovered = Future(Mock(success=True, message=""))
        client = self.make_client(failure)
        client.call_async.side_effect = (failure, recovered)
        adapter = TorqueBatch({"j": client}, self.Request)
        self.assertEqual(adapter.start(True)["j"], "failure: bus error")
        self.assertEqual(adapter.start(True)["j"], "success")
        pending = Future()
        adapter = TorqueBatch({"j": self.make_client(pending)}, self.Request)
        adapter.start(False)
        adapter.close()
        self.assertTrue(pending.was_cancelled)
        pending.value = Mock(success=True, message="")
        pending.callback(pending)
        self.assertIn("closed", adapter.outcomes["j"])


if __name__ == "__main__":
    unittest.main()
