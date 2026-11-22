"""Bridge lifecycle and routing regressions without ROS, WSL, or hardware."""

import io
import json
import subprocess
import sys
import tempfile
import threading
import unittest
from concurrent.futures import Future
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

BRIDGE = Path(__file__).resolve().parents[1] / "ros_manager" / "wsl_bridge"
sys.path.insert(0, str(BRIDGE.parent))

from wsl_bridge import commands, processes, protocol, runtime, subscriptions  # noqa: E402


class BridgeEntryTests(unittest.TestCase):
    def test_package_directory_runs_without_desktop_or_ros_imports(self):
        with tempfile.TemporaryDirectory(prefix="bridge launch ") as directory:
            result = subprocess.run(
                [sys.executable, "-I", "-u", str(BRIDGE), "--help"],
                cwd=directory, capture_output=True, text=True, timeout=10,
            )
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("--workspace", result.stdout)

    def test_reader_preserves_lines_and_signals_eof(self):
        shutdown = threading.Event()
        reader = protocol.CommandReader(shutdown)
        with patch.object(protocol.sys, "stdin", io.StringIO('{"op":"release"}\n')):
            reader.start()
            reader.thread.join(timeout=2)
        self.assertFalse(reader.thread.is_alive())
        self.assertTrue(shutdown.is_set())
        self.assertEqual(json.loads(reader.queue.get_nowait()), {"op": "release"})
        self.assertIsNone(reader.queue.get_nowait())


class RobotCommandsTests(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        service_type = SimpleNamespace(Request=SimpleNamespace)
        self.commands = commands.RobotCommands(self.node, Mock(), service_type, service_type)

    def test_release_all_cancels_queued_commands_before_service_request(self):
        self.commands.handle({"op": "publish_position", "id": "pose"})
        self.commands.handle({"op": "set_torque", "id": "torque"})
        with patch.object(commands, "emit") as emit:
            self.commands.handle({"op": "release_all_torque", "id": "release"})
        self.assertEqual([cmd["id"] for cmd in self.commands.pending_commands], ["release"])
        self.assertEqual([call.kwargs["id"] for call in emit.call_args_list], ["pose", "torque"])
        self.assertTrue(all(not call.kwargs["success"] for call in emit.call_args_list))
        self.node.create_client.assert_not_called()

    def test_services_wait_for_readiness_then_report_async_results(self):
        for op, service in (
            ("refresh_servo_limits", "/saprog/read_servo_limits"),
            ("release_all_torque", "/saprog/release_all_torque"),
            ("set_torque", "/saprog/servo_11/torque"),
        ):
            with self.subTest(op=op):
                client = Mock()
                client.service_is_ready.return_value = False
                self.node.create_client.return_value = client
                future = Future()
                client.call_async.return_value = future
                with patch.object(commands, "emit") as emit, patch.object(
                    commands.time, "monotonic", return_value=100,
                ):
                    self.commands.handle({
                        "op": op, "id": op, "joint": "servo_11", "enabled": True,
                    })
                    self.commands.process_pending()
                    self.assertEqual(len(self.commands.pending_commands), 1)
                    client.call_async.assert_not_called()
                    client.service_is_ready.return_value = True
                    self.commands.process_pending()
                    self.assertEqual(self.commands.pending_commands, [])
                    emit.assert_not_called()
                    future.set_result(SimpleNamespace(success=True, message="done"))
                    emit.assert_called_once_with(
                        "command_result", id=op, success=True, message="done",
                    )
                self.assertEqual(self.node.create_client.call_args.args[1], service)
                if op == "set_torque":
                    self.assertTrue(client.call_async.call_args.args[0].data)

    def test_unavailable_service_times_out_without_sending(self):
        client = self.node.create_client.return_value
        client.service_is_ready.return_value = False
        with patch.object(commands.time, "monotonic", return_value=100):
            self.commands.handle({"op": "refresh_servo_limits", "id": "limits"})
        with patch.object(commands.time, "monotonic", return_value=131), patch.object(
            commands, "emit",
        ) as emit:
            self.commands.process_pending()
        self.assertEqual(self.commands.pending_commands, [])
        client.call_async.assert_not_called()
        self.assertFalse(emit.call_args.kwargs["success"])


class TopicTests(unittest.TestCase):
    def setUp(self):
        self.node = Mock()
        self.topics = subscriptions.TopicSubscriptions(self.node)
        self.qos = Mock()
        self.sensor_qos = object()
        self.get_message = Mock(return_value=object())
        ros_modules = patch.dict(sys.modules, {
            "rclpy.qos": SimpleNamespace(
                QoSProfile=self.qos, qos_profile_sensor_data=self.sensor_qos,
            ),
            "rosidl_runtime_py.convert": SimpleNamespace(message_to_ordereddict=lambda msg: msg),
            "rosidl_runtime_py.utilities": SimpleNamespace(get_message=self.get_message),
        })
        ros_modules.start()
        self.addCleanup(ros_modules.stop)
        self.request = {
            "op": "subscribe", "token": "a", "topic": "/temperature",
            "message_type": "sensor_msgs/msg/Temperature",
            "package": "saprog_dynamixel", "executable": "readback",
        }

    def test_subscription_is_shared_until_last_owner_releases_it(self):
        with patch.object(subscriptions, "emit"):
            self.topics.handle(self.request)
            self.topics.handle(dict(self.request, token="b"))
        self.node.create_subscription.assert_called_once()
        self.qos.assert_called_once_with(depth=100)
        self.topics.handle({"op": "release", "token": "a"})
        self.assertEqual(len(self.topics.active_requests()), 1)
        self.node.destroy_subscription.assert_not_called()
        self.topics.handle({"op": "release", "token": "b"})
        self.assertEqual(self.topics.active_requests(), [])
        self.node.destroy_subscription.assert_called_once_with(
            self.node.create_subscription.return_value,
        )
        self.assertEqual(self.topics.last_status, {})

    def test_conflicting_provider_is_rejected_and_messages_are_forwarded(self):
        with patch.object(subscriptions, "emit") as emit:
            self.topics.handle(self.request)
            self.topics.handle(dict(self.request, token="b", package="other"))
            self.assertNotIn("b", self.topics.requests)
            self.assertIn("conflicting", emit.call_args.kwargs["status"])
            receive = self.node.create_subscription.call_args.args[2]
            with patch.object(subscriptions.time, "monotonic", return_value=100):
                receive({"temperature": 42})
                self.assertEqual(self.topics.topic_status("/temperature", 1), "Connected")
            emit.assert_called_with(
                "message", topic="/temperature", data={"temperature": 42},
                received_at_ns=self.node.get_clock().now().nanoseconds,
            )
            with patch.object(subscriptions.time, "monotonic", return_value=103):
                self.assertEqual(self.topics.topic_status("/temperature", 1), "Stale IMU data")


class NodeProcessesTests(unittest.TestCase):
    def test_relaunch_reuses_successful_build_and_reports_process_exit(self):
        manager = processes.NodeProcesses("/workspace", threading.Event(), Mock())
        request = {"topic": "/topic", "package": "pkg", "executable": "node"}
        key = ("pkg", "node")
        node = Mock()
        node.count_publishers.return_value = 0
        first, second = Mock(), Mock()
        first.poll.return_value = 1
        second.poll.return_value = None
        with patch.object(processes, "ensure_executable") as build, patch.object(
            processes, "launch_node", side_effect=[first, second],
        ) as launch, patch.object(processes, "emit"), patch.object(
            processes.time, "monotonic", return_value=100,
        ) as clock:
            manager._build_and_launch(key, "/topic", request)
            manager.collect_results()
            manager.start_time = 0
            with patch.object(processes.threading, "Thread") as thread:
                manager.ensure_provider(request, node, Mock())
                thread.assert_not_called()
                clock.return_value = 105
                manager.ensure_provider(request, node, Mock())
                thread.assert_called_once()
            manager._build_and_launch(key, "/topic", request)
            manager.collect_results()
        build.assert_called_once()
        self.assertEqual(launch.call_count, 2)
        self.assertTrue(any("exited with code 1" in call.args[1]
                            for call in manager.set_status.call_args_list))

    def test_failure_backoff_is_shared_by_topics_and_capped(self):
        manager = processes.NodeProcesses("/workspace", threading.Event(), Mock())
        manager.start_time = 0
        key = ("pkg", "node")
        node = Mock()
        node.count_publishers.return_value = 0
        request = {"topic": "/other", "package": "pkg", "executable": "node"}
        with patch.object(processes.time, "monotonic", return_value=100), patch.object(
            processes.threading, "Thread",
        ) as thread:
            for delay in (5, 10, 20, 40, 60, 60):
                manager._record_failure(key, "/topic", "USB unavailable")
                self.assertEqual(manager.retry_at[key], 100 + delay)
                manager.ensure_provider(request, node, Mock())
                self.assertEqual(manager.set_status.call_args.args, (
                    "/other", f"USB unavailable; retrying in {delay} seconds",
                ))
            thread.assert_not_called()

    def test_output_is_drained_and_exit_status_includes_failure_reason(self):
        process = subprocess.Popen(
            [sys.executable, "-c",
             "import sys; print('output\\n' * 40000); "
             "print('USB service is stopped', file=sys.stderr); sys.exit(1)"],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
        )
        self.addCleanup(lambda: process.kill() if process.poll() is None else None)
        process.saprog_output = processes.NodeOutput(process.stdout)
        self.assertEqual(process.wait(timeout=10), 1)
        manager = processes.NodeProcesses("/workspace", threading.Event(), Mock())
        manager.start_time = 0
        manager.processes[("pkg", "node")] = process
        node = Mock()
        node.count_publishers.return_value = 0
        with patch.object(processes.time, "monotonic", return_value=100):
            manager.ensure_provider(
                {"topic": "/topic", "package": "pkg", "executable": "node"}, node, Mock(),
            )
        status = manager.set_status.call_args.args[1]
        self.assertIn("USB service is stopped", status)
        self.assertIn("retrying in 5 seconds", status)
        self.assertLessEqual(len(process.saprog_output.lines), 20)

    def test_build_errors_wait_before_retrying(self):
        manager = processes.NodeProcesses("/workspace", threading.Event(), Mock())
        manager.start_time = 0
        key = ("pkg", "node")
        manager.build_results.put((key, "/topic", None, RuntimeError("build failed")))
        node = Mock()
        node.count_publishers.return_value = 0
        with patch.object(processes.time, "monotonic", return_value=100), patch.object(
            processes.threading, "Thread",
        ) as thread:
            manager.collect_results()
            manager.ensure_provider(
                {"topic": "/topic", "package": "pkg", "executable": "node"}, node, Mock(),
            )
            thread.assert_not_called()
        self.assertIn("build failed", manager.set_status.call_args.args[1])

    def test_backoff_resets_after_running_not_after_a_long_build(self):
        manager = processes.NodeProcesses("/workspace", threading.Event(), Mock())
        manager.start_time = 0
        key = ("pkg", "node")
        manager.failures[key] = 5
        manager.last_attempt[key] = 1
        process = Mock()
        process.poll.return_value = None
        manager.build_results.put((key, "/topic", process, None))
        node = Mock()
        node.count_publishers.return_value = 0
        request = {"topic": "/topic", "package": "pkg", "executable": "node"}
        with patch.object(processes.time, "monotonic", return_value=100) as clock:
            manager.collect_results()
            manager.ensure_provider(request, node, Mock())
            self.assertEqual(manager.failures[key], 5)
            clock.return_value = 130
            manager.ensure_provider(request, node, Mock())
            self.assertNotIn(key, manager.failures)
            process.poll.return_value = 1
            manager.ensure_provider(request, node, Mock())
            self.assertIn("retrying in 5 seconds", manager.set_status.call_args.args[1])

    def test_shutdown_joins_builds_and_stops_late_and_registered_processes(self):
        shutdown = threading.Event()
        manager = processes.NodeProcesses("/workspace", shutdown, Mock())
        process, late_process = Mock(), Mock()
        manager.processes[("pkg", "node")] = process
        build = Mock()
        build.join.side_effect = lambda: manager.build_results.put(
            (("other", "node"), "/topic", late_process, None)
        )
        manager.build_jobs[("other", "node")] = build
        with patch.object(processes, "stop_process") as stop, patch.object(
            processes, "stop_processes",
        ) as stop_all:
            manager.close()
        self.assertTrue(shutdown.is_set())
        build.join.assert_called_once()
        stop.assert_called_once_with(late_process)
        stop_all.assert_called_once_with({("pkg", "node"): process})

    def test_existing_publisher_does_not_trigger_build(self):
        manager = processes.NodeProcesses("/workspace", threading.Event(), Mock())
        manager.start_time = 0
        node = Mock()
        node.count_publishers.return_value = 1
        status = Mock(return_value="Connected")
        with patch.object(processes.time, "monotonic", return_value=100), patch.object(
            processes.threading, "Thread",
        ) as thread:
            manager.ensure_provider({"topic": "/topic", "package": "pkg", "executable": "node"},
                                    node, status)
        thread.assert_not_called()
        manager.set_status.assert_called_once_with("/topic", "Connected")


class RuntimeTests(unittest.TestCase):
    def test_telemetry_bursts_are_drained_without_waiting_per_callback(self):
        executor = Mock()
        callbacks = list(range(60))
        processed = []

        def spin_once(*, timeout_sec):
            self.assertEqual(timeout_sec, 0)
            if callbacks:
                processed.append(callbacks.pop())

        executor.spin_once.side_effect = spin_once
        runtime.spin_ready_callbacks(executor, clock=lambda: 100)
        self.assertEqual(len(processed), 60)
        # Sustained traffic is bounded so input/cancel processing gets a turn.
        self.assertEqual(executor.spin_once.call_count, 100)

    def test_slow_callbacks_yield_to_command_processing_after_time_budget(self):
        executor = Mock()
        runtime.spin_ready_callbacks(executor, clock=iter((100, 100.003, 100.006)).__next__)
        self.assertEqual(executor.spin_once.call_count, 2)

    def test_dispatch_routes_topics_and_commands_and_reports_invalid_input(self):
        robot, topics = Mock(), Mock()
        robot.handle.return_value = True
        runtime.dispatch_command('{"op":"set_torque"}', robot, topics)
        topics.handle.assert_not_called()
        robot.handle.return_value = False
        runtime.dispatch_command('{"op":"release","token":"a"}', robot, topics)
        topics.handle.assert_called_once_with({"op": "release", "token": "a"})
        with patch.object(runtime, "emit") as emit:
            runtime.dispatch_command("bad json", robot, topics)
        self.assertIn("Invalid manager request", emit.call_args.kwargs["status"])

    def test_eof_closes_processes_and_ros_node(self):
        ros = Mock()
        ros.ok.return_value = True
        with patch.dict(sys.modules, {
            "rclpy": ros,
            "rclpy.executors": SimpleNamespace(SingleThreadedExecutor=Mock()),
            "sensor_msgs.msg": SimpleNamespace(JointState=Mock()),
            "std_msgs.msg": SimpleNamespace(String=Mock()),
            "std_srvs.srv": SimpleNamespace(SetBool=Mock(), Trigger=Mock()),
        }), patch.object(runtime, "CommandReader") as reader, patch.object(
            runtime, "NodeProcesses",
        ) as nodes:
            reader.return_value.queue.get.return_value = None
            runtime.run("/workspace")
        nodes.return_value.close.assert_called_once()
        ros.create_node.return_value.destroy_node.assert_called_once()
        ros.shutdown.assert_called_once()


if __name__ == "__main__":
    unittest.main()
