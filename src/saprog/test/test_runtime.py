import builtins
import os
import unittest
from unittest import mock

from home import Home
from saprog_desktop import runtime
from static_animator import AnimatorViews


class FakeRclpy:
    def __init__(self):
        self.events = []
        self.active = True

    def ok(self):
        return self.active

    def spin_once(self, node, timeout_sec):
        self.events.append(("spin", node.name, timeout_sec))

    def shutdown(self):
        self.events.append(("shutdown",))
        self.active = False


class FakeRoot:
    def __init__(self):
        self.callbacks = {}
        self.cancelled = []
        self.events = []
        self.next_id = 0

    def after(self, _delay_ms, callback=None):
        self.next_id += 1
        after_id = f"after-{self.next_id}"
        if callback is not None:
            self.callbacks[after_id] = callback
        return after_id

    def after_cancel(self, after_id):
        self.cancelled.append(after_id)
        self.callbacks.pop(after_id, None)

    def run_next(self):
        after_id = next(iter(self.callbacks))
        callback = self.callbacks.pop(after_id)
        callback()

    def destroy(self):
        self.events.append(("destroy-root",))


class FakeNode:
    def __init__(self, name, events):
        self.name = name
        self.events = events

    def destroy_node(self):
        self.events.append(("destroy-node", self.name))


class CloseProbe:
    def __init__(self, name, events):
        self.name = name
        self.events = events
        self.close_callback = None
        self.close_requests = 0

    def close(self):
        self.close_requests += 1
        self.events.append(("request-close", self.name))


class ProcessProbe:
    def __init__(self):
        self.running = True
        self.terminated = False

    def poll(self):
        return None if self.running else 0

    def terminate(self):
        self.terminated = True
        self.running = False

    def wait(self, timeout=None):
        return 0


class CloseServiceProbe:
    def __init__(self):
        self.closed = False

    def close(self):
        self.closed = True


class RuntimeTests(unittest.TestCase):
    def test_creator_widgets_use_injected_tk_runtime(self):
        import snapshot_window

        creator = snapshot_window.SnapshotWindow.__new__(snapshot_window.SnapshotWindow)
        creator.root = object()
        creator.container = mock.Mock()
        creator.dependencies = mock.Mock()
        creator.show_snapshot_loader = False
        creator.moving_speed = mock.Mock()
        creator.torque_strength = mock.Mock()
        creator.torque_strength_changed = None
        creator.snapshot_name = mock.Mock()
        creator.selected = mock.Mock()
        creator.hold_after_load = mock.Mock()
        creator.status = mock.Mock()

        with (
            mock.patch.object(snapshot_window, "apply_style"),
            mock.patch.object(snapshot_window, "add_slider"),
            mock.patch.object(snapshot_window, "status_box") as status_box,
        ):
            creator._build()

        self.assertFalse(hasattr(snapshot_window, "tk"))
        self.assertFalse(hasattr(snapshot_window, "ttk"))
        creator.dependencies.ttk.Frame.assert_called()
        creator.dependencies.tk.Canvas.assert_called_once()
        status_box.assert_called_once_with(
            mock.ANY,
            creator.status,
            height=3,
            width=64,
            tk_module=creator.dependencies.tk,
        )

    def test_animator_partial_initialization_closes_constructed_views(self):
        import snapshot_window
        import static_animator
        import timeline_window

        class FakeWidget:
            instances = []

            def __init__(self, *_args, **_kwargs):
                self.__class__.instances.append(self)
                self.bindings = {}
                self.added = []
                self.sash_positions = []

            def pack(self, **_kwargs):
                pass

            def add(self, child, **kwargs):
                self.added.append((child, kwargs))

            def bind(self, sequence, callback, add=None):
                self.bindings[sequence] = callback
                return sequence

            def unbind(self, sequence, _binding_id=None):
                self.bindings.pop(sequence, None)

            def sashpos(self, index, position):
                self.sash_positions.append((index, position))

        creator, player = mock.Mock(), mock.Mock()
        dependencies = mock.Mock()
        dependencies.ttk.Notebook = FakeWidget
        dependencies.ttk.Frame = FakeWidget
        dependencies.ttk.Panedwindow = FakeWidget
        with (
            mock.patch.object(
                snapshot_window, "SnapshotWindow", return_value=creator
            ) as creator_factory,
            mock.patch.object(
                static_animator.snapshot_player, "SnapshotPlayer", return_value=player
            ) as player_factory,
            mock.patch.object(
                static_animator.snapshot_player, "load_torque_strength", return_value=512
            ),
            mock.patch.object(
                static_animator.snapshot_player, "player_settings_path", return_value="settings"
            ),
            mock.patch.object(
                timeline_window,
                "TimelineWindow",
                side_effect=RuntimeError("timeline construction failed"),
            ) as timeline_factory,
        ):
            with self.assertRaisesRegex(RuntimeError, "timeline construction failed"):
                static_animator.build_animator_views(
                    object(),
                    object(),
                    object(),
                    object(),
                    object(),
                    ".",
                    dependencies=dependencies,
                )

        split = next(
            widget
            for widget in FakeWidget.instances
            if len(widget.added) == 2 and all("weight" in options for _, options in widget.added)
        )
        self.assertEqual([pane[1]["weight"] for pane in split.added], [1, 1])
        split.bindings["<Configure>"](mock.Mock(width=1200))
        self.assertEqual(split.sash_positions, [(0, 600)])
        self.assertNotIn("<Configure>", split.bindings)
        creator.close.assert_called_once_with()
        player.close.assert_called_once_with()
        creator_factory.assert_called_once()
        self.assertIs(creator_factory.call_args.kwargs["dependencies"], dependencies)
        self.assertFalse(creator_factory.call_args.kwargs["show_snapshot_loader"])
        player_factory.assert_called_once()
        player_args = player_factory.call_args.kwargs
        self.assertIs(player_args["dependencies"], dependencies)
        self.assertIs(player_args["on_pose_selected"], creator.load_targets_from_file)
        timeline_factory.assert_called_once()
        self.assertIs(timeline_factory.call_args.kwargs["dependencies"], dependencies)

    def test_animator_close_waits_for_timeline_outcome_and_disposes_once(self):
        events = []
        creator = CloseProbe("creator", events)
        player = CloseProbe("player", events)
        timeline = CloseProbe("timeline", events)
        host_close = mock.Mock(side_effect=lambda: events.append(("host-close",)))
        views = AnimatorViews(creator, player, timeline, object(), object(), host_close)
        timeline.close_callback = views.dispose

        views.request_close()
        self.assertEqual(events, [("request-close", "timeline")])
        self.assertFalse(views._disposed)
        host_close.assert_not_called()

        # Timeline invokes this only after dirty edits and any active action
        # have reached a confirmed safe close outcome.
        timeline.close_callback()
        views.request_close()
        views.dispose()

        self.assertEqual(
            events,
            [
                ("request-close", "timeline"),
                ("request-close", "creator"),
                ("request-close", "player"),
                ("host-close",),
            ],
        )
        self.assertEqual(creator.close_requests, 1)
        self.assertEqual(player.close_requests, 1)
        self.assertEqual(timeline.close_requests, 1)
        host_close.assert_called_once_with()

    def test_home_preserves_backend_until_timeline_confirms_close(self):
        home = Home.__new__(Home)
        home.closed = False
        home.children = {}
        backend = ProcessProbe()
        home.backend = backend
        home.imu_backend = None
        home.imu_monitor = CloseServiceProbe()
        home.robot_view = CloseServiceProbe()
        home.scheduler = mock.Mock()
        events = []
        creator, player = CloseProbe("creator", events), CloseProbe("player", events)
        timeline = CloseProbe("timeline", events)
        views = AnimatorViews(creator, player, timeline, object(), object(), home._finish_close)
        timeline.close_callback = views.dispose
        home.animator_views = views

        home.close()

        self.assertTrue(home.backend.running)
        self.assertFalse(home.closed)
        home.scheduler.request_close.assert_not_called()
        self.assertFalse(home.imu_monitor.closed)

        # A terminal result (or a safe no-action close) reaches Timeline's
        # callback and only then disposes Home's backend/root resources.
        timeline.close_callback()

        self.assertFalse(backend.running)
        self.assertTrue(backend.terminated)
        self.assertIsNone(home.backend)
        self.assertTrue(home.closed)
        self.assertTrue(home.imu_monitor.closed)
        home.scheduler.request_close.assert_called_once_with()

    def test_scheduler_bounds_and_fairly_rotates_registered_nodes(self):
        root = FakeRoot()
        rclpy = FakeRclpy()
        nodes = [FakeNode(name, []) for name in ("a", "b", "c")]
        scheduler = runtime.RootRosScheduler(
            root, mock.Mock(rclpy=rclpy), nodes[:1], interval_ms=7, spin_budget=2
        )
        scheduler.add_node(nodes[1])
        scheduler.add_node(nodes[2])

        scheduler.start()
        scheduler.start()
        self.assertEqual(len(root.callbacks), 1)
        root.run_next()
        self.assertEqual([event[1] for event in rclpy.events], ["a", "b"])
        self.assertEqual(len(root.callbacks), 1)
        root.run_next()
        self.assertEqual([event[1] for event in rclpy.events], ["a", "b", "c", "a"])

        scheduler.stop_callbacks()
        scheduler.start()
        self.assertEqual(root.callbacks, {})

    def test_shutdown_cancels_pending_view_callbacks_before_node_and_ros_teardown(self):
        root = FakeRoot()
        rclpy = FakeRclpy()
        events = rclpy.events
        node = FakeNode("shared", events)
        scheduler = runtime.RootRosScheduler(root, mock.Mock(rclpy=rclpy), [node])
        view_callback = root.after(500, lambda: events.append(("view-callback",)))
        scheduler.start()

        scheduler.shutdown()

        self.assertCountEqual(root.cancelled, [view_callback, "after-2"])
        self.assertEqual(
            events,
            [("destroy-node", "shared"), ("shutdown",)],
        )
        self.assertEqual(root.callbacks, {})

    def test_request_close_cancels_callbacks_before_destroying_root(self):
        root = FakeRoot()
        rclpy = FakeRclpy()
        scheduler = runtime.RootRosScheduler(root, mock.Mock(rclpy=rclpy), [])
        pending = root.after(20, lambda: None)

        scheduler.request_close()

        self.assertEqual(root.cancelled, [pending])
        self.assertEqual(root.events, [("destroy-root",)])

    def test_setup_fallback_preserves_original_arguments_and_entrypoint(self):
        marker = "SAPROG_SNAPSHOT_PLAYER_BOOTSTRAPPED"
        args = ["--directory", "my snapshots", "--ros-args", "-r", "x:=y"]
        real_import = builtins.__import__

        def import_without_tk(name, *import_args, **import_kwargs):
            if name == "geometry_msgs.msg":
                raise ImportError("missing ROS dependency for fallback test")
            return real_import(name, *import_args, **import_kwargs)

        with mock.patch.dict(os.environ, {"ROS_DISTRO": "jazzy"}, clear=False) as env:
            env.pop(marker, None)
            with (
                mock.patch("builtins.__import__", side_effect=import_without_tk),
                mock.patch.object(runtime.platform, "system", return_value="Linux"),
                mock.patch.object(runtime.os.path, "isfile", return_value=True),
                mock.patch.object(runtime.os, "access", return_value=True),
                mock.patch.object(
                    runtime.os,
                    "execvpe",
                    side_effect=RuntimeError("captured re-exec"),
                ) as execvpe,
            ):
                with self.assertRaisesRegex(RuntimeError, "captured re-exec"):
                    runtime.load_desktop_dependencies(
                        args, entrypoint="/workspace/scripts/snapshot_player.py"
                    )

        command = execvpe.call_args.args[1]
        self.assertEqual(command[0], "bash")
        self.assertIn("/workspace/scripts/snapshot_player.py", command)
        self.assertEqual(command[-len(args) :], args)
        self.assertEqual(execvpe.call_args.args[2][marker], "1")

    def test_missing_dependency_error_keeps_setup_guidance(self):
        real_import = builtins.__import__

        def import_without_tk(name, *import_args, **import_kwargs):
            if name == "geometry_msgs.msg":
                raise ImportError("missing ROS dependency for diagnostics test")
            return real_import(name, *import_args, **import_kwargs)

        with (
            mock.patch("builtins.__import__", side_effect=import_without_tk),
            mock.patch.object(runtime.platform, "system", return_value="Windows"),
        ):
            with self.assertRaises(SystemExit) as raised:
                runtime.load_desktop_dependencies(
                    [], require_timeline=True, entrypoint="snapshot_player.py"
                )

        message = str(raised.exception)
        self.assertIn("ROS 2 Python packages and Tk", message)
        self.assertIn("saprog action interface", message)
        self.assertIn("missing ROS dependency for diagnostics test", message)


if __name__ == "__main__":
    unittest.main()
