"""Standalone ROS/Tk entry points; views remain independent of process startup."""

import argparse
import os
import signal
import sys

from .runtime import RootRosScheduler, load_desktop_dependencies
from .snapshot_files import default_snapshot_directory


def run_ros_desktop(argv, entrypoint, parser, node_names, build_app, require_timeline=False):
    """Run one canonical ROS/Tk bootstrap and bounded root teardown."""
    original_args = sys.argv[1:] if argv is None else list(argv)
    args, ros_args = parser.parse_known_args(original_args)
    timeline_required = (
        require_timeline(args) if callable(require_timeline) else require_timeline
    )
    dependencies = load_desktop_dependencies(
        original_args,
        require_timeline=timeline_required,
        entrypoint=entrypoint,
    )
    dependencies.rclpy.init(args=ros_args)
    root = dependencies.tk.Tk()
    nodes = [
        dependencies.Node(name)
        for name in node_names
    ]
    scheduler = RootRosScheduler(root, dependencies, nodes)
    try:
        close_app = build_app(args, root, dependencies, nodes, scheduler)
        if close_app is None:
            close_app = scheduler.request_close
        root.protocol("WM_DELETE_WINDOW", close_app)

        def signal_close(_signum, _frame):
            root.after(0, close_app)

        signal.signal(signal.SIGTERM, signal_close)
        signal.signal(signal.SIGINT, signal_close)
        scheduler.start()
        root.mainloop()
    finally:
        scheduler.shutdown()
        try:
            root.destroy()
        except Exception:
            pass
    return 0


def snapshot_player_main(argv=None, entrypoint=None):
    """Start the snapshot player or timeline as a standalone root window."""
    parser = argparse.ArgumentParser(
        description=(
            "Open the saprog snapshot player. A separately started saprog "
            "controller is required to send commands."
        )
    )
    parser.add_argument("--directory", default=default_snapshot_directory())
    parser.add_argument("--timeline", action="store_true", help="Open the timeline editor")

    def build_app(args, root, dependencies, nodes, scheduler):
        directory = os.path.abspath(os.path.expanduser(args.directory))
        if args.timeline:
            from .views.timeline_window import TimelineWindow

            TimelineWindow(
                root,
                nodes[0],
                directory,
                dependencies=dependencies,
                on_close=scheduler.request_close,
            )
        else:
            from .views.snapshot_player import SnapshotPlayer

            SnapshotPlayer(
                root,
                nodes[0],
                directory,
                dependencies=dependencies,
            )
        return None

    return run_ros_desktop(
        argv,
        entrypoint,
        parser,
        ["saprog_snapshot_player"],
        build_app,
        require_timeline=lambda args: args.timeline,
    )


def snapshot_creator_main(argv=None, entrypoint=None):
    """Start the snapshot creator after ROS setup and dependency checks."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", default=default_snapshot_directory())

    def build_app(args, root, dependencies, nodes, scheduler):
        from .views.snapshot_window import SnapshotWindow

        SnapshotWindow(
            root,
            nodes[0],
            os.path.abspath(os.path.expanduser(args.directory)),
            dependencies=dependencies,
        )
        return None

    return run_ros_desktop(
        argv,
        entrypoint,
        parser,
        ["saprog_snapshot_window"],
        build_app,
    )


def temperature_monitor_main(argv=None, entrypoint=None):
    """Start the temperature monitor with the root scheduler as spin owner."""
    parser = argparse.ArgumentParser()

    def build_app(_args, root, dependencies, nodes, scheduler):
        from .views.temperature_monitor import TemperatureMonitor
        from .desktop_style import apply_style

        apply_style(root)
        root.title("Temperature Monitor")
        root.geometry("1050x700")
        root.minsize(700, 450)
        app = TemperatureMonitor(root, nodes[0], dependencies)

        def poll():
            if not app.closed:
                app.poll()
                root.after(100, poll)

        root.after(100, poll)

        def close_app():
            app.close()
            scheduler.request_close()

        return close_app

    return run_ros_desktop(
        argv,
        entrypoint,
        parser,
        ["saprog_temperature_monitor"],
        build_app,
    )


def imu_monitor_main(argv=None, entrypoint=None):
    """Start the standalone IMU display under root-owned ROS scheduling."""
    parser = argparse.ArgumentParser()

    def build_app(_args, root, dependencies, nodes, scheduler):
        from .views.imu_monitor import ImuMonitor

        app = ImuMonitor(root, nodes[0], dependencies=dependencies)

        def close_app():
            app.close()
            scheduler.request_close()

        return close_app

    return run_ros_desktop(
        argv,
        entrypoint,
        parser,
        ["saprog_imu_monitor"],
        build_app,
    )


def static_animator_main(argv=None, entrypoint=None):
    """Start the canonical Creator/Player/Timeline composition."""
    parser = argparse.ArgumentParser(description="Open the saprog Static Animator.")
    parser.add_argument("--directory", default=default_snapshot_directory())

    def build_app(args, root, dependencies, nodes, scheduler):
        from .apps.static_animator import build_animator_views
        from .desktop_style import apply_style

        apply_style(root)
        root.title("Static Animator")
        root.geometry("1200x900")
        views = build_animator_views(
            root,
            root,
            nodes[0],
            nodes[1],
            nodes[2],
            args.directory,
            dependencies=dependencies,
            on_closed=scheduler.request_close,
        )
        return views.request_close

    return run_ros_desktop(
        argv,
        entrypoint,
        parser,
        [
            "saprog_static_animator_creator",
            "saprog_static_animator_player",
            "saprog_static_animator_timeline",
        ],
        build_app,
        require_timeline=True,
    )


def home_main(argv=None, entrypoint=None):
    """Start Home after bootstrap checks and own every shared ROS node."""
    parser = argparse.ArgumentParser()

    def build_app(_args, root, dependencies, nodes, scheduler):
        from .apps.home import Home

        app = Home(root, nodes[0], dependencies, scheduler)
        return app.close

    return run_ros_desktop(
        argv,
        entrypoint,
        parser,
        ["saprog_home_temperature_monitor"],
        build_app,
        require_timeline=True,
    )
