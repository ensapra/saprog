"""Standalone ROS/Tk entry points; views remain independent of process startup."""

import argparse
import os
import signal
import sys

from .runtime import RootRosScheduler, load_desktop_dependencies
from .snapshot_files import default_snapshot_directory


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
    args, ros_args = parser.parse_known_args(argv)
    original_args = sys.argv[1:] if argv is None else argv
    dependencies = load_desktop_dependencies(
        original_args, require_timeline=args.timeline, entrypoint=entrypoint
    )
    dependencies.rclpy.init(args=ros_args)
    node = dependencies.Node("saprog_snapshot_player")
    root = dependencies.tk.Tk()
    scheduler = RootRosScheduler(root, dependencies, [node])
    try:
        directory = os.path.abspath(os.path.expanduser(args.directory))
        if args.timeline:
            from .views.timeline_window import TimelineWindow

            TimelineWindow(
                root,
                node,
                directory,
                dependencies=dependencies,
                on_close=scheduler.request_close,
            )
        else:
            from .views.snapshot_player import SnapshotPlayer

            SnapshotPlayer(root, node, directory, dependencies=dependencies)
            root.protocol("WM_DELETE_WINDOW", scheduler.request_close)
        scheduler.start()
        root.mainloop()
    finally:
        scheduler.shutdown()
        try:
            root.destroy()
        except Exception:
            pass
    return 0


def snapshot_creator_main(argv=None, entrypoint=None):
    """Start the snapshot creator after ROS setup and dependency checks."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--directory", default=default_snapshot_directory())
    args, ros_args = parser.parse_known_args(argv)
    original_args = sys.argv[1:] if argv is None else argv
    dependencies = load_desktop_dependencies(original_args, entrypoint=entrypoint)
    dependencies.rclpy.init(args=ros_args)
    node = dependencies.Node("saprog_snapshot_window")
    root = dependencies.tk.Tk()
    scheduler = RootRosScheduler(root, dependencies, [node])
    try:
        from .views.snapshot_window import SnapshotWindow

        SnapshotWindow(
            root,
            node,
            os.path.abspath(os.path.expanduser(args.directory)),
            dependencies=dependencies,
        )
        root.protocol("WM_DELETE_WINDOW", scheduler.request_close)
        scheduler.start()
        root.mainloop()
    finally:
        scheduler.shutdown()
        try:
            root.destroy()
        except Exception:
            pass
    return 0


def temperature_monitor_main(argv=None, entrypoint=None):
    """Start the temperature monitor with the root scheduler as spin owner."""
    original_args = sys.argv[1:] if argv is None else argv
    _unused, ros_args = argparse.ArgumentParser().parse_known_args(original_args)
    dependencies = load_desktop_dependencies(original_args, entrypoint=entrypoint)
    dependencies.rclpy.init(args=ros_args)
    node = dependencies.Node("saprog_temperature_monitor")
    root = dependencies.tk.Tk()
    scheduler = RootRosScheduler(root, dependencies, [node])
    try:
        from .views.temperature_monitor import TemperatureMonitor
        from .desktop_style import apply_style

        apply_style(root)
        root.title("Temperature Monitor")
        root.geometry("1050x700")
        root.minsize(700, 450)
        app = TemperatureMonitor(root, node, dependencies)

        def poll():
            if not app.closed:
                app.poll()
                root.after(100, poll)

        root.protocol("WM_DELETE_WINDOW", scheduler.request_close)
        root.after(100, poll)
        signal.signal(signal.SIGTERM, lambda *_args: (app.close(), scheduler.request_close()))
        signal.signal(signal.SIGINT, lambda *_args: (app.close(), scheduler.request_close()))
        scheduler.start()
        root.mainloop()
    finally:
        scheduler.shutdown()
        try:
            root.destroy()
        except Exception:
            pass
    return 0


def imu_monitor_main(argv=None, entrypoint=None):
    """Start the standalone IMU display under root-owned ROS scheduling."""
    original_args = sys.argv[1:] if argv is None else argv
    _unused, ros_args = argparse.ArgumentParser().parse_known_args(original_args)
    dependencies = load_desktop_dependencies(original_args, entrypoint=entrypoint)
    dependencies.rclpy.init(args=ros_args)
    root = dependencies.tk.Tk()
    node = dependencies.Node("saprog_imu_monitor")
    scheduler = RootRosScheduler(root, dependencies, [node])
    try:
        from .views.imu_monitor import ImuMonitor

        app = ImuMonitor(root, node, dependencies=dependencies)
        root.protocol("WM_DELETE_WINDOW", scheduler.request_close)
        signal.signal(signal.SIGTERM, lambda *_args: (app.close(), scheduler.request_close()))
        signal.signal(signal.SIGINT, lambda *_args: (app.close(), scheduler.request_close()))
        scheduler.start()
        root.mainloop()
    finally:
        scheduler.shutdown()
        try:
            root.destroy()
        except Exception:
            pass
    return 0


def home_main(argv=None, entrypoint=None):
    """Start Home after bootstrap checks and own every shared ROS node."""
    original_args = sys.argv[1:] if argv is None else argv
    dependencies = load_desktop_dependencies(
        original_args, require_timeline=True, entrypoint=entrypoint
    )
    _unused, ros_args = argparse.ArgumentParser().parse_known_args(original_args)
    dependencies.rclpy.init(args=ros_args)
    root = dependencies.tk.Tk()
    node = dependencies.Node("saprog_home_temperature_monitor")
    scheduler = RootRosScheduler(root, dependencies, [node])
    try:
        from .apps.home import Home

        app = Home(root, node, dependencies, scheduler)
        signal.signal(signal.SIGTERM, lambda *_args: app.close())
        signal.signal(signal.SIGINT, lambda *_args: app.close())
        scheduler.start()
        root.mainloop()
    finally:
        scheduler.shutdown()
        try:
            root.destroy()
        except Exception:
            pass
    return 0
