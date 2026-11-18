#!/usr/bin/env python3
"""Open Creator, Player and Timeline together in one Tk window."""

import argparse
import os
import signal
import sys
import tkinter as tk

import snapshot_player
from snapshot_files import default_snapshot_directory


def main():
    parser = argparse.ArgumentParser(description="Open the saprog Static Animator.")
    parser.add_argument("--directory", default=default_snapshot_directory())
    args, ros_args = parser.parse_known_args()
    snapshot_player._load_runtime(sys.argv[1:], require_timeline=True, entrypoint=__file__)
    import rclpy
    from rclpy.node import Node
    from tkinter import ttk

    from desktop_style import apply_style
    from snapshot_window import SnapshotWindow
    from timeline_window import TimelineWindow

    rclpy.init(args=ros_args)
    nodes = [
        Node("saprog_static_animator_creator"),
        Node("saprog_static_animator_player"),
        Node("saprog_static_animator_timeline"),
    ]
    root = tk.Tk()
    closed = False
    try:
        apply_style(root)
        root.title("Static Animator")
        root.geometry("1200x900")
        notebook = ttk.Notebook(root)
        notebook.pack(fill="both", expand=True)
        creator_tab, player_tab, timeline_tab = (ttk.Frame(notebook) for _ in range(3))
        notebook.add(creator_tab, text="Snapshot Creator")
        notebook.add(player_tab, text="Snapshot Player")
        notebook.add(timeline_tab, text="Timeline")
        directory = os.path.abspath(os.path.expanduser(args.directory))
        SnapshotWindow(root, nodes[0], directory, container=creator_tab)
        snapshot_player.SnapshotPlayer(root, nodes[1], directory, container=player_tab)

        def destroy_window():
            nonlocal closed
            if not closed:
                closed = True
                root.destroy()

        app = TimelineWindow(
            root, nodes[2], directory, container=timeline_tab, on_close=destroy_window
        )
    except Exception:
        root.destroy()
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        raise

    def close():
        app.close()

    root.protocol("WM_DELETE_WINDOW", close)

    def spin():
        if not closed and rclpy.ok():
            for node in nodes:
                rclpy.spin_once(node, timeout_sec=0.0)
            if not closed:
                root.after(10, spin)

    def signal_close(_signum, _frame):
        # Give the timeline editor its normal cancellation/confirmation flow.
        root.after(0, close)

    signal.signal(signal.SIGTERM, signal_close)
    signal.signal(signal.SIGINT, signal_close)
    root.after(10, spin)
    try:
        root.mainloop()
    finally:
        for node in nodes:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
