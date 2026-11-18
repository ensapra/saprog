#!/usr/bin/env python3
"""Open Creator, Player and Timeline together in one Tk window."""

import argparse
import os
import signal
import sys
from dataclasses import dataclass

from saprog_desktop.views import snapshot_player
from saprog_desktop.runtime import RootRosScheduler
from saprog_desktop.snapshot_files import default_snapshot_directory
from saprog_desktop.startup import load_desktop_dependencies


@dataclass
class AnimatorViews:
    """Own the assembled Tk views without owning the root or ROS nodes."""

    creator: object
    player: object
    timeline: object
    notebook: object
    snapshots_split: object
    on_closed: object = None
    _disposed: bool = False

    def request_close(self):
        """Ask Timeline to resolve dirty edits and active actions before teardown."""
        if not self._disposed:
            self.timeline.close()

    def dispose(self):
        """Release view adapters after Timeline has reached a safe close outcome."""
        if self._disposed:
            return
        self._disposed = True
        self.creator.close()
        self.player.close()
        if self.on_closed is not None:
            self.on_closed()


def build_animator_views(
    root,
    parent,
    creator_node,
    player_node,
    timeline_node,
    directory,
    *,
    dependencies,
    torque_strength_changed=None,
    on_closed=None,
):
    """Build and own the shared Creator/Player and Timeline composition."""
    from saprog_desktop.views.snapshot_window import SnapshotWindow
    from saprog_desktop.views.timeline_window import TimelineWindow

    directory = os.path.abspath(os.path.expanduser(directory))
    ttk = dependencies.ttk
    notebook = ttk.Notebook(parent)
    notebook.pack(fill="both", expand=True)
    snapshots_tab, timeline_tab = (ttk.Frame(notebook) for _ in range(2))
    notebook.add(snapshots_tab, text="Creator / Player")
    notebook.add(timeline_tab, text="Timeline")
    snapshots_split = ttk.Panedwindow(snapshots_tab, orient="horizontal")
    snapshots_split.pack(fill="both", expand=True)
    creator_panel, player_panel = ttk.Frame(snapshots_split), ttk.Frame(snapshots_split)
    snapshots_split.add(creator_panel, weight=1)
    snapshots_split.add(player_panel, weight=1)

    if torque_strength_changed is None:

        def torque_strength_changed(value):
            strength = max(0, min(1023, round(float(value))))
            try:
                snapshot_player.save_torque_strength(
                    snapshot_player.player_settings_path(), strength
                )
            except OSError as error:
                creator.status.set(
                    f"Torque strength changed, but could not save preference: {error}"
                )

    creator = player = timeline = None
    try:
        creator = SnapshotWindow(
            root,
            creator_node,
            directory,
            container=creator_panel,
            show_snapshot_loader=False,
            initial_torque_strength=snapshot_player.load_torque_strength(
                snapshot_player.player_settings_path()
            ),
            torque_strength_changed=torque_strength_changed,
            dependencies=dependencies,
        )
        player = snapshot_player.SnapshotPlayer(
            root,
            player_node,
            directory,
            container=player_panel,
            on_pose_selected=creator.load_targets_from_file,
            moving_speed=creator.moving_speed,
            torque_strength=creator.torque_strength,
            hold_after_load=creator.hold_after_load,
            can_start_playback=lambda: not creator.playback_state and not creator.pending,
            dependencies=dependencies,
        )
        creator.external_busy = lambda: player.playback is not None or player.torque_batch is not None
        creator.stop_external_operation = player.stop_playback_and_disable_all_torque
        views = AnimatorViews(creator, player, None, notebook, snapshots_split, on_closed)
        timeline = TimelineWindow(
            root,
            timeline_node,
            directory,
            container=timeline_tab,
            on_close=views.dispose,
            dependencies=dependencies,
        )
        views.timeline = timeline
        return views
    except Exception:
        for view in (timeline, player, creator):
            if view is not None:
                view.close()
        raise


def main():
    parser = argparse.ArgumentParser(description="Open the saprog Static Animator.")
    parser.add_argument("--directory", default=default_snapshot_directory())
    args, ros_args = parser.parse_known_args()
    dependencies = load_desktop_dependencies(
        sys.argv[1:], require_timeline=True, entrypoint=__file__
    )
    from saprog_desktop.desktop_style import apply_style

    dependencies.rclpy.init(args=ros_args)
    nodes = [
        dependencies.Node("saprog_static_animator_creator"),
        dependencies.Node("saprog_static_animator_player"),
        dependencies.Node("saprog_static_animator_timeline"),
    ]
    root = dependencies.tk.Tk()
    scheduler = RootRosScheduler(root, dependencies, nodes)
    try:
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
    except Exception:
        scheduler.shutdown()
        root.destroy()
        raise

    def close():
        views.request_close()

    root.protocol("WM_DELETE_WINDOW", close)

    def signal_close(_signum, _frame):
        # Give the timeline editor its normal cancellation/confirmation flow.
        root.after(0, close)

    signal.signal(signal.SIGTERM, signal_close)
    signal.signal(signal.SIGINT, signal_close)
    scheduler.start()
    try:
        root.mainloop()
    finally:
        scheduler.shutdown()
        try:
            root.destroy()
        except Exception:
            pass


if __name__ == "__main__":
    main()
