"""Coordinate the WSL bridge's ROS executor, input, and lifecycle."""

import argparse
import json
import queue
import threading

from .commands import RobotCommands
from .processes import NodeProcesses
from .protocol import CommandReader, emit
from .subscriptions import TopicSubscriptions


def dispatch_command(line, commands, topics):
    try:
        command = json.loads(line)
        if not commands.handle(command):
            topics.handle(command)
    except (json.JSONDecodeError, KeyError, TypeError) as error:
        emit("status", topic="", status=f"Invalid manager request: {error}")


def run(workspace):
    # Import ROS only in WSL, keeping this package importable without ROS or Qt.
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from sensor_msgs.msg import JointState
    from std_srvs.srv import SetBool, Trigger

    rclpy.init(args=None)
    node = rclpy.create_node("saprog_desktop_wsl_bridge")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    shutdown_event = threading.Event()
    topics = TopicSubscriptions(node)
    commands = RobotCommands(node, JointState, SetBool, Trigger)
    processes = NodeProcesses(workspace, shutdown_event, topics.set_status)
    reader = CommandReader(shutdown_event)
    reader.start()

    try:
        while rclpy.ok():
            processes.collect_results()
            try:
                line = reader.queue.get(timeout=0.01)
                if line is None:
                    break
                dispatch_command(line, commands, topics)
                while True:
                    try:
                        line = reader.queue.get_nowait()
                    except queue.Empty:
                        break
                    if line is None:
                        return
                    dispatch_command(line, commands, topics)
            except queue.Empty:
                pass

            for request in topics.active_requests():
                processes.ensure_provider(request, node, topics.topic_status)
            commands.process_pending()
            executor.spin_once(timeout_sec=0.05)
    finally:
        processes.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--workspace", required=True)
    args = parser.parse_args()
    try:
        run(args.workspace)
    except Exception as error:
        emit("status", topic="", status=f"WSL ROS bridge failed: {error}")
        raise
