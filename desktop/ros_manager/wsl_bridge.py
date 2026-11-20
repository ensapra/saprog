"""ROS 2 bridge process run inside WSL by the Windows desktop manager.

Protocol: JSON commands arrive on stdin; status and plain-data message events are
written to stdout. Keep this file ROS-only and independent of Qt so it can run
inside the WSL ROS Python environment.
"""

import argparse
import json
import os
import select
import shlex
import subprocess
import sys
import time


def emit(event, **values):
    print(json.dumps({"event": event, **values}, separators=(",", ":")), flush=True)


def ros_shell(workspace, command):
    distro = os.environ.get("ROS_DISTRO", "jazzy")
    parts = [f"source /opt/ros/{shlex.quote(distro)}/setup.bash"]
    overlay = os.path.join(workspace, "install", "setup.bash")
    if os.path.isfile(overlay):
        parts.append(f"source {shlex.quote(overlay)}")
    parts.append(command)
    return ["bash", "-lc", " && ".join(parts)]


def ensure_executable(workspace, package, executable):
    package_name = package
    quoted_package = shlex.quote(package_name)
    result = subprocess.run(
        ros_shell(workspace, f"ros2 pkg executables {quoted_package}"),
        cwd=workspace,
        capture_output=True,
        text=True,
        timeout=15,
        check=False,
    )
    expected = f"{package_name} {executable}"
    if result.returncode == 0 and expected in result.stdout.splitlines():
        return

    emit("status", topic="", status=f"Building {package_name}")
    build = subprocess.run(
        ros_shell(
            workspace,
            f"cd {shlex.quote(workspace)} && colcon build --packages-up-to {quoted_package}",
        ),
        cwd=workspace,
        capture_output=True,
        text=True,
        timeout=600,
        check=False,
    )
    if build.returncode != 0:
        details = (build.stderr or build.stdout).strip()
        raise RuntimeError(details[-1500:] or "colcon build failed")


def launch_node(workspace, package, executable):
    command = f"ros2 run {shlex.quote(package)} {shlex.quote(executable)}"
    return subprocess.Popen(
        ros_shell(workspace, f"exec {command}"),
        cwd=workspace,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def stop_processes(processes):
    for process in processes.values():
        if process.poll() is None:
            try:
                os.killpg(process.pid, 15)
            except ProcessLookupError:
                pass
    for process in processes.values():
        if process.poll() is None:
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, 9)
                except ProcessLookupError:
                    pass


def run(workspace):
    import rclpy
    from rosidl_runtime_py.convert import message_to_ordereddict
    from rosidl_runtime_py.utilities import get_message
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data

    rclpy.init(args=None)
    node = rclpy.create_node("saprog_desktop_wsl_bridge")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    requests = {}
    subscriptions = {}
    processes = {}
    last_attempt = {}
    last_status = {}
    start_time = time.monotonic()

    def set_status(topic, status):
        if last_status.get(topic) != status:
            last_status[topic] = status
            emit("status", topic=topic, status=status)

    def process_command(command):
        op = command.get("op")
        token = command.get("token")
        if op == "release":
            requests.pop(token, None)
            return
        if op != "subscribe" or not token:
            return

        request = {
            "topic": command["topic"],
            "message_type": command["message_type"],
            "package": command["package"],
            "executable": command["executable"],
        }
        previous = next(
            (entry for entry in requests.values() if entry["topic"] == request["topic"]),
            None,
        )
        if previous and previous != request:
            set_status(request["topic"], "Topic requested with a conflicting message type/provider")
            return
        requests[token] = request
        if request["topic"] in subscriptions:
            return

        try:
            message_class = get_message(request["message_type"])
        except (ImportError, AttributeError, ValueError) as error:
            set_status(request["topic"], f"Unknown ROS message type: {error}")
            return

        def receive(message, topic=request["topic"]):
            emit("message", topic=topic, data=dict(message_to_ordereddict(message)))

        subscriptions[request["topic"]] = node.create_subscription(
            message_class, request["topic"], receive, qos_profile_sensor_data
        )
        set_status(request["topic"], "Subscribed; waiting for publisher")

    try:
        while rclpy.ok():
            if select.select([sys.stdin], [], [], 0.01)[0]:
                line = sys.stdin.readline()
                if not line:
                    break
                try:
                    process_command(json.loads(line))
                except (json.JSONDecodeError, KeyError, TypeError) as error:
                    emit("status", topic="", status=f"Invalid manager request: {error}")

            active_topics = {request["topic"] for request in requests.values()}
            for topic in set(subscriptions) - active_topics:
                node.destroy_subscription(subscriptions.pop(topic))
                last_status.pop(topic, None)

            for topic in active_topics:
                request = next(item for item in requests.values() if item["topic"] == topic)
                if node.count_publishers(topic) > 0:
                    set_status(topic, "Connected")
                    continue
                if time.monotonic() - start_time < 1.5:
                    continue

                key = (request["package"], request["executable"])
                process = processes.get(key)
                if process is not None and process.poll() is None:
                    set_status(topic, f"Starting {request['executable']}")
                    continue
                if time.monotonic() - last_attempt.get(key, 0.0) < 5.0:
                    continue

                last_attempt[key] = time.monotonic()
                set_status(topic, f"Checking {request['package']}")
                try:
                    ensure_executable(workspace, *key)
                    emit("status", topic=topic, status=f"Launching {request['package']} ({request['executable']})")
                    processes[key] = launch_node(workspace, *key)
                    set_status(topic, f"Started {request['executable']}")
                except (OSError, RuntimeError, subprocess.SubprocessError) as error:
                    set_status(topic, f"Could not start {request['package']}: {error}")

            executor.spin_once(timeout_sec=0.05)
    finally:
        stop_processes(processes)
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


if __name__ == "__main__":
    main()
