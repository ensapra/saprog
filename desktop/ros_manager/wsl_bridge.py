"""ROS 2 bridge process run inside WSL by the Windows desktop manager.

Protocol: JSON commands arrive on stdin; status and plain-data message events are
written to stdout. Keep this file ROS-only and independent of Qt so it can run
inside the WSL ROS Python environment.
"""

import argparse
import json
import os
import queue
import signal
import shlex
import subprocess
import sys
import threading
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


def ensure_executable(workspace, package, cancel_event):
    quoted_package = shlex.quote(package)
    emit("status", topic="", status=f"Checking build for {package}")
    build = subprocess.Popen(
        ros_shell(
            workspace,
            f"cd {shlex.quote(workspace)} && colcon build --event-handlers console_direct+ --packages-up-to {quoted_package}",
        ),
        cwd=workspace,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    deadline = time.monotonic() + 600
    while True:
        try:
            stdout, stderr = build.communicate(timeout=0.2)
            break
        except subprocess.TimeoutExpired:
            cancelled = cancel_event.is_set()
            timed_out = time.monotonic() >= deadline
            if not cancelled and not timed_out:
                continue
            try:
                os.killpg(build.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                stdout, stderr = build.communicate(timeout=2)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(build.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                stdout, stderr = build.communicate()
            if cancelled:
                raise RuntimeError("ROS package build cancelled")
            raise RuntimeError("ROS package build timed out after 600 seconds")
    if build.returncode != 0:
        details = (stderr or stdout).strip()
        raise RuntimeError(details[-1500:] or "colcon build failed")
    build_output = f"{stdout}\n{stderr}"
    if f"Starting >>> {package}" in build_output:
        emit("status", topic="", status=f"Built {package}")
    else:
        emit("status", topic="", status=f"{package} is already up to date")


def launch_node(workspace, package, executable):
    quoted_package = shlex.quote(package)
    run_command = f"ros2 run {quoted_package} {shlex.quote(executable)}"
    command = (
        f"share_dir=$(ros2 pkg prefix --share {quoted_package} 2>/dev/null || true); "
        f"if [ -n \"$share_dir\" ] && [ -f \"$share_dir/config/saprog.yaml\" ]; then "
        f"exec {run_command} --ros-args --params-file \"$share_dir/config/saprog.yaml\"; "
        f"else exec {run_command}; fi"
    )
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


def stop_process(process):
    if process.poll() is None:
        try:
            os.killpg(process.pid, 15)
        except ProcessLookupError:
            return
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
    last_message_at = {}
    build_results = queue.Queue()
    build_jobs = {}
    start_time = time.monotonic()
    command_queue = queue.Queue()
    shutdown_event = threading.Event()

    def read_commands():
        try:
            for line in sys.stdin:
                command_queue.put(line)
        finally:
            shutdown_event.set()
            command_queue.put(None)

    command_reader = threading.Thread(target=read_commands, name="saprog-command-reader", daemon=True)
    command_reader.start()

    def set_status(topic, status):
        if last_status.get(topic) != status:
            last_status[topic] = status
            emit("status", topic=topic, status=status)

    def topic_status(topic, publishers):
        last_message = last_message_at.get(topic)
        if last_message is not None:
            return "Stale IMU data" if time.monotonic() - last_message > 2.0 else "Connected"
        return "Waiting for IMU data" if publishers > 0 else "Subscribed; waiting for publisher"

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
            last_message_at[topic] = time.monotonic()
            emit("message", topic=topic, data=dict(message_to_ordereddict(message)))

        subscriptions[request["topic"]] = node.create_subscription(
            message_class, request["topic"], receive, qos_profile_sensor_data
        )
        set_status(request["topic"], "Subscribed; waiting for publisher")

    try:
        while rclpy.ok():
            while True:
                try:
                    key, topic, process, error = build_results.get_nowait()
                except queue.Empty:
                    break
                build_jobs.pop(key, None)
                if error is not None:
                    set_status(topic, f"Could not start {key[0]}: {error}")
                elif process is not None:
                    processes[key] = process
                    set_status(topic, f"Started {key[1]}")

            try:
                line = command_queue.get(timeout=0.01)
                if line is None:
                    break
                try:
                    process_command(json.loads(line))
                except (json.JSONDecodeError, KeyError, TypeError) as error:
                    emit("status", topic="", status=f"Invalid manager request: {error}")
                while True:
                    try:
                        line = command_queue.get_nowait()
                    except queue.Empty:
                        break
                    if line is None:
                        return
                    try:
                        process_command(json.loads(line))
                    except (json.JSONDecodeError, KeyError, TypeError) as error:
                        emit("status", topic="", status=f"Invalid manager request: {error}")
            except queue.Empty:
                pass

            active_topics = {request["topic"] for request in requests.values()}
            for topic in set(subscriptions) - active_topics:
                node.destroy_subscription(subscriptions.pop(topic))
                last_message_at.pop(topic, None)
                last_status.pop(topic, None)

            for topic in active_topics:
                request = next(item for item in requests.values() if item["topic"] == topic)
                if time.monotonic() - start_time < 1.5:
                    continue

                key = (request["package"], request["executable"])
                process = processes.get(key)
                publishers = node.count_publishers(topic)
                if process is not None and process.poll() is None:
                    set_status(
                        topic,
                        topic_status(topic, publishers) if publishers > 0 else f"Starting {request['executable']}",
                    )
                    continue
                if publishers > 0:
                    set_status(topic, topic_status(topic, publishers))
                    continue
                if time.monotonic() - last_attempt.get(key, 0.0) < 5.0:
                    continue

                last_attempt[key] = time.monotonic()
                set_status(topic, f"Checking {request['package']}")
                if key not in build_jobs:
                    def build_and_launch(build_key=key, build_topic=topic, build_request=request):
                        try:
                            ensure_executable(
                                workspace,
                                build_request["package"],
                                cancel_event=shutdown_event,
                            )
                            if shutdown_event.is_set():
                                process = None
                            else:
                                emit(
                                    "status",
                                    topic=build_topic,
                                    status=f"Launching {build_request['package']} ({build_request['executable']})",
                                )
                                process = launch_node(
                                    workspace, build_request["package"], build_request["executable"]
                                )
                            error = None
                        except Exception as build_error:
                            process = None
                            error = build_error
                        build_results.put((build_key, build_topic, process, error))

                    build_thread = threading.Thread(
                        target=build_and_launch,
                        name=f"saprog-build-{request['package']}",
                        daemon=True,
                    )
                    build_jobs[key] = build_thread
                    build_thread.start()

            executor.spin_once(timeout_sec=0.05)
    finally:
        shutdown_event.set()
        for build_thread in build_jobs.values():
            build_thread.join()
        while True:
            try:
                _key, _topic, process, _error = build_results.get_nowait()
            except queue.Empty:
                break
            if process is not None:
                stop_process(process)
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
