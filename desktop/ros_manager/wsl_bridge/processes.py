"""ROS shell setup, cancellable builds, and managed node processes."""

import os
import queue
import shlex
import signal
import subprocess
import threading
import time
from collections import deque

from .protocol import emit


class NodeOutput:
    """Drain node output without blocking it, retaining a bounded diagnostic tail."""

    def __init__(self, stream):
        self.lines = deque(maxlen=20)
        self.lock = threading.Lock()
        self.thread = threading.Thread(target=self._read, args=(stream,), daemon=True)
        self.thread.start()

    def _read(self, stream):
        with stream:
            for line in stream:
                with self.lock:
                    self.lines.append(line.rstrip()[-1500:])

    def tail(self):
        self.thread.join(timeout=0.2)
        with self.lock:
            return "\n".join(self.lines).strip()[-1500:]


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
    usb_preflight = ""
    if package == "saprog_dynamixel":
        helper = os.path.join(os.path.dirname(workspace), "scripts", "ensure_usb.bash")
        usb_preflight = (
            f"if [ -f {shlex.quote(helper)} ]; then "
            f"if [ -z \"$share_dir\" ]; then exit 1; fi; "
            f"bash {shlex.quote(helper)} \"$share_dir/config/saprog.yaml\" || exit 1; fi; "
        )
    elif package == "saprog_imu":
        helper = os.path.join(os.path.dirname(workspace), "scripts", "ensure_imu_usb.bash")
        usb_preflight = (
            f'if [ -z "${{SAPROG_IMU_DEVICE:-}}" ] && [ -f {shlex.quote(helper)} ]; then '
            f"SAPROG_IMU_DEVICE=$(bash {shlex.quote(helper)}) || exit 1; "
            'export SAPROG_IMU_DEVICE; fi; '
        )
    command = (
        f"share_dir=$(ros2 pkg prefix --share {quoted_package} 2>/dev/null || true); "
        f"{usb_preflight}"
        f"if [ -n \"$share_dir\" ] && [ -f \"$share_dir/config/saprog.yaml\" ]; then "
        f"exec {run_command} --ros-args --params-file \"$share_dir/config/saprog.yaml\"; "
        f"else exec {run_command}; fi"
    )
    process = subprocess.Popen(
        ros_shell(workspace, command),
        cwd=workspace,
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        encoding="utf-8",
        errors="replace",
        start_new_session=True,
    )
    process.saprog_output = NodeOutput(process.stdout)
    return process


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


class NodeProcesses:
    """Discover providers, build and launch missing ones, and stop owned jobs."""

    def __init__(self, workspace, shutdown_event, set_status):
        self.workspace = workspace
        self.shutdown_event = shutdown_event
        self.set_status = set_status
        self.processes = {}
        self.started_at = {}
        self.last_attempt = {}
        self.failures = {}
        self.retry_at = {}
        self.failure_status = {}
        self.build_results = queue.Queue()
        self.build_jobs = {}
        self.built_packages = set()
        self.build_lock = threading.Lock()
        self.start_time = time.monotonic()

    def collect_results(self):
        while True:
            try:
                key, topic, process, error = self.build_results.get_nowait()
            except queue.Empty:
                break
            self.build_jobs.pop(key, None)
            if error is not None:
                self._record_failure(key, topic, f"Could not start {key[0]}: {error}")
            elif process is not None:
                self.processes[key] = process
                self.started_at[key] = time.monotonic()
                self.set_status(topic, f"Started {key[1]}")

    def ensure_provider(self, request, node, topic_status):
        if time.monotonic() - self.start_time < 1.5:
            return
        topic = request["topic"]
        key = (request["package"], request["executable"])
        process = self.processes.get(key)
        publishers = node.count_publishers(topic)
        if process is not None and process.poll() is None:
            if time.monotonic() - self.started_at.get(key, time.monotonic()) >= 30.0:
                self.failures.pop(key, None)
            self.set_status(
                topic,
                topic_status(topic, publishers) if publishers > 0
                else f"Starting {request['executable']}",
            )
            return
        if process is not None:
            exit_code = process.poll()
            self.processes.pop(key, None)
            self.started_at.pop(key, None)
            output = getattr(process, "saprog_output", None)
            details = output.tail() if isinstance(output, NodeOutput) else ""
            reason = f"{key[1]} exited with code {exit_code}"
            if details:
                reason += f": {details}"
            self._record_failure(key, topic, reason)
        if publishers > 0:
            self.set_status(topic, topic_status(topic, publishers))
            return
        if time.monotonic() < self.retry_at.get(key, 0.0):
            self.set_status(topic, self.failure_status[key])
            return
        if time.monotonic() - self.last_attempt.get(key, 0.0) < 5.0:
            return

        self.last_attempt[key] = time.monotonic()
        self.set_status(topic, f"Checking {request['package']}")
        if key not in self.build_jobs:
            build_thread = threading.Thread(
                target=self._build_and_launch,
                args=(key, topic, request),
                name=f"saprog-build-{request['package']}",
                daemon=True,
            )
            self.build_jobs[key] = build_thread
            build_thread.start()

    def _record_failure(self, key, topic, reason):
        failures = min(self.failures.get(key, 0) + 1, 5)
        self.failures[key] = failures
        delay = min(5 * 2 ** (failures - 1), 60)
        self.retry_at[key] = time.monotonic() + delay
        self.failure_status[key] = f"{reason}; retrying in {delay} seconds"
        self.set_status(topic, self.failure_status[key])

    def _build_and_launch(self, key, topic, request):
        try:
            with self.build_lock:
                if request["package"] not in self.built_packages:
                    ensure_executable(
                        self.workspace, request["package"], cancel_event=self.shutdown_event,
                    )
                    self.built_packages.add(request["package"])
            if self.shutdown_event.is_set():
                process = None
            else:
                emit(
                    "status", topic=topic,
                    status=f"Launching {request['package']} ({request['executable']})",
                )
                process = launch_node(self.workspace, request["package"], request["executable"])
            error = None
        except Exception as build_error:
            process = None
            error = build_error
        self.build_results.put((key, topic, process, error))

    def close(self):
        self.shutdown_event.set()
        for build_thread in self.build_jobs.values():
            build_thread.join()
        while True:
            try:
                _key, _topic, process, _error = self.build_results.get_nowait()
            except queue.Empty:
                break
            if process is not None:
                stop_process(process)
        stop_processes(self.processes)


