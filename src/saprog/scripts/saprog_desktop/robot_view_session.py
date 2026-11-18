"""Own the external ROS launch process used by the embedded robot renderer."""

import os
import signal
import subprocess
import tempfile
import threading
from pathlib import Path


class RendererSession:
    """Manage one process group and readiness marker; poll from the Tk thread."""

    def __init__(self, workspace, ros2="ros2"):
        self.workspace = Path(workspace)
        self.ros2 = ros2
        self.process = None
        self.ready_file = None
        self.ready = False
        self.stop_requested = False
        self.closing = False
        self.state = "stopped"
        self.message = ""
        self.exit_code = None

    def start(self, parent_window, width, height):
        if self.closing:
            return False
        if self.process is not None:
            if self.process.poll() is None:
                return False
            self._finish()
        descriptor, self.ready_file = tempfile.mkstemp(prefix="saprog-rviz-ready-")
        os.close(descriptor)
        os.unlink(self.ready_file)
        try:
            self.process = subprocess.Popen(
                [self.ros2, "launch", "saprog", "robot_view.launch.py",
                 f"parent_window:={parent_window}", f"width:={width}",
                 f"height:={height}", f"ready_file:={self.ready_file}"],
                cwd=str(self.workspace), env=os.environ.copy(), start_new_session=True,
            )
        except OSError as error:
            self.process = None
            self._remove_ready_file()
            self.state = "error"
            self.message = f"Could not start embedded RViz: {error}"
            return False
        self.ready = False
        self.stop_requested = False
        self.exit_code = None
        self.state = "starting"
        self.message = "Starting embedded RViz renderer…"
        return True

    def poll(self):
        if self.process is None:
            return self.state
        if not self.ready and self.ready_file and os.path.exists(self.ready_file):
            self.ready = True
            self.state = "running"
            self.message = "Embedded RViz is running. Drag in the viewport to orbit."
        if self.process.poll() is not None:
            self._finish()
        return self.state

    def stop(self):
        if self.process is None:
            return
        if self.process.poll() is not None:
            self._finish()
            return
        if self.stop_requested:
            return
        self.stop_requested = True
        self.state = "stopping"
        self.message = "Stopping embedded RViz…"
        process = self.process
        self._signal_group(process, signal.SIGTERM)
        timer = threading.Timer(1.8, self._force_stop, args=(process,))
        timer.daemon = True
        timer.start()

    def close(self):
        if self.closing:
            return
        self.closing = True
        process = self.process
        if process is not None and process.poll() is None:
            self._signal_group(process, signal.SIGTERM)

            def force_cleanup():
                self._force_stop(process)
                try:
                    process.wait(timeout=0.5)
                except subprocess.TimeoutExpired:
                    pass

            timer = threading.Timer(1.8, force_cleanup)
            timer.daemon = True
            timer.start()
        self._remove_ready_file()

    def _finish(self):
        process = self.process
        if process is None:
            return
        self.exit_code = process.returncode
        was_ready = self.ready
        was_stopping = self.stop_requested
        self._signal_group(process, signal.SIGTERM)
        timer = threading.Timer(0.8, self._force_stop, args=(process,))
        timer.daemon = True
        timer.start()
        self.process = None
        self.stop_requested = False
        self.ready = False
        self._remove_ready_file()
        if was_stopping:
            self.state, self.message = "stopped", "Embedded RViz stopped."
        elif not was_ready:
            self.state, self.message = "error", (
                "Embedded RViz failed during startup; check X11, RViz plugins, and launch output."
            )
        else:
            self.state, self.message = "error", (
                f"Embedded RViz exited unexpectedly (code {self.exit_code}); "
                "see the ROS launch output."
            )

    @staticmethod
    def _signal_group(process, sig):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass

    def _force_stop(self, process):
        self._signal_group(process, signal.SIGKILL)

    def _remove_ready_file(self):
        if self.ready_file:
            try:
                os.unlink(self.ready_file)
            except FileNotFoundError:
                pass
            self.ready_file = None
