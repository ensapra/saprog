"""On-demand Gazebo controls; constructing the tab starts no ROS providers."""

import json
from pathlib import Path

from PySide6.QtCore import QTimer
from PySide6.QtWidgets import (
    QCheckBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QVBoxLayout,
    QWidget,
)
from ros_manager.config import SIMULATION_STATUS
from servo_limits import get_servo_limits
from widgets.servo_layout import SERVO_NAMES

TAB_TITLE = "Simulation"
TAB_ORDER = 50
ROOT = Path(__file__).resolve().parents[3]


class SimulationTab(QWidget):
    def __init__(self, ros_manager=None, parent=None):
        super().__init__(parent)
        self.manager = ros_manager
        self.limits = get_servo_limits(ros_manager)
        self.token = None
        self.pending = None
        self.state = {"running": False, "busy": False, "viewer": False, "paused": False}
        layout = QVBoxLayout(self)
        title = QLabel("Simulation")
        title.setObjectName("title")
        layout.addWidget(title)
        description = QLabel(
            "Test the robot in Gazebo with gravity and a ground plane. "
            "Start without the viewer for the lowest CPU and GPU load."
        )
        description.setWordWrap(True)
        layout.addWidget(description)

        settings = self.settings = QGroupBox("Robot and physics")
        form = QFormLayout(settings)
        self.urdf_path = QLineEdit(str(ROOT / "urdf/urdf/URDF.urdf"))
        self.urdf_path.setAccessibleName("Robot URDF file")
        browse = QPushButton("Browse…")
        browse.clicked.connect(self.browse_urdf)
        path_row = QHBoxLayout()
        path_row.addWidget(self.urdf_path, 1)
        path_row.addWidget(browse)
        form.addRow("URDF", path_row)
        self.speed = self._number(0.1, 4.0, 0.5, 0.1, "×", 1)
        self.speed.setToolTip("0.5× runs in slow motion and reduces physics work per wall-clock second.")
        form.addRow("Simulation speed", self.speed)
        self.step_size = self._number(1, 10, 4, 1, " ms", 1)
        self.step_size.setToolTip("Larger steps reduce CPU work but give less precise contact dynamics.")
        form.addRow("Physics step", self.step_size)
        self.spawn_height = self._number(0.05, 2, 0.3, 0.05, " m", 2)
        form.addRow("Spawn height", self.spawn_height)
        self.viewer_on_start = QCheckBox("Open Gazebo viewer on start")
        self.viewer_on_start.setChecked(False)
        form.addRow(self.viewer_on_start)
        self.start_paused = QCheckBox("Start paused")
        form.addRow(self.start_paused)
        form.addRow("Gravity", QLabel("−9.81 m/s²"))
        layout.addWidget(settings)

        controls = QHBoxLayout()
        self.start_button = self._button("Start", self.start, controls)
        self.pause_button = self._button("Pause", self.pause, controls)
        self.reset_button = self._button("Reset", lambda: self.send("reset"), controls)
        self.stop_button = self._button("Stop", lambda: self.send("stop"), controls)
        self.viewer_button = self._button("Open viewer", self.toggle_viewer, controls)
        controls.addStretch()
        layout.addLayout(controls)

        hint = QLabel("The robot's joints move freely under gravity. Controls affect this simulation only.")
        hint.setWordWrap(True)
        hint.setObjectName("compactStatus")
        layout.addWidget(hint)

        self.metrics = QLabel("Simulation time: —    Actual speed: —")
        self.metrics.setWordWrap(True)
        layout.addWidget(self.metrics)
        self.status = QLabel("Stopped. Press Start to load the URDF.")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addStretch()
        self.watchdog = QTimer(self)
        self.watchdog.setSingleShot(True)
        self.watchdog.setInterval(95000)
        self.watchdog.timeout.connect(self.timed_out)
        if self.manager is not None:
            self.manager.message_received.connect(self.receive)
            self.manager.status_changed.connect(self.provider_status)
            self.manager.command_result.connect(self.command_result)
        self.update_controls()
        self.limits.status_changed.connect(self.status.setText)
        self.limits.changed.connect(self.limits_changed)

    def showEvent(self, event):
        super().showEvent(event)
        self.limits.refresh()

    def limits_changed(self, _bounds):
        if self.manager is not None and (self.state.get("running") or self.pending is not None):
            self.manager.simulation_command("stop")
        self.update_controls()

    @staticmethod
    def _number(lower, upper, value, step, suffix, decimals):
        widget = QDoubleSpinBox()
        widget.setRange(lower, upper)
        widget.setDecimals(decimals)
        widget.setSingleStep(step)
        widget.setValue(value)
        widget.setSuffix(suffix)
        return widget

    @staticmethod
    def _button(text, callback, layout):
        button = QPushButton(text)
        button.clicked.connect(callback)
        layout.addWidget(button)
        return button

    def browse_urdf(self):
        path, _ = QFileDialog.getOpenFileName(self, "Robot URDF", self.urdf_path.text(), "URDF (*.urdf)")
        if path:
            self.urdf_path.setText(path)

    def start(self):
        path = Path(self.urdf_path.text()).expanduser()
        if not path.is_file():
            self.status.setText("Choose an existing URDF file.")
            return
        try:
            bounds = self.limits.require(SERVO_NAMES)
        except ValueError as error:
            self.status.setText(str(error))
            return
        self.send(
            "start", urdf_path=str(path.resolve()), real_time_factor=self.speed.value(),
            step_size=self.step_size.value() / 1000, spawn_height=self.spawn_height.value(),
            viewer=self.viewer_on_start.isChecked(), paused=self.start_paused.isChecked(),
            servo_bounds=dict(bounds),
        )

    def pause(self):
        self.send("resume" if self.state.get("paused") else "pause")

    def toggle_viewer(self):
        self.send("hide_viewer" if self.state.get("viewer") else "show_viewer")

    def send(self, action, **options):
        if self.manager is None:
            self.status.setText("ROS manager is unavailable.")
            return
        if self.pending is not None:
            return
        try:
            if self.token is None:
                config = SIMULATION_STATUS
                self.token = self.manager.request_topic(
                    config.topic, config.message_type, package=config.package, executable=config.executable,
                )
            self.pending = self.manager.simulation_command(action, **options)
            self.status.setText(f"{action.replace('_', ' ').capitalize()} requested…")
            self.watchdog.start()
        except (RuntimeError, ValueError) as error:
            self.status.setText(str(error))
        self.update_controls()

    def receive(self, topic, message):
        if topic != SIMULATION_STATUS.topic:
            return
        try:
            state = json.loads(message["data"])
            if not isinstance(state, dict):
                return
            self.state.update(state)
            if "sim_time" in state:
                self.metrics.setText(
                    f"Simulation time: {state['sim_time']:.1f} s    "
                    f"Actual speed: {state.get('real_time_factor', 0):.2f}×    "
                    f"Body height: {state.get('height', 0):.3f} m    "
                    f"Tilt: {state.get('tilt_degrees', 0):.1f}°"
                )
            self.status.setText(state.get("error") or (
                "Gazebo statistics are stale. Check the simulator." if state.get("stale")
                else state.get("message", "")
            ))
            self.update_controls()
        except (ValueError, TypeError, KeyError):
            return

    def provider_status(self, topic, status):
        if self.token is not None and topic in ("", SIMULATION_STATUS.topic) and status != "Connected":
            self.status.setText(status)

    def command_result(self, request_id, success, message):
        if request_id != self.pending:
            return
        self.pending = None
        self.watchdog.stop()
        self.status.setText(message)
        self.update_controls()

    def timed_out(self):
        self.pending = None
        self.state["busy"] = False
        self.status.setText("Simulation request timed out. Check the ROS/WSL status and retry.")
        self.update_controls()

    def update_controls(self):
        busy = self.pending is not None or self.state.get("busy", False)
        running = self.state.get("running", False)
        self.settings.setEnabled(not running and not busy)
        self.start_button.setEnabled(not busy and not running and self.manager is not None)
        for widget in (self.pause_button, self.reset_button, self.stop_button, self.viewer_button):
            widget.setEnabled(running and not busy)
        self.pause_button.setText("Resume" if self.state.get("paused") else "Pause")
        self.viewer_button.setText("Close viewer" if self.state.get("viewer") else "Open viewer")

    def closeEvent(self, event):
        if self.token is not None:
            self.manager.release(self.token)
            self.token = None
        super().closeEvent(event)


def create_tab(ros_manager=None):
    return SimulationTab(ros_manager=ros_manager)
