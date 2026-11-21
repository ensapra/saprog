"""Single-leg physics controls. Opening the page starts no ROS providers."""

import json
import math
from collections import deque
from pathlib import Path

import pyqtgraph as pg
from desktop_style import ACCENT_LIGHT, FOREGROUND, MUTED
from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QScrollArea,
    QSplitter,
    QVBoxLayout,
    QWidget,
)
from ros_manager.config import LEG_TEST_STATUS
from widgets.leg_linkage_view import LegLinkageView

TAB_TITLE = "Single Leg Test"
TAB_ORDER = 55

# Display units are mm, grams and degrees; the node contract uses SI units.
# These are example dimensions, NOT measurements extracted from the CAD meshes.
GEOMETRY = (
    ("pivot_x", "Motor to knee X", -400, 400, 0, 1000, " mm"),
    ("pivot_z", "Motor to knee Z", -400, 400, -75, 1000, " mm"),
    ("crank_radius", "Crank pin radius", 1, 500, 17, 1000, " mm"),
    ("rod_length", "Connecting rod length", 1, 500, 80, 1000, " mm"),
    ("knee_radius", "Knee pin radius", 1, 500, 25, 1000, " mm"),
    ("foot_length", "Knee to foot length", 1, 500, 150, 1000, " mm"),
    ("foot_offset", "Foot angle from knee arm", -360, 360, -100, 180 / math.pi, " deg"),
)
CALIBRATION = (
    ("crank_zero", "Crank angle at servo zero", -360, 360, math.degrees(0.8803420273988994),
     180 / math.pi, " deg"),
    ("servo_lower", "Servo lower limit", -math.pi, math.pi, 0.2661499152601324, 1, " rad"),
    ("servo_upper", "Servo upper limit", -math.pi, math.pi, 0.8803420273988994, 1, " rad"),
    ("initial_servo", "Initial servo angle", -math.pi, math.pi, 0.4, 1, " rad"),
)
MOTOR = (
    ("crank_mass", "Crank mass", 1, 5000, 13, 1000, " g"),
    ("rod_mass", "Rod mass", 1, 5000, 10, 1000, " g"),
    ("leg_mass", "Lower leg mass", 1, 5000, 58, 1000, " g"),
    ("torque_limit", "Motor torque limit", 0.01, 10, 0.5, 1, " N m"),
    ("speed_limit", "Motor no-load speed", 0.01, 10, 1, 1, " rad/s"),
    ("kp", "Position gain", 0.01, 100, 8, 1, " N m/rad"),
    ("kd", "Velocity damping", 0.001, 10, 0.2, 1, " N m s/rad"),
    ("foot_load", "Downward foot load", 0, 50, 0, 1, " N"),
    ("real_time_factor", "Simulation speed", 0.1, 2, 0.5, 1, " x"),
)


class SingleLegTestTab(QWidget):
    def __init__(self, ros_manager=None, parent=None):
        super().__init__(parent)
        self.manager, self.token, self.pending = ros_manager, None, None
        self.state = {"running": False, "paused": False, "sweeping": False}
        self.history = deque(maxlen=600)
        self.fields = {}
        self.active_limits = None
        layout = QVBoxLayout(self)
        title = QLabel("Single Leg Test")
        title.setObjectName("title")
        layout.addWidget(title)
        description = QLabel(
            "Test the crank, passive connecting rod and knee on a fixed mount. "
            "The physics solver closes the pin joint; only the crank has a motor."
        )
        description.setWordWrap(True)
        layout.addWidget(description)
        note = QLabel("Example geometry - check dimensions and assembly branch against your leg before using results.")
        note.setWordWrap(True)
        layout.addWidget(note)
        splitter = QSplitter(Qt.Orientation.Horizontal)
        layout.addWidget(splitter, 1)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        settings = QWidget()
        settings_layout = QVBoxLayout(settings)
        self.settings_groups = []
        for name, definitions in (("Linkage geometry", GEOMETRY), ("Servo calibration", CALIBRATION),
                                  ("Motor and load (example model)", MOTOR)):
            group = QGroupBox(name)
            form = QFormLayout(group)
            form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
            for key, label, low, high, default, scale, suffix in definitions:
                decimals = 6 if key in ("servo_lower", "servo_upper", "initial_servo") else (
                    2 if scale != 1 else 3
                )
                field = self.number(low, high, default, suffix, decimals)
                field.setSingleStep(1 if scale == 1000 else 0.05)
                field.setAccessibleName(label)
                self.fields[key] = field
                form.addRow(label, field)
            if definitions is GEOMETRY:
                self.branch = QComboBox()
                self.branch.addItem("Branch +1", 1)
                self.branch.addItem("Branch -1", -1)
                self.branch.setToolTip("Choose the physical side of the four-bar assembly.")
                form.addRow("Assembly", self.branch)
            settings_layout.addWidget(group)
            self.settings_groups.append(group)
        profile_row = QHBoxLayout()
        self.load_button = self.button("Load settings", self.load_profile, profile_row)
        self.button("Save settings", self.save_profile, profile_row)
        settings_layout.addLayout(profile_row)
        settings_layout.addStretch()
        scroll.setWidget(settings)
        splitter.addWidget(scroll)
        right = QWidget()
        right_layout = QVBoxLayout(right)
        self.view = LegLinkageView()
        right_layout.addWidget(self.view, 2)
        self.plot = pg.PlotWidget()
        self.plot.setBackground(None)
        self.plot.setLabel("bottom", "Simulation time", units="s")
        self.plot.setLabel("left", "Angle", units="rad")
        self.plot.showGrid(x=True, y=True, alpha=0.2)
        self.plot.addLegend()
        self.curves = [self.plot.plot(pen=pg.mkPen(color, width=2), name=name) for name, color in (
            ("Servo", ACCENT_LIGHT), ("Target", MUTED), ("Knee", FOREGROUND),
        )]
        right_layout.addWidget(self.plot, 1)
        self.metrics = QLabel("No simulation data yet.")
        self.metrics.setWordWrap(True)
        right_layout.addWidget(self.metrics)
        splitter.addWidget(right)
        splitter.setStretchFactor(0, 0)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([390, 700])

        controls = QHBoxLayout()
        self.start_button = self.button("Start", self.start, controls)
        self.pause_button = self.button("Pause", self.pause, controls)
        self.reset_button = self.button("Reset", lambda: self.send("reset"), controls)
        self.stop_button = self.button("Stop", lambda: self.send("stop"), controls)
        controls.addStretch()
        layout.addLayout(controls)
        movement = QHBoxLayout()
        movement.addWidget(QLabel("Servo target"))
        self.target = self.number(-math.pi, math.pi, 0.4, " rad", 6)
        movement.addWidget(self.target)
        self.target_button = self.button("Move", lambda: self.send("target", angle=self.target.value()), movement)
        self.hold_button = self.button("Hold", lambda: self.send("stop_sweep"), movement)
        movement.addStretch()
        layout.addLayout(movement)
        movement = QHBoxLayout()
        movement.addWidget(QLabel("Sweep"))
        self.sweep_lower = self.number(-math.pi, math.pi, 0.3, " rad", 6)
        self.sweep_upper = self.number(-math.pi, math.pi, 0.8, " rad", 6)
        self.period = self.number(1, 120, 8, " s", 1)
        for field, name in ((self.sweep_lower, "Sweep lower angle"), (self.sweep_upper, "Sweep upper angle"),
                            (self.period, "Sweep period in simulation seconds")):
            field.setAccessibleName(name)
            movement.addWidget(field)
        self.sweep_button = self.button("Sweep", self.sweep, movement)
        movement.addStretch()
        layout.addLayout(movement)
        self.status = QLabel("Stopped. Press Start to launch the single-leg node.")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        self.watchdog = QTimer(self)
        self.watchdog.setSingleShot(True)
        self.watchdog.setInterval(95000)
        self.watchdog.timeout.connect(self.timed_out)
        self.stale_timer = QTimer(self)
        self.stale_timer.setSingleShot(True)
        self.stale_timer.setInterval(3000)
        self.stale_timer.timeout.connect(self.stale)
        if self.manager is not None:
            self.manager.message_received.connect(self.receive)
            self.manager.command_result.connect(self.command_result)
            self.manager.status_changed.connect(self.provider_status)
        self.update_controls()

    @staticmethod
    def number(low, high, default, suffix, decimals=4):
        field = QDoubleSpinBox()
        field.setDecimals(decimals)
        field.setRange(low, high)
        field.setValue(default)
        field.setSuffix(suffix)
        field.setSingleStep(0.05)
        return field

    @staticmethod
    def button(text, callback, row):
        button = QPushButton(text)
        button.clicked.connect(callback)
        row.addWidget(button)
        return button

    def configuration(self):
        values = {key: self.fields[key].value() / scale
                  for key, _, _, _, _, scale, _ in (*GEOMETRY, *CALIBRATION, *MOTOR)}
        values["branch"] = self.branch.currentData()
        return values

    def start(self):
        config = self.configuration()
        if not config["servo_lower"] < config["servo_upper"]:
            self.status.setText("Choose ordered servo travel limits.")
            return
        config["initial_servo"] = max(
            config["servo_lower"], min(config["servo_upper"], config["initial_servo"]),
        )
        self.fields["initial_servo"].setValue(config["initial_servo"])
        self.active_limits = (config["servo_lower"], config["servo_upper"])
        self.target.setValue(config["initial_servo"])
        self.sweep_lower.setValue(config["servo_lower"])
        self.sweep_upper.setValue(config["servo_upper"])
        self.send("start", config=config)

    def pause(self):
        self.send("resume" if self.state.get("paused") else "pause")

    def sweep(self):
        self.send("sweep", lower=self.sweep_lower.value(), upper=self.sweep_upper.value(), period=self.period.value())

    def send(self, action, **options):
        if self.manager is None:
            self.status.setText("ROS manager is unavailable.")
            return
        if self.pending is not None:
            return
        if action in ("target", "sweep") and self.active_limits:
            low, high = self.active_limits
            if action == "target":
                if type(options["angle"]) not in (int, float) or not math.isfinite(options["angle"]):
                    self.status.setText("Target angle must be a finite number.")
                    return
                options["angle"] = max(low, min(high, options["angle"]))
                self.target.setValue(options["angle"])
            else:
                if any(
                    type(options[key]) not in (int, float) or not math.isfinite(options[key])
                    for key in ("lower", "upper")
                ):
                    self.status.setText("Sweep angles must be finite numbers.")
                    return
                options["lower"] = max(low, min(high, options["lower"]))
                options["upper"] = max(low, min(high, options["upper"]))
                self.sweep_lower.setValue(options["lower"])
                self.sweep_upper.setValue(options["upper"])
        try:
            if self.token is None:
                config = LEG_TEST_STATUS
                self.token = self.manager.request_topic(
                    config.topic, config.message_type, package=config.package, executable=config.executable,
                )
            request_id = self.manager.leg_test_command(action, **options)
            self.pending = (request_id, action)
            self.watchdog.start()
            self.status.setText(f"{action.replace('_', ' ').capitalize()} requested...")
        except (RuntimeError, ValueError) as error:
            self.status.setText(str(error))
        self.update_controls()

    def receive(self, topic, message):
        if topic != LEG_TEST_STATUS.topic:
            return
        try:
            state = json.loads(message["data"])
            if not isinstance(state, dict):
                return
            if "sim_time" in state:
                for name in ("sim_time", "servo_angle", "servo_target", "knee_angle", "torque",
                             "servo_speed", "closure_error"):
                    if type(state[name]) not in (int, float) or not math.isfinite(state[name]):
                        return
                stamp = state["sim_time"]
                if self.history and stamp < self.history[-1][0]:
                    self.history.clear()
                if not self.history or stamp != self.history[-1][0]:
                    self.history.append((stamp, state["servo_angle"], state["servo_target"], state["knee_angle"]))
                for index, curve in enumerate(self.curves, 1):
                    curve.setData([r[0] for r in self.history], [r[index] for r in self.history])
                self.view.set_state(state)
                self.metrics.setText(
                    f"Time {stamp:.2f} s | Servo {state['servo_angle']:.4f} rad | "
                    f"Knee {state['knee_angle']:.4f} rad\n"
                    f"Motor {state['torque']:.3f} N m | Speed {state['servo_speed']:.3f} rad/s | "
                    f"Pin closure error {state['closure_error'] * 1000:.4f} mm"
                )
            self.state.update(state)
            if "config" in state:
                self.active_limits = (state["config"]["servo_lower"], state["config"]["servo_upper"])
            self.status.setText(state.get("error") or state.get("message", ""))
            self.stale_timer.start()
            self.update_controls()
        except (KeyError, TypeError, ValueError):
            return

    def command_result(self, request_id, success, message):
        if self.pending is None or self.pending[0] != request_id:
            return
        action = self.pending[1]
        self.pending = None
        self.watchdog.stop()
        if success:
            if action in ("start", "reset"):
                self.history.clear()
                self.view.clear()
                for curve in self.curves:
                    curve.clear()
            if action in ("start", "stop"):
                self.state["running"] = action == "start"
                self.state["paused"] = False
            if action in ("pause", "resume"):
                self.state["paused"] = action == "pause"
        self.status.setText(message)
        self.update_controls()

    def provider_status(self, topic, status):
        if self.token is not None and topic in ("", LEG_TEST_STATUS.topic) and status != "Connected":
            self.status.setText(status)

    def timed_out(self):
        self.pending = None
        self.status.setText("Single-leg request timed out. Check the ROS/WSL connection.")
        self.update_controls()

    def stale(self):
        self.status.setText("Single-leg telemetry is stale. Check the ROS/WSL connection.")

    def update_controls(self):
        busy, running = self.pending is not None, self.state.get("running", False)
        self.start_button.setEnabled(not busy and not running and self.manager is not None)
        for control in (self.pause_button, self.reset_button, self.stop_button,
                        self.target_button, self.sweep_button, self.hold_button):
            control.setEnabled(running and not busy)
        self.pause_button.setText("Resume" if self.state.get("paused") else "Pause")
        for group in self.settings_groups:
            group.setEnabled(not running and not busy)
        self.load_button.setEnabled(not running and not busy)

    def save_profile(self):
        path, _ = QFileDialog.getSaveFileName(self, "Save single-leg settings", "leg-test.json", "JSON (*.json)")
        if path:
            try:
                Path(path).write_text(json.dumps({"version": 1, "config": self.configuration()}, indent=2) + "\n",
                                      encoding="utf-8")
                self.status.setText("Settings saved. Dimensions still require physical validation.")
            except OSError as error:
                self.status.setText(str(error))

    def load_profile(self):
        path, _ = QFileDialog.getOpenFileName(self, "Load single-leg settings", "", "JSON (*.json)")
        if path:
            try:
                profile = json.loads(Path(path).read_text(encoding="utf-8"))
                config = profile["config"]
                if profile["version"] != 1 or set(config) != set(self.configuration()):
                    raise ValueError("Unsupported or incomplete single-leg settings")
                # Prepare every field before changing the form.
                updates = []
                for key, _, low, high, _, scale, _ in (*GEOMETRY, *CALIBRATION, *MOTOR):
                    value = config[key]
                    if type(value) not in (float, int) or not math.isfinite(value):
                        raise ValueError(f"Invalid value for {key}")
                    updates.append((self.fields[key], max(low, min(high, value * scale))))
                if type(config["branch"]) is not int or config["branch"] not in (-1, 1):
                    raise ValueError("Invalid assembly branch")
                for field, value in updates:
                    field.setValue(value)
                self.branch.setCurrentIndex(0 if config["branch"] == 1 else 1)
                self.status.setText("Settings loaded. Press Start to validate linkage closure.")
            except (OSError, ValueError, TypeError, KeyError) as error:
                self.status.setText(str(error))

    def closeEvent(self, event):
        self.watchdog.stop()
        self.stale_timer.stop()
        if self.token is not None:
            if self.state.get("running") or (self.pending and self.pending[1] == "start"):
                self.manager.leg_test_command("stop")
            self.manager.release(self.token)
            self.token = None
        super().closeEvent(event)


def create_tab(ros_manager=None):
    return SingleLegTestTab(ros_manager=ros_manager)
