"""Device and axis assignment for single-leg Cartesian joystick control."""

import json

from controller_input import ControllerMonitor
from PySide6.QtCore import QSettings, Signal
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QPushButton,
    QSpinBox,
    QVBoxLayout,
)
from widgets.controller_selector import ControllerSelector

from .joystick import CartesianJoystick


class JoystickPanel(QGroupBox):
    start_requested = Signal(bool)

    def __init__(self, manager=None, parent=None, *, monitor=None, settings=None):
        super().__init__("Joystick foot control", parent)
        self.monitor = monitor if monitor is not None else ControllerMonitor(self)
        self.settings = settings if settings is not None else QSettings("Saprog", "LegJoystick")
        self.engine = CartesianJoystick(manager, self, read_input=getattr(self.monitor, "read_state", None))
        self.guid = None
        self.pressed = set()
        self._loading = False
        self._closed = False
        layout = QVBoxLayout(self)
        hint = QLabel("Stick left/right moves the foot along X; up/down moves along Z. Y stays fixed. "
                      "Live control holds the leg's x1 servo at 0 rad. "
                      "Deflection sets speed; centering holds the target. Servo limits block further motion.")
        hint.setWordWrap(True)
        layout.addWidget(hint)
        self.controller_selector = ControllerSelector(self.monitor, self.settings, self)
        self.device = self.controller_selector.combo
        layout.addWidget(self.controller_selector)
        form = QFormLayout()
        self.setup_toggle = QCheckBox("Controller setup and motor settings")
        self.setup = QGroupBox("Controller setup and motor settings")
        self.setup.setVisible(False)
        self.setup_toggle.toggled.connect(self.setup.setVisible)
        setup_layout = QVBoxLayout(self.setup)
        setup_form = QFormLayout()
        setup_form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        form.setRowWrapPolicy(QFormLayout.RowWrapPolicy.WrapLongRows)
        self.horizontal_axis, self.vertical_axis = QComboBox(), QComboBox()
        self.horizontal_axis.setAccessibleName("Horizontal stick axis")
        self.vertical_axis.setAccessibleName("Vertical stick axis")
        self.reverse_horizontal = QCheckBox("Reverse horizontal")
        self.reverse_vertical = QCheckBox("Reverse vertical")
        self.enable_button = QComboBox()
        self.enable_button.setAccessibleName("Physical controller button to hold for live movement")
        self.enable_button.setToolTip("Choose a physical controller button. Press buttons and watch the live button readings below to identify them.")
        self.speed = QDoubleSpinBox()
        self.speed.setRange(.1, 50)
        self.speed.setValue(10)
        self.speed.setSuffix(" mm/s")
        self.deadzone = QSpinBox()
        self.deadzone.setRange(0, 80)
        self.deadzone.setValue(15)
        self.deadzone.setSuffix(" %")
        self.servo_speed, self.torque = QSpinBox(), QSpinBox()
        self.servo_speed.setRange(1, 1023)
        self.servo_speed.setValue(100)
        self.torque.setRange(1, 1023)
        self.torque.setValue(300)
        form.addRow("Foot speed", self.speed)
        form.addRow("Hold to move (live)", self.enable_button)
        layout.addLayout(form)
        layout.addWidget(self.setup_toggle)
        layout.addWidget(self.setup)
        for label, field in (("Left/right stick axis", self.horizontal_axis),
                             ("Up/down stick axis", self.vertical_axis),
                             ("", self.reverse_horizontal), ("", self.reverse_vertical),
                             ("Stick dead zone", self.deadzone),
                             ("Live servo speed ceiling (AX)", self.servo_speed),
                             ("Live torque (AX)", self.torque)):
            setup_form.addRow(label, field)
        setup_layout.addLayout(setup_form)
        self.readings = QLabel("Move the stick to identify its axes. Stick up normally gives a negative reading.")
        self.readings.setWordWrap(True)
        setup_layout.addWidget(self.readings)
        self.button_readings = QLabel("Controller buttons: none pressed")
        self.button_readings.setWordWrap(True)
        layout.addWidget(self.button_readings)
        row = QHBoxLayout()
        preview = QPushButton("Preview stick")
        preview.clicked.connect(lambda: self.start_requested.emit(False))
        live = QPushButton("Control real leg")
        live.setEnabled(manager is not None)
        live.clicked.connect(lambda: self.start_requested.emit(True))
        stop = QPushButton("Stop joystick")
        stop.clicked.connect(lambda: self.engine.stop())
        for button in (preview, live):
            row.addWidget(button)
        layout.addLayout(row)
        layout.addWidget(stop)
        self.status = QLabel("Detect a controller, then preview. Open controller setup if the stick needs remapping.")
        self.status.setWordWrap(True)
        layout.addWidget(self.status)
        layout.addStretch()
        self.engine.status_changed.connect(self.status.setText)
        self.controller_selector.selection_requested.connect(self._select_device)
        self.monitor.selected_changed.connect(self._selected_changed)
        self.monitor.inputs_changed.connect(self._buttons_changed)
        self.monitor.axes_changed.connect(self._axes_changed)
        self.monitor.status_changed.connect(self.status.setText)
        for field in (self.horizontal_axis, self.vertical_axis, self.enable_button):
            field.currentIndexChanged.connect(self._mapping_changed)
        for field in (self.reverse_horizontal, self.reverse_vertical):
            field.toggled.connect(self._mapping_changed)
        for field in (self.speed, self.deadzone, self.servo_speed, self.torque):
            field.valueChanged.connect(self._mapping_changed)

    def detect(self):
        self.controller_selector.detect()

    def _select_device(self, _instance=None):
        self.engine.stop("Joystick stopped: controller selection changed")
        self.monitor.select(self.device.currentData())

    def _selected_changed(self, device):
        self.engine.stop("Joystick stopped: controller changed or disconnected")
        self.guid = device["guid"] if device else None
        self._buttons_changed(set())
        self._loading = True
        for field in (self.horizontal_axis, self.vertical_axis, self.enable_button):
            field.clear()
        if device:
            for axis in range(device["axis_count"]):
                for field in (self.horizontal_axis, self.vertical_axis):
                    field.addItem(f"Axis {axis+1}", axis)
            for key, name in device["controls"]:
                if key.startswith("button:"):
                    self.enable_button.addItem(name, key)
            try:
                saved = json.loads(self.settings.value(f"profiles/{self.guid}", "{}"))
                if not isinstance(saved, dict):
                    saved = {}
            except (TypeError, ValueError):
                saved = {}
            for key, field, default in self._profile_fields():
                value = saved.get(key, default)
                if isinstance(field, QComboBox):
                    field.setCurrentIndex(max(0, field.findData(value)))
                elif isinstance(field, QCheckBox):
                    field.setChecked(value if isinstance(value, bool) else default)
                else:
                    field.setValue(value if type(value) in (int, float) else default)
        self._loading = False

    def _profile_fields(self):
        return (("horizontal_axis", self.horizontal_axis, 0),
                ("vertical_axis", self.vertical_axis, 1),
                ("reverse_horizontal", self.reverse_horizontal, False),
                ("reverse_vertical", self.reverse_vertical, False),
                ("enable_button", self.enable_button, "button:0"),
                ("speed", self.speed, 10), ("deadzone", self.deadzone, 15),
                ("servo_speed", self.servo_speed, 100), ("torque", self.torque, 300))

    def _mapping_changed(self):
        if self._loading:
            return
        self.engine.stop("Joystick stopped: mapping changed")
        if self.guid is not None:
            values = {}
            for key, field, _default in self._profile_fields():
                values[key] = (field.currentData() if isinstance(field, QComboBox) else
                               field.isChecked() if isinstance(field, QCheckBox) else field.value())
            self.settings.setValue(f"profiles/{self.guid}", json.dumps(values))

    def _buttons_changed(self, pressed):
        self.pressed = set(pressed)
        names = [self.enable_button.itemText(i) for i in range(self.enable_button.count())
                 if self.enable_button.itemData(i) in self.pressed]
        self.button_readings.setText("Controller buttons: " + (", ".join(names) or "none pressed"))

    def _axes_changed(self, axes):
        self.engine.update_input(axes, self.pressed)
        self.readings.setText(" | ".join(f"Axis {i+1}: {v:+.2f}" for i, v in enumerate(axes)) or
                              "Controller disconnected")

    def start(self, model, seed, hardware):
        if self.guid is None:
            raise ValueError("Detect and select a controller first")
        self.engine.start(
            model, seed, axes=(self.horizontal_axis.currentData(), self.vertical_axis.currentData()),
            horizontal=0,
            reverse=(self.reverse_horizontal.isChecked(), self.reverse_vertical.isChecked()),
            speed=self.speed.value()/1000, deadzone=self.deadzone.value()/100,
            hardware=hardware, enable_button=self.enable_button.currentData(),
            enable_button_name=self.enable_button.currentText(),
            servo_speed=self.servo_speed.value(), torque=self.torque.value(),
        )

    def shutdown(self):
        if not self._closed:
            self._closed = True
            self.engine.stop()
            self.monitor.close()
