"""Controller selection, live inputs and persistent per-controller ROS mappings."""

import json
from pathlib import Path

from PySide6.QtCore import QSettings, QTimer
from PySide6.QtWidgets import (
    QApplication,
    QComboBox,
    QFileDialog,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QPushButton,
    QScrollArea,
    QSpinBox,
    QVBoxLayout,
    QWidget,
)

from .devices import ControllerMonitor
from .playback import ControllerPlayback

TAB_TITLE = "Controller"
TAB_ORDER = 40


class ControllerTab(QWidget):
    """Map controller inputs to torque, snapshots and one-shot or held sequences."""

    ACTIONS = (
        ("Unmapped", ""), ("Torque on", "torque_on"), ("Torque off", "torque_off"),
        ("Snapshot", "snapshot"), ("Sequence (hold to repeat)", "sequence"),
        ("Sequence (one shot)", "sequence_once"),
    )

    def __init__(self, parent=None, *, ros_manager=None, monitor=None, settings=None,
                 snapshot_directory=None, sequence_directory=None):
        super().__init__(parent)
        workspace = Path(__file__).resolve().parents[3]
        self.settings = settings if settings is not None else QSettings("Saprog", "Controller")
        try:
            self.profiles = json.loads(self.settings.value("mappings", "{}"))
            if not isinstance(self.profiles, dict):
                self.profiles = {}
        except (TypeError, ValueError):
            self.profiles = {}
        self.profile = None
        self.rows = {}
        self.pressed = set()
        self._baseline = True
        self._closed = False
        self.paths = {}
        self._source_folders = None
        self.monitor = monitor if monitor is not None else ControllerMonitor(self)
        self.playback = ControllerPlayback(
            ros_manager, snapshot_directory or workspace / "snapshots", self,
        )
        outer = QVBoxLayout(self)
        title = QLabel("Controller")
        title.setObjectName("title")
        outer.addWidget(title)
        hint = QLabel(
            "Choose a controller and map its inputs. Snapshots and torque run on press; "
            "sequences run once per press or repeat while held. "
            "The latest mapped press takes control."
        )
        hint.setWordWrap(True)
        outer.addWidget(hint)
        devices = QHBoxLayout()
        devices.addWidget(QLabel("Controller"))
        self.device_combo = QComboBox()
        self.device_combo.setAccessibleName("Controller selection")
        self.device_combo.addItem("No controller detected", None)
        self.device_combo.currentIndexChanged.connect(self._select_device)
        devices.addWidget(self.device_combo, 1)
        refresh = QPushButton("Refresh controllers")
        refresh.clicked.connect(self.monitor.refresh)
        devices.addWidget(refresh)
        outer.addLayout(devices)
        sources = QGridLayout()
        self.folders = {}
        for row, (kind, folder) in enumerate((
            ("snapshot", snapshot_directory or workspace / "snapshots"),
            ("sequence", sequence_directory or workspace / "sequences"),
        )):
            sources.addWidget(QLabel(f"{kind.capitalize()} folder"), row, 0)
            saved = self.settings.value(f"{kind}_directory", str(folder))
            field = QLineEdit(str(folder) if (
                snapshot_directory if kind == "snapshot" else sequence_directory
            ) is not None else saved)
            field.setAccessibleName(f"{kind.capitalize()} folder")
            field.editingFinished.connect(self.refresh_files)
            sources.addWidget(field, row, 1)
            browse = QPushButton("Browse…")
            browse.clicked.connect(lambda _checked=False, key=kind: self._browse(key))
            sources.addWidget(browse, row, 2)
            self.folders[kind] = field
        reload_files = QPushButton("Refresh files")
        reload_files.clicked.connect(self.refresh_files)
        sources.addWidget(reload_files, 0, 3, 2, 1)
        outer.addLayout(sources)
        motion = QHBoxLayout()
        self.speed = QSpinBox()
        self.speed.setRange(1, 1023)
        self.speed.setValue(512)
        self.torque = QSpinBox()
        self.torque.setRange(0, 1023)
        self.torque.setValue(1023)
        for label, field in (("Snapshot speed", self.speed), ("Snapshot torque", self.torque)):
            field.setAccessibleName(label)
            motion.addWidget(QLabel(label))
            motion.addWidget(field)
        motion.addStretch(1)
        stop = QPushButton("Stop playback")
        stop.clicked.connect(lambda: self._stop("Controller playback stopped"))
        motion.addWidget(stop)
        outer.addLayout(motion)
        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        outer.addWidget(self.scroll, 1)
        self.device_status = QLabel("Detecting controllers…")
        self.device_status.setWordWrap(True)
        outer.addWidget(self.device_status)
        self.status_label = QLabel("Mappings are saved automatically for each controller type.")
        self.status_label.setWordWrap(True)
        outer.addWidget(self.status_label)
        note = QLabel(
            "Release stops held sequences; one-shot sequences continue. "
            "A movement already sent may finish."
        )
        note.setWordWrap(True)
        note.setObjectName("compactStatus")
        outer.addWidget(note)
        self.monitor.devices_changed.connect(self._devices_changed)
        self.monitor.selected_changed.connect(self._selected_changed)
        self.monitor.inputs_changed.connect(self._inputs_changed)
        self.monitor.status_changed.connect(self.device_status.setText)
        self.playback.status_changed.connect(self.status_label.setText)
        self.refresh_files()
        self._build_rows([])
        app = QApplication.instance()
        if app is not None:
            app.aboutToQuit.connect(self.shutdown)
        QTimer.singleShot(0, lambda: self.monitor.start() if not self._closed else None)

    def _browse(self, kind):
        folder = QFileDialog.getExistingDirectory(
            self, f"Choose {kind} folder", self.folders[kind].text(),
        )
        if folder:
            self.folders[kind].setText(folder)
            self.refresh_files()

    def refresh_files(self):
        folders = {kind: field.text() for kind, field in self.folders.items()}
        if self._source_folders is not None and folders != self._source_folders:
            self._stop("Source folder changed")
        self._source_folders = folders
        for kind, field in self.folders.items():
            directory = Path(field.text()).expanduser()
            self.paths[kind] = sorted(
                (path for path in directory.rglob("*.json") if path.is_file()),
                key=lambda path: str(path).casefold(),
            ) if directory.is_dir() else []
            self.settings.setValue(f"{kind}_directory", str(directory))
        self.playback.snapshot_directory = Path(self.folders["snapshot"].text()).expanduser()
        for key, (_label, action, target) in self.rows.items():
            self._fill_targets(key, action.currentData(), target)

    def showEvent(self, event):
        super().showEvent(event)
        self.refresh_files()

    def _devices_changed(self, devices):
        previous = self.device_combo.currentData()
        self.device_combo.blockSignals(True)
        self.device_combo.clear()
        self.device_combo.addItem("Select a controller" if devices else "No controller detected", None)
        for device in devices:
            self.device_combo.addItem(f"{device['name']} ({device['id']})", device["id"])
        index = self.device_combo.findData(previous) if previous is not None else -1
        if index < 0:
            preferred = self.settings.value("device_guid", "")
            preferred_device = next((item for item in devices if item["guid"] == preferred), None)
            index = self.device_combo.findData(preferred_device["id"]) if preferred_device else (
                1 if devices else 0
            )
        self.device_combo.setCurrentIndex(index)
        self.device_combo.blockSignals(False)
        if previous != self.device_combo.currentData():
            self._select_device()

    def _select_device(self, _index=None):
        self._stop("Controller selection changed")
        self.monitor.select(self.device_combo.currentData())

    def _selected_changed(self, device):
        self._stop()
        self.pressed = set()
        self._baseline = True
        self.profile = device["guid"] if device else None
        if device:
            if not isinstance(self.profiles.get(self.profile), dict):
                self.profiles[self.profile] = {}
            self.settings.setValue("device_guid", self.profile)
        self._build_rows(device["controls"] if device else [])

    def _build_rows(self, controls):
        body = QWidget()
        grid = QGridLayout(body)
        grid.setColumnStretch(2, 1)
        for column, text in enumerate(("Input / live state", "Action", "Snapshot / sequence")):
            grid.addWidget(QLabel(text), 0, column)
        self.rows = {}
        for row, (key, name) in enumerate(controls, 1):
            label = QLabel(name)
            label.setProperty("inputName", name)
            grid.addWidget(label, row, 0)
            action = QComboBox()
            action.setAccessibleName(f"{name} action")
            for text, kind in self.ACTIONS:
                action.addItem(text, kind)
            saved = self._mapping(key)
            action.setCurrentIndex(max(0, action.findData(saved.get("kind", ""))))
            grid.addWidget(action, row, 1)
            target = QComboBox()
            target.setMinimumContentsLength(16)
            target.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
            target.setAccessibleName(f"{name} target")
            grid.addWidget(target, row, 2)
            self.rows[key] = (label, action, target)
            self._fill_targets(key, action.currentData(), target)
            action.currentIndexChanged.connect(
                lambda _index, control=key: self._action_changed(control)
            )
            target.currentIndexChanged.connect(
                lambda _index, control=key: self._save_mapping(control)
            )
        if not controls:
            grid.addWidget(QLabel("Select a connected controller to see its available inputs."), 1, 0, 1, 3)
        grid.setRowStretch(len(controls) + 2, 1)
        old = self.scroll.takeWidget()
        self.scroll.setWidget(body)
        if old is not None:
            old.deleteLater()

    def _mapping(self, key):
        profile = self.profiles.get(self.profile, {})
        mapping = profile.get(key, {})
        return mapping if isinstance(mapping, dict) else {}

    def _fill_targets(self, key, kind, target):
        target.blockSignals(True)
        target.clear()
        target.addItem("Choose a file…", None)
        source = "sequence" if kind == "sequence_once" else kind
        for path in self.paths.get(source, []):
            target.addItem(str(path.relative_to(Path(self.folders[source].text()).expanduser())), str(path.resolve()))
        saved = self._mapping(key)
        path = saved.get("path") if saved.get("kind") == kind else None
        index = target.findData(path)
        if path and index < 0:
            target.addItem(f"Missing: {Path(path).name}", path)
            index = target.count() - 1
        target.setCurrentIndex(max(0, index))
        target.setEnabled(kind in ("snapshot", "sequence", "sequence_once"))
        target.blockSignals(False)

    def _action_changed(self, key):
        _label, action, target = self.rows[key]
        self._fill_targets(key, action.currentData(), target)
        self._save_mapping(key)

    def _save_mapping(self, key):
        if self.profile is None:
            return
        self._stop("Mapping updated")
        _label, action, target = self.rows[key]
        self.profiles[self.profile][key] = {"kind": action.currentData(), "path": target.currentData()}
        self.settings.setValue("mappings", json.dumps(self.profiles))
        self.settings.sync()
        if self.settings.status() != QSettings.Status.NoError:
            self.status_label.setText("Mapping changed, but could not be saved")

    def _inputs_changed(self, pressed):
        pressed = set(pressed)
        previous, self.pressed = self.pressed, pressed
        for key, (label, _action, _target) in self.rows.items():
            active = key in pressed
            label.setStyleSheet("color: #34d399; font-weight: bold;" if active else "")
        for key in previous - pressed:
            self.playback.release(key)
        if self._baseline:
            self._baseline = False
            return
        edges = sorted(pressed - previous)
        # Torque off wins simultaneous presses; no later motion can re-enable it.
        off = next((key for key in edges if self._mapping(key).get("kind") == "torque_off"), None)
        for key in ([off] if off is not None else edges):
            mapping = self._mapping(key)
            kind = mapping.get("kind")
            try:
                if kind == "torque_on":
                    self.playback.torque(True)
                elif kind == "torque_off":
                    self.playback.torque(False)
                elif kind in ("snapshot", "sequence", "sequence_once"):
                    path = mapping.get("path")
                    if not path:
                        raise ValueError("Choose a file for this input first")
                    if kind == "snapshot":
                        self.playback.snapshot(path, self.speed.value(), self.torque.value())
                    else:
                        self.playback.start(key, path, repeat=kind == "sequence")
            except (OSError, RuntimeError, ValueError, TypeError, KeyError) as error:
                self._stop(f"Controller action failed: {error}")

    def _stop(self, message=None):
        self.playback.stop(message)

    def shutdown(self):
        if not self._closed:
            self._closed = True
            self.monitor.close()
            self.playback.close()

    def closeEvent(self, event):
        self.shutdown()
        super().closeEvent(event)


def create_tab(ros_manager=None):
    """Build the native Controller tab with the shared ROS manager."""
    return ControllerTab(ros_manager=ros_manager)
