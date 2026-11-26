"""Controller discovery UI shared by action mapping and Cartesian control."""

from PySide6.QtCore import Signal
from PySide6.QtWidgets import QComboBox, QHBoxLayout, QPushButton, QWidget


class ControllerSelector(QWidget):
    """Restore a controller by GUID while retaining its current instance ID.

    Consumers stop their motion before selecting the emitted instance on the
    monitor. This keeps cancellation policy with the owner of each motion.
    """

    selection_requested = Signal(object)

    def __init__(self, monitor, settings, parent=None):
        super().__init__(parent)
        self.monitor, self.settings = monitor, settings
        self._started = False
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        self.combo = QComboBox()
        self.combo.setMinimumContentsLength(12)
        self.combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self.combo.setAccessibleName("Controller selection")
        self.combo.addItem("Detect and select a controller", None)
        layout.addWidget(self.combo, 1)
        refresh = QPushButton("Detect controllers")
        refresh.clicked.connect(self.detect)
        layout.addWidget(refresh)
        self.combo.currentIndexChanged.connect(self._selection_changed)
        monitor.devices_changed.connect(self._devices_changed)
        monitor.selected_changed.connect(self._remember_device)

    def detect(self):
        if not self._started:
            self._started = True
            self.monitor.start()
        else:
            self.monitor.refresh()

    def _selection_changed(self):
        self.selection_requested.emit(self.combo.currentData())

    def _devices_changed(self, devices):
        previous = self.combo.currentData()
        self.combo.blockSignals(True)
        self.combo.clear()
        self.combo.addItem("Select controller" if devices else "No controller detected", None)
        for device in devices:
            self.combo.addItem(f"{device['name']} ({device['id']})", device["id"])
        index = self.combo.findData(previous) if previous is not None else -1
        if index < 0:
            preferred = self.settings.value("device_guid", "")
            index = next((i for i, device in enumerate(devices, 1)
                          if device["guid"] == preferred), 1 if devices else 0)
        self.combo.setCurrentIndex(index)
        self.combo.blockSignals(False)
        if previous != self.combo.currentData():
            self._selection_changed()

    def _remember_device(self, device):
        if device:
            self.settings.setValue("device_guid", device["guid"])
