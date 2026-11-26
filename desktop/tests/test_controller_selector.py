"""Device restoration and selection behavior shared by both controller tabs."""

import os
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtCore import QObject, QSettings, Signal  # noqa: E402
from PySide6.QtWidgets import QApplication  # noqa: E402
from widgets.controller_selector import ControllerSelector  # noqa: E402

_APP = QApplication.instance() or QApplication([])


class Monitor(QObject):
    devices_changed = Signal(object)
    selected_changed = Signal(object)

    def __init__(self):
        super().__init__()
        self.start = Mock()
        self.refresh = Mock()


class ControllerSelectorTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.settings = QSettings(str(Path(self.temp.name) / "devices.ini"), QSettings.Format.IniFormat)
        self.monitor = Monitor()
        self.selector = ControllerSelector(self.monitor, self.settings)
        self.selections = []
        self.selector.selection_requested.connect(self.selections.append)
        self.addCleanup(self.selector.close)

    def test_detect_starts_once_then_refreshes(self):
        self.selector.detect()
        self.selector.detect()
        self.monitor.start.assert_called_once_with()
        self.monitor.refresh.assert_called_once_with()

    def test_restores_guid_after_disconnect_with_new_instance_id(self):
        self.settings.setValue("device_guid", "preferred")
        self.monitor.devices_changed.emit([
            dict(id=1, name="Pad", guid="other"),
            dict(id=2, name="Pad", guid="preferred"),
        ])
        self.assertEqual(self.selections, [2])
        self.monitor.devices_changed.emit([])
        self.assertIsNone(self.selections[-1])
        self.monitor.devices_changed.emit([
            dict(id=3, name="Pad", guid="other"),
            dict(id=4, name="Pad", guid="preferred"),
        ])
        self.assertEqual(self.selections[-1], 4)

    def test_refresh_retains_active_instance_without_reselecting(self):
        devices = [dict(id=1, name="First", guid="first"),
                   dict(id=2, name="Second", guid="second")]
        self.monitor.devices_changed.emit(devices)
        self.selector.combo.setCurrentIndex(2)
        self.monitor.selected_changed.emit(devices[1])
        self.assertEqual(self.settings.value("device_guid"), "second")
        self.monitor.devices_changed.emit(list(reversed(devices)))
        self.assertEqual(self.selector.combo.currentData(), 2)
        self.assertEqual(self.selections, [1, 2])


if __name__ == "__main__":
    unittest.main()
