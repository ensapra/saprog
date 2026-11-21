"""Detached pages remain visible, interactive and alive when restored."""

import os
import sys
import unittest
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PySide6.QtCore import QCoreApplication, QEvent, Qt
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication, QLineEdit, QPushButton, QVBoxLayout, QWidget

from desktop.tab_workspace import WorkspaceMainWindow


class TabWorkspaceTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_double_click_detaches_visible_working_page_and_restores_state(self):
        for selected in (True, False):
            with self.subTest(selected=selected):
                window = WorkspaceMainWindow()
                page = QWidget()
                layout = QVBoxLayout(page)
                field = QLineEdit("Loaded document")
                button = QPushButton("Edit")
                button.clicked.connect(lambda _checked=False: field.setText("Edited document"))
                layout.addWidget(field)
                layout.addWidget(button)
                window.add_tab(page, "Document")
                window.add_tab(QWidget(), "Other")
                window.tabs.setCurrentIndex(0 if selected else 1)
                window.show()
                self.app.processEvents()
                try:
                    for _ in range(2):
                        bar = window.tabs.tabBar()
                        index = window.tabs.indexOf(page)
                        QTest.mouseDClick(
                            bar, Qt.MouseButton.LeftButton, pos=bar.tabRect(index).center()
                        )
                        self.app.processEvents()
                        detached = window._detached_windows[page]
                        self.assertIs(detached.centralWidget(), page)
                        self.assertEqual(window.tabs.indexOf(page), -1)
                        self.assertTrue(detached.isVisible())
                        self.assertTrue(page.isVisible())
                        self.assertTrue(field.isVisible())
                        self.assertTrue(button.isVisible())
                        self.assertGreater(page.width(), 0)
                        self.assertGreater(page.height(), 0)
                        QTest.mouseClick(button, Qt.MouseButton.LeftButton)
                        self.assertEqual(field.text(), "Edited document")
                        field.setText("Ready to detach again")

                        self.assertTrue(detached.close())
                        QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)
                        self.app.processEvents()
                        self.assertFalse(window._detached_windows)
                        self.assertIs(window.tabs.currentWidget(), page)
                        self.assertTrue(page.isVisible())
                        self.assertEqual(field.text(), "Ready to detach again")
                finally:
                    for detached in list(window._detached_windows.values()):
                        detached.close()
                    window.close()
                    window.deleteLater()
                    QCoreApplication.sendPostedEvents(None, QEvent.Type.DeferredDelete)


if __name__ == "__main__":
    unittest.main()
