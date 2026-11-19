"""Minimal Qt Home window, independent of ROS and WSL."""

import sys

from PySide6.QtWidgets import QApplication, QMainWindow, QTabWidget

from desktop_style import apply_style
from tabs import discover_tabs


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Saprog")
    apply_style(app)

    window = QMainWindow()
    window.setWindowTitle("Saprog")
    window.resize(1000, 700)
    window.setMinimumSize(640, 420)

    tabs = QTabWidget()
    tabs.setDocumentMode(True)
    for title, widget in discover_tabs():
        tabs.addTab(widget, title)
    window.setCentralWidget(tabs)
    window.show()
    return app.exec()
