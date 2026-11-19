"""Minimal Qt Home window, independent of ROS and WSL."""

import sys

from PySide6.QtWidgets import QApplication

from desktop_style import apply_style
from tab_workspace import WorkspaceMainWindow
from tabs import discover_tabs


def main():
    app = QApplication(sys.argv)
    app.setApplicationName("Saprog")
    apply_style(app)

    window = WorkspaceMainWindow()
    window.setWindowTitle("Saprog")
    window.resize(1000, 700)
    window.setMinimumSize(640, 420)

    for title, widget in discover_tabs():
        window.add_tab(widget, title)
    window.show()
    return app.exec()
