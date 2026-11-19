"""Empty Simulation tab package."""

from PySide6.QtWidgets import QWidget

TAB_TITLE = "Simulator"
TAB_ORDER = 20


def create_tab():
    """Build the empty Simulation tab."""
    return QWidget()
