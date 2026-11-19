"""Empty Controller tab package."""

from PySide6.QtWidgets import QWidget

TAB_TITLE = "Controller"
TAB_ORDER = 40


def create_tab():
    """Build the empty Controller tab."""
    return QWidget()
