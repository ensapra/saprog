"""Empty Robot tab package."""

from PySide6.QtWidgets import QWidget

TAB_TITLE = "Robot Visualizer"
TAB_ORDER = 10


def create_tab():
    """Build the empty Robot tab."""
    return QWidget()
