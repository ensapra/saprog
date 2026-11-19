"""Empty Static Animator tab package."""

from PySide6.QtWidgets import QWidget

TAB_TITLE = "Static Animator"
TAB_ORDER = 30


def create_tab():
    """Build the empty Static Animator tab."""
    return QWidget()
