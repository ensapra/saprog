"""Empty Home tab package."""

from PySide6.QtWidgets import QWidget

TAB_TITLE = "Home"
TAB_ORDER = 0


class HomeTab(QWidget):
    """Selectable Home tab, ready for its own UI content later."""


def create_tab():
    """Build the Home tab for the main window."""
    return HomeTab()
