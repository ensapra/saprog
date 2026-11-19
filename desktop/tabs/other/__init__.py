"""Empty Other panels tab package."""

from PySide6.QtWidgets import QWidget

TAB_TITLE = "Other"
TAB_ORDER = 1000


def create_tab():
    """Build the empty Other panels tab."""
    return QWidget()
