"""Tabbed workspace with detachable pages for the standalone desktop panels."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QMainWindow, QTabWidget


class DetachedTabWindow(QMainWindow):
    """Temporary top-level window that returns its page when closed."""

    def __init__(self, workspace, widget, title):
        super().__init__(workspace)
        self._workspace = workspace
        self._tab_widget = widget
        self.setWindowFlag(Qt.WindowType.Window, True)
        self.setWindowTitle(title)
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
        self.setCentralWidget(widget)
        self.resize(800, 560)

    def closeEvent(self, event):
        self._workspace.restore_tab(self._tab_widget, self.windowTitle())
        super().closeEvent(event)


class WorkspaceMainWindow(QMainWindow):
    """Main window whose panels are pages in one movable tab bar."""

    def __init__(self):
        super().__init__()
        self.tabs = QTabWidget(self)
        self.tabs.setMovable(True)
        self.tabs.tabBarDoubleClicked.connect(self.detach_tab)
        self.setCentralWidget(self.tabs)

    def add_tab(self, widget, title):
        """Add a panel page to the workspace."""
        self.tabs.addTab(widget, title)

    def detach_tab(self, index):
        """Open the selected page in a separate window."""
        if index < 0:
            return

        widget = self.tabs.widget(index)
        title = self.tabs.tabText(index)
        self.tabs.removeTab(index)
        window = DetachedTabWindow(self, widget, title)
        window.show()

    def restore_tab(self, widget, title):
        """Return a detached page to the main tab bar."""
        self.add_tab(widget, title)
        self.tabs.setCurrentWidget(widget)
