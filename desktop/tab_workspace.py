"""Tabbed workspace with detachable pages for the standalone desktop panels."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QMainWindow, QTabWidget
from tabs.motion_recorder import MotionRecorderTab
from widgets.player import StaticAnimatorWidget


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
        # Removing a tab and reparenting it hides the page explicitly. Showing
        # the new window alone does not clear that hidden state.
        widget.show()
        self.resize(800, 560)

    def closeEvent(self, event):
        self._workspace.restore_tab(self._tab_widget, self.windowTitle())
        super().closeEvent(event)


class WorkspaceMainWindow(QMainWindow):
    """Main window whose panels are pages in one movable tab bar."""

    def __init__(self):
        super().__init__()
        self._detached_windows = {}
        self._pages = []
        self.tabs = QTabWidget(self)
        self.tabs.setMovable(True)
        self.tabs.tabBarDoubleClicked.connect(self.detach_tab)
        self.setCentralWidget(self.tabs)

    def add_tab(self, widget, title):
        """Add a panel page to the workspace."""
        if widget not in self._pages:
            self._pages.append(widget)
            if isinstance(widget, MotionRecorderTab):
                widget.pose_edit_requested.connect(self._edit_recorded_pose)
        self.tabs.addTab(widget, title)

    def activate_tab(self, widget):
        """Focus the existing page, including its detached window if present."""
        window = self._detached_windows.get(widget)
        if window is None:
            self.tabs.setCurrentWidget(widget)
            window = self
        if window.isMinimized():
            window.showNormal()
        else:
            window.show()
        window.raise_()
        window.activateWindow()

    def _edit_recorded_pose(self, recorder, frame):
        animator = next(
            (page for page in self._pages if isinstance(page, StaticAnimatorWidget)), None,
        )
        if animator is None:
            recorder.status.setText("Static Animator is unavailable in this workspace.")
            return
        self.activate_tab(animator)
        name = frame.get("label") or f"Pose {recorder.pose_list.currentRow() + 1}"
        animator.edit_pose(
            name, frame["positions"], lambda positions: recorder.apply_pose_edit(frame, positions),
        )
        recorder.status.setText(f"Editing {name} in Static Animator.")

    def detach_tab(self, index):
        """Open the selected page in a separate window."""
        if index < 0:
            return

        widget = self.tabs.widget(index)
        title = self.tabs.tabText(index)
        self.tabs.removeTab(index)
        # removeTab() removes the page from the stack but leaves it parented to
        # the stack. Explicitly release it before assigning it to another window.
        widget.setParent(None)
        window = DetachedTabWindow(self, widget, title)
        self._detached_windows[widget] = window
        window.show()

    def restore_tab(self, widget, title):
        """Return a detached page to the main tab bar."""
        self._detached_windows.pop(widget, None)
        self.add_tab(widget, title)
        self.tabs.setCurrentWidget(widget)
