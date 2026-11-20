"""Robot statistics tab and its data visualization widgets."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import QHBoxLayout, QSplitter, QWidget

from widgets.imu_visualizer import IMUVisualizer
from widgets.dynamixel_graph import DynamixelGraph

TAB_TITLE = "Statistics"
TAB_ORDER = 0


class StatisticsTab(QWidget):
    """Show robot statistics widgets."""

    def __init__(self, ros_manager=None, parent=None):
        super().__init__(parent)
        layout = QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        splitter = QSplitter(Qt.Orientation.Horizontal, self)
        splitter.setChildrenCollapsible(False)
        splitter.setHandleWidth(6)
        splitter.addWidget(DynamixelGraph(splitter, ros_manager=ros_manager))
        splitter.addWidget(IMUVisualizer(splitter, ros_manager=ros_manager))
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 1)
        splitter.setSizes([660, 330])
        layout.addWidget(splitter)


def create_tab(ros_manager=None):
    """Build the Statistics tab for the main window."""
    return StatisticsTab(ros_manager=ros_manager)
