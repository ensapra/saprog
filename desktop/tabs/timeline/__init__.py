"""UI-only TimeLine tab based on the legacy timeline editor layout."""

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QFormLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QPushButton,
    QSlider,
    QSpinBox,
    QSplitter,
    QVBoxLayout,
    QWidget,
)

TAB_TITLE = "TimeLine"
TAB_ORDER = 35


class TimeLineTab(QWidget):
    """Present the old timeline editor's layout without its behavior."""

    def __init__(self, parent=None):
        super().__init__(parent)
        outer = QVBoxLayout(self)
        outer.setContentsMargins(14, 12, 14, 12)
        outer.setSpacing(7)

        title = QLabel("Timeline")
        title.setObjectName("title")
        outer.addWidget(title)
        subtitle = QLabel(
            "Build a sequence of poses and waits. Select a transition to edit its motion."
        )
        subtitle.setObjectName("compactStatus")
        outer.addWidget(subtitle)

        actions = QHBoxLayout()
        actions.addWidget(QPushButton("Add snapshot…"))
        actions.addWidget(QPushButton("Add wait"))
        actions.addWidget(QPushButton("Refresh"))
        actions.addStretch(1)
        for label in ("New timeline", "Load timeline", "Save timeline"):
            actions.addWidget(QPushButton(label))
        outer.addLayout(actions)

        sources = QHBoxLayout()
        sources.addWidget(QLabel("Available sequence"))
        sources.addWidget(QComboBox(), 1)
        sources.addWidget(QPushButton("Load sequence"))
        sources.addSpacing(8)
        sources.addWidget(QLabel("Available snapshot"))
        sources.addWidget(QComboBox(), 1)
        sources.addWidget(QPushButton("Add to timeline"))
        outer.addLayout(sources)

        body = QSplitter(Qt.Orientation.Horizontal)
        body.setChildrenCollapsible(False)
        steps_panel = QWidget()
        steps_layout = QVBoxLayout(steps_panel)
        steps_layout.setContentsMargins(0, 0, 4, 0)
        heading = QHBoxLayout()
        heading.addWidget(QLabel("Steps"))
        hint = QLabel("Transitions move into the following pose")
        hint.setObjectName("compactStatus")
        heading.addStretch(1)
        heading.addWidget(hint)
        steps_layout.addLayout(heading)
        steps = QListWidget()
        steps.setAlternatingRowColors(True)
        steps.addItem("No steps yet")
        steps.setToolTip("Add a snapshot or wait to start building your timeline.")
        steps_layout.addWidget(steps, 1)
        step_actions = QHBoxLayout()
        for label in ("Up", "Down", "Duplicate", "Remove"):
            step_actions.addWidget(QPushButton(label))
        steps_layout.addLayout(step_actions)
        body.addWidget(steps_panel)

        editor = QGroupBox("Step settings")
        editor_layout = QVBoxLayout(editor)
        duration = QFormLayout()
        duration_picker = QComboBox()
        duration_picker.addItems(("Until reached", "X seconds"))
        duration.addRow("Duration", duration_picker)
        editor_layout.addLayout(duration)
        editor_layout.addWidget(QLabel("Control mode"))
        mode = QComboBox()
        mode.addItems(("Global Control", "Custom Control"))
        editor_layout.addWidget(mode)
        explanation = QLabel(
            "Select a transition or wait to see its settings here.\n"
            "Switching to finer controls copies current values."
        )
        explanation.setWordWrap(True)
        explanation.setObjectName("compactStatus")
        editor_layout.addWidget(explanation)
        controls = QGroupBox("Motion controls")
        controls_layout = QFormLayout(controls)
        for label, initial in (("Speed", 512), ("Torque strength", 512)):
            row = QWidget()
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(0, 0, 0, 0)
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setRange(1, 1023)
            slider.setValue(initial)
            value = QSpinBox()
            value.setRange(1, 1023)
            value.setValue(initial)
            value.setFixedWidth(76)
            row_layout.addWidget(slider, 1)
            row_layout.addWidget(value)
            controls_layout.addRow(label, row)
        editor_layout.addWidget(controls, 1)
        body.addWidget(editor)
        body.setStretchFactor(0, 2)
        body.setStretchFactor(1, 1)
        body.setSizes([660, 390])
        outer.addWidget(body, 1)

        lifecycle = QHBoxLayout()
        lifecycle.addWidget(QLabel("After timeline"))
        end_action = QComboBox()
        end_action.addItems(("Torque on", "Torque off"))
        lifecycle.addWidget(end_action)
        lifecycle.addSpacing(12)
        lifecycle.addWidget(QCheckBox("Loop timeline"))
        lifecycle.addStretch(1)
        outer.addLayout(lifecycle)

        playback = QHBoxLayout()
        playback.addWidget(QPushButton("▶  Play timeline"))
        playback.addWidget(QPushButton("Test selected step"))
        playback.addWidget(QPushButton("■  Stop timeline"))
        playback.addStretch(1)
        outer.addLayout(playback)
        stop_hint = QLabel(
            "Stop halts later steps; motion or torque changes already sent may finish."
        )
        stop_hint.setObjectName("compactStatus")
        outer.addWidget(stop_hint)
        status = QLabel("Waiting for separately started saprog controller")
        status.setFrameStyle(QLabel.Shape.Panel | QLabel.Shadow.Sunken)
        status.setMinimumHeight(42)
        outer.addWidget(status)


def create_tab():
    """Build the display-only TimeLine tab."""
    return TimeLineTab()
