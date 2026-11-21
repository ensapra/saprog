"""Local timeline editor with no ROS or playback calls."""

from pathlib import Path

from PySide6.QtCore import QEvent, QSize, Qt
from PySide6.QtWidgets import (
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QMessageBox,
    QPushButton,
    QScrollArea,
    QSlider,
    QSpinBox,
    QSplitter,
    QStyle,
    QVBoxLayout,
    QWidget,
)

from ..controller.playback import ControllerPlayback
from .backend import (
    CONTROL_MODES,
    MIRROR_MODES,
    TimelineDocument,
    control_groups,
    motion_settings,
    timeline_rows,
    transition_duration,
)

TAB_TITLE = "TimeLine"
TAB_ORDER = 35


class TimeLineTab(QWidget):
    """Edit and persist timelines locally using the shared timeline file format."""

    def __init__(self, parent=None, *, ros_manager=None, snapshot_directory=None, sequence_directory=None):
        super().__init__(parent)
        workspace = Path(__file__).resolve().parents[3]
        snapshot_directory = snapshot_directory or workspace / "snapshots"
        sequence_directory = sequence_directory or workspace / "sequences"
        self.playback = ControllerPlayback(ros_manager, snapshot_directory, self)
        self.document = TimelineDocument(snapshot_directory, bounds=self.playback.bounds)
        self._syncing = False
        self._watched_window = None
        self.motion_controls = {}
        outer = QVBoxLayout(self)
        outer.setContentsMargins(14, 12, 14, 12)
        outer.setSpacing(7)

        actions = QHBoxLayout()
        add_wait = QPushButton("Add wait")
        add_wait.clicked.connect(self.add_wait)
        actions.addWidget(add_wait)
        actions.addStretch(1)
        for label, icon, callback in (
            ("New timeline", QStyle.StandardPixmap.SP_FileIcon, self.new_timeline),
            ("Load timeline", QStyle.StandardPixmap.SP_DialogOpenButton, self.load_timeline),
            ("Save timeline", QStyle.StandardPixmap.SP_DialogSaveButton, self.save_timeline),
        ):
            button = QPushButton()
            button.setIcon(self.style().standardIcon(icon))
            button.setIconSize(QSize(24, 24))
            button.setToolTip(label)
            button.setAccessibleName(label)
            button.clicked.connect(lambda _checked=False, action=callback: action())
            actions.addWidget(button)
        outer.addLayout(actions)

        sources = QGridLayout()
        sources.setColumnStretch(1, 1)
        sources.setColumnStretch(2, 1)
        self.source_controls = []
        for row, (label, folder) in enumerate(
            (("Available sequences", sequence_directory), ("Available snapshots", snapshot_directory))
        ):
            sources.addWidget(QLabel(label), row, 0)
            picker = QComboBox()
            picker.setMinimumContentsLength(12)
            picker.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
            picker.setAccessibleName(label)
            sources.addWidget(picker, row, 1)
            source_type = "sequences" if row == 0 else "snapshots"
            directory = QLineEdit(str(folder))
            directory.setAccessibleName(f"{label} folder")
            directory.setToolTip(f"Folder containing {source_type} JSON files")
            directory.editingFinished.connect(self.refresh_sources)
            sources.addWidget(directory, row, 2)
            browse = QPushButton("Browse…")
            browse.setAccessibleName(f"Browse {source_type}")
            browse.clicked.connect(
                lambda _checked=False, field=directory: self.browse_directory(field)
            )
            sources.addWidget(browse, row, 3)
            load = QPushButton("Load")
            load.setAccessibleName(f"Load {source_type.removesuffix('s')}")
            load.setToolTip(
                "Load selected sequence" if row == 0 else "Add selected snapshot to timeline"
            )
            load.clicked.connect(lambda _checked=False, source=row: self.load_source(source))
            sources.addWidget(load, row, 4)
            self.source_controls.append((picker, directory))
        outer.addLayout(sources)
        self.refresh_sources()

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
        self.steps_list = QListWidget()
        self.steps_list.setAlternatingRowColors(True)
        self.steps_list.setToolTip("Select a transition or wait to edit its settings.")
        self.steps_list.currentRowChanged.connect(self._select_step)
        steps_layout.addWidget(self.steps_list, 1)
        step_actions = QHBoxLayout()
        self.step_buttons = {}
        for label, callback in (
            ("Up", lambda: self.move_step(-1)),
            ("Down", lambda: self.move_step(1)),
            ("Duplicate", self.duplicate_step),
            ("Remove", self.remove_step),
        ):
            button = QPushButton(label)
            button.clicked.connect(lambda _checked=False, action=callback: action())
            self.step_buttons[label] = button
            step_actions.addWidget(button)
        steps_layout.addLayout(step_actions)
        body.addWidget(steps_panel)

        self.editor = QGroupBox("Step settings")
        editor_layout = QVBoxLayout(self.editor)
        self.editor_scroll = QScrollArea()
        self.editor_scroll.setWidgetResizable(True)
        self.editor_scroll.setFrameShape(QScrollArea.Shape.NoFrame)
        editor_layout.addWidget(self.editor_scroll)
        body.addWidget(self.editor)
        body.setStretchFactor(0, 2)
        body.setStretchFactor(1, 1)
        body.setSizes([660, 390])
        outer.addWidget(body, 1)

        lifecycle = QHBoxLayout()
        lifecycle.addWidget(QLabel("After timeline"))
        self.end_action = QComboBox()
        self.end_action.addItems(("Torque on", "Torque off"))
        self.end_action.currentIndexChanged.connect(self._lifecycle_changed)
        lifecycle.addWidget(self.end_action)
        lifecycle.addSpacing(12)
        self.loop_toggle = QCheckBox("Loop timeline")
        self.loop_toggle.toggled.connect(self._lifecycle_changed)
        lifecycle.addWidget(self.loop_toggle)
        lifecycle.addStretch(1)
        outer.addLayout(lifecycle)

        playback = QHBoxLayout()
        self.smooth_playback = QCheckBox("Smooth timed playback")
        self.smooth_playback.setToolTip("Adjust each servo speed to the pose duration, up to its saved speed.")
        self.smooth_playback.toggled.connect(self._smooth_changed)
        playback.addWidget(self.smooth_playback)
        self.play_buttons = {}
        for label in ("▶  Play timeline", "Test selected step", "■  Stop timeline"):
            button = QPushButton(label)
            button.setEnabled(ros_manager is not None)
            button.setToolTip("Connect to ROS to play this timeline" if ros_manager is None else label)
            callback = {
                "▶  Play timeline": self.play_timeline,
                "Test selected step": self.test_selected_step,
                "■  Stop timeline": self.stop_timeline,
            }[label]
            button.clicked.connect(callback)
            self.play_buttons[label] = button
            playback.addWidget(button)
        playback.addStretch(1)
        outer.addLayout(playback)
        stop_hint = QLabel(
            "Stop halts later steps; motion or torque changes already sent may finish."
        )
        stop_hint.setObjectName("compactStatus")
        outer.addWidget(stop_hint)
        self.status_label = QLabel()
        self.status_label.setWordWrap(True)
        self.status_label.setFrameStyle(QLabel.Shape.Panel | QLabel.Shadow.Sunken)
        self.status_label.setMinimumHeight(42)
        outer.addWidget(self.status_label)
        self.playback.status_changed.connect(self.status_label.setText)
        self._redraw()

    def showEvent(self, event):
        """Update the file choices whenever this tab becomes visible."""
        super().showEvent(event)
        self.playback.limits.refresh()
        self.refresh_sources()
        window = self.window()
        # Detached tabs return to their owner on close; guard the owning workspace,
        # whose closure actually destroys the document, even while it is detached.
        while window.parentWidget() is not None:
            window = window.parentWidget().window()
        if window is not self._watched_window:
            if self._watched_window is not None:
                self._watched_window.removeEventFilter(self)
            self._watched_window = window if window is not self else None
            if self._watched_window is not None:
                self._watched_window.installEventFilter(self)

    def browse_directory(self, field):
        chosen = QFileDialog.getExistingDirectory(self, "Choose source folder", field.text())
        if chosen:
            field.setText(chosen)
            self.refresh_sources()

    def refresh_sources(self):
        """Refresh source selectors without changing the open document's reference base."""
        for picker, field in self.source_controls:
            selected = picker.currentData()
            picker.clear()
            directory = Path(field.text().strip()).expanduser()
            if not field.text().strip() or not directory.is_dir():
                picker.setPlaceholderText("Folder not found")
                continue
            try:
                paths = sorted(
                    (path for path in directory.glob("*.json") if path.is_file()),
                    key=lambda path: path.name.casefold(),
                )
            except OSError:
                picker.setPlaceholderText("Folder unavailable")
                continue
            picker.setPlaceholderText("No files found" if not paths else "")
            for path in paths:
                picker.addItem(path.name, str(path))
                picker.setItemData(picker.count() - 1, str(path), Qt.ItemDataRole.ToolTipRole)
            index = picker.findData(selected)
            picker.setCurrentIndex(index if index >= 0 else (0 if paths else -1))

    def _selected(self):
        item = self.steps_list.currentItem()
        return item.data(Qt.ItemDataRole.UserRole) if item is not None else None

    def _state_changed(self):
        name = self.document.path.name if self.document.path else "New timeline"
        state = "Unsaved changes" if self.document.dirty else "Saved" if self.document.path else "Ready"
        self.status_label.setText(f"{name} — {state}")
        self.status_label.setToolTip(str(self.document.path or ""))
        for row in range(self.steps_list.count()):
            item = self.steps_list.item(row)
            entry = item.data(Qt.ItemDataRole.UserRole)
            if entry is not None and entry[1] == "wait":
                seconds = self.document.steps[entry[0]]["seconds"]
                item.setText(f"Wait — {seconds:g} seconds")

    def _redraw(self, selected_index=None):
        self._syncing = True
        self.steps_list.clear()
        selected_row = None
        for row, (index, kind) in enumerate(timeline_rows(self.document.steps)):
            step = self.document.steps[index]
            if kind == "wait":
                text = f"Wait — {step['seconds']:g} seconds"
            else:
                name = Path(step["snapshot"]).stem
                text = f"Transition {index + 1} → {name}" if kind == "transition" else f"    Snapshot: {name}"
            item = QListWidgetItem(text, self.steps_list)
            item.setData(Qt.ItemDataRole.UserRole, (index, kind))
            if index == selected_index and selected_row is None:
                selected_row = row
        if not self.document.steps:
            item = QListWidgetItem("No steps yet", self.steps_list)
            item.setFlags(Qt.ItemFlag.NoItemFlags)
        else:
            self.steps_list.setCurrentRow(selected_row if selected_row is not None else 0)
        self.end_action.setCurrentIndex(0 if self.document.on_end == "on" else 1)
        self.loop_toggle.setChecked(self.document.loop)
        self.smooth_playback.setChecked(self.document.smooth_playback)
        self._syncing = False
        self._select_step()
        self._state_changed()

    def _select_step(self, _row=None):
        if self._syncing:
            return
        selected = self._selected()
        index = selected[0] if selected is not None else None
        self.play_buttons["▶  Play timeline"].setEnabled(
            self.playback.manager is not None and bool(self.document.steps)
        )
        for label, button in self.step_buttons.items():
            enabled = index is not None
            if label == "Up":
                enabled = enabled and index > 0
            elif label == "Down":
                enabled = enabled and index < len(self.document.steps) - 1
            button.setEnabled(enabled)
        self.play_buttons["Test selected step"].setEnabled(
            self.playback.manager is not None and selected is not None and selected[1] != "wait"
        )
        self._render_editor()

    def _render_editor(self):
        old = self.editor_scroll.takeWidget()
        if old is not None:
            old.deleteLater()
        content = QWidget()
        layout = QVBoxLayout(content)
        self.editor_scroll.setWidget(content)
        self.motion_controls = {}
        self.duration_picker = self.duration_seconds = self.wait_seconds = self.mode_picker = None
        self.editor.setTitle("Step settings")
        selected = self._selected()
        if selected is None:
            label = QLabel("Select a transition or wait to see its settings here.")
            label.setWordWrap(True)
            layout.addWidget(label)
            layout.addStretch(1)
            return
        index, kind = selected
        step = self.document.steps[index]
        if kind == "wait":
            self.editor.setTitle("Wait")
            self.wait_seconds = self._seconds_input(step["seconds"])
            self.wait_seconds.setAccessibleName("Wait duration in seconds")
            form = QFormLayout()
            form.addRow("Duration (seconds)", self.wait_seconds)
            layout.addLayout(form)
            self.wait_seconds.valueChanged.connect(
                lambda seconds: self._edit(self.document.update_wait, index, seconds)
            )
            layout.addWidget(QLabel("No servo commands are sent during a wait."))
            layout.addStretch(1)
            return
        try:
            snapshot = self.document.snapshot(index)
            if kind == "transition":
                duration = transition_duration(step)
                mode = step.get(
                    "control_mode", "Custom Control" if step.get("overrides") else "Global Control"
                )
                resolved = motion_settings(step, snapshot["positions"])
                groups = control_groups(mode, snapshot["positions"])
        except (OSError, ValueError, TypeError, KeyError) as error:
            label = QLabel(f"Could not read step: {error}")
            label.setWordWrap(True)
            layout.addWidget(label)
            layout.addStretch(1)
            return
        if kind == "snapshot":
            self.editor.setTitle(f"Snapshot: {snapshot['name']}")
            label = QLabel(
                f"{step['snapshot']}\n{len(snapshot['positions'])} servo targets\n\n"
                "Select the transition above this snapshot to edit its motion."
            )
            label.setWordWrap(True)
            layout.addWidget(label)
            mirror_picker = QComboBox()
            mirror_picker.addItems(MIRROR_MODES)
            mirror_picker.setAccessibleName("Snapshot mirror mode")
            mirror_picker.setCurrentText(step.get("mirror", "None"))
            mirror_picker.currentTextChanged.connect(
                lambda mode, i=index: self._edit(self.document.update_mirror, i, mode)
            )
            form = QFormLayout()
            form.addRow("Play mirrored", mirror_picker)
            layout.addLayout(form)
            hint = QLabel("Mirroring swaps corresponding leg targets across the selected axis.")
            hint.setWordWrap(True)
            hint.setObjectName("compactStatus")
            layout.addWidget(hint)
            layout.addStretch(1)
            return
        self.editor.setTitle(f"Transition {index + 1} → {snapshot['name']}")
        self.duration_picker = QComboBox()
        self.duration_picker.addItems(("Until reached", "X seconds"))
        self.duration_picker.setCurrentIndex(1 if duration["mode"] == "seconds" else 0)
        self.duration_seconds = self._seconds_input(duration["seconds"])
        self.duration_seconds.setAccessibleName("Transition duration in seconds")
        form = QFormLayout()
        form.addRow("Duration", self.duration_picker)
        # Keep the seconds editor unlabeled so hiding it also removes its entire row.
        form.addRow(self.duration_seconds)
        self.duration_seconds.setVisible(duration["mode"] == "seconds")
        layout.addLayout(form)
        self.duration_picker.currentIndexChanged.connect(self._duration_changed)
        self.duration_seconds.valueChanged.connect(self._duration_changed)

        self.mode_picker = QComboBox()
        self.mode_picker.addItems(CONTROL_MODES)
        self.mode_picker.setCurrentText(mode)
        form.addRow("Control mode", self.mode_picker)
        self.mode_picker.currentTextChanged.connect(self._mode_changed)
        explanation = QLabel(
            "Switching to finer controls copies current values.\n"
            "Combining different values uses their rounded average."
        )
        explanation.setWordWrap(True)
        explanation.setObjectName("compactStatus")
        layout.addWidget(explanation)
        for group, joints in groups.items():
            box = QGroupBox(group)
            controls = QFormLayout(box)
            self.motion_controls[group] = {}
            for key, label in (("speed", "Speed"), ("torque", "Torque strength")):
                initial = round(sum(resolved[joint][key] for joint in joints) / len(joints))
                row = QWidget()
                row_layout = QHBoxLayout(row)
                row_layout.setContentsMargins(0, 0, 0, 0)
                slider = QSlider(Qt.Orientation.Horizontal)
                value = QSpinBox()
                for widget in (slider, value):
                    widget.setRange(1, 1023)
                    widget.setValue(initial)
                    widget.setAccessibleName(f"{group} {label}")
                value.setFixedWidth(76)
                slider.valueChanged.connect(value.setValue)
                value.valueChanged.connect(slider.setValue)
                value.valueChanged.connect(
                    lambda number, i=index, m=mode, g=group, k=key:
                    self._edit(self.document.update_motion, i, m, g, k, number)
                )
                self.motion_controls[group][key] = value
                row_layout.addWidget(slider, 1)
                row_layout.addWidget(value)
                controls.addRow(label, row)
            layout.addWidget(box)
        layout.addStretch(1)

    @staticmethod
    def _seconds_input(value):
        spin = QDoubleSpinBox()
        spin.setDecimals(6)
        spin.setRange(0, 1e12)
        spin.setSingleStep(0.1)
        spin.setSuffix(" s")
        spin.setValue(value)
        spin.setKeyboardTracking(False)
        return spin

    def _edit(self, action, *args):
        try:
            action(*args)
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.status_label.setText(f"Could not edit timeline: {error}")
            return False
        self._state_changed()
        return True

    def _duration_changed(self, _value=None):
        selected = self._selected()
        if selected is None:
            return
        fixed = self.duration_picker.currentIndex() == 1
        self.duration_seconds.setVisible(fixed)
        self._edit(
            self.document.update_duration, selected[0],
            "seconds" if fixed else "until_reached", self.duration_seconds.value(),
        )

    def _mode_changed(self, mode):
        selected = self._selected()
        if selected is not None and self._edit(self.document.set_control_mode, selected[0], mode):
            self._render_editor()

    def _lifecycle_changed(self, _value=None):
        if not self._syncing:
            self._edit(
                self.document.set_lifecycle,
                "on" if self.end_action.currentIndex() == 0 else "off",
                self.loop_toggle.isChecked(),
            )

    def _smooth_changed(self, enabled):
        if not self._syncing:
            self._edit(self.document.set_smooth_playback, enabled)

    def play_timeline(self):
        try:
            self.playback.start_steps(
                self, self.document.steps, repeat=self.document.loop,
                smooth=self.document.smooth_playback,
            )
        except (OSError, RuntimeError, ValueError, TypeError, KeyError) as error:
            self.status_label.setText(f"Could not play timeline: {error}")
            return False
        return True

    def test_selected_step(self):
        selected = self._selected()
        if selected is None or selected[1] == "wait":
            return False
        try:
            self.playback.start_steps(self, [self.document.steps[selected[0]]])
        except (OSError, RuntimeError, ValueError, TypeError, KeyError) as error:
            self.status_label.setText(f"Could not test step: {error}")
            return False
        return True

    def stop_timeline(self):
        self.playback.stop("Timeline stopped")

    def load_source(self, source):
        path = self.source_controls[source][0].currentData()
        if not path:
            self.status_label.setText("Choose a source file first")
            return False
        if source == 0:
            return self.load_timeline(path)
        try:
            index = self.document.add_snapshot(path)
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.status_label.setText(f"Snapshot rejected: {error}")
            return False
        self._redraw(index)
        return True

    def add_wait(self):
        selected = self._selected()
        index = selected[0] + 1 if selected is not None else len(self.document.steps)
        index = self.document.add_wait(index)
        self._redraw(index)

    def move_step(self, delta):
        selected = self._selected()
        if selected is not None:
            index = self.document.presenter.move(selected[0], delta)
            if index is not None:
                self._redraw(index)

    def duplicate_step(self):
        selected = self._selected()
        if selected is not None:
            self._redraw(self.document.presenter.duplicate(selected[0]))

    def remove_step(self):
        selected = self._selected()
        if selected is not None:
            self._redraw(self.document.presenter.remove(selected[0]))

    def _commit_pending_edits(self):
        # Duration text is committed on focus loss, which can follow a close event.
        for spin in (self.wait_seconds, self.duration_seconds):
            if spin is not None:
                spin.interpretText()

    def _confirm_discard(self):
        self._commit_pending_edits()
        if not self.document.dirty:
            return True
        choice = QMessageBox.question(
            self, "Unsaved timeline", "Save changes to this timeline?",
            QMessageBox.StandardButton.Save | QMessageBox.StandardButton.Discard
            | QMessageBox.StandardButton.Cancel,
            QMessageBox.StandardButton.Cancel,
        )
        if choice == QMessageBox.StandardButton.Save:
            return self.save_timeline()
        return choice == QMessageBox.StandardButton.Discard

    def new_timeline(self):
        if not self._confirm_discard():
            return False
        self.document.new(self.source_controls[1][1].text().strip())
        self._redraw()
        return True

    def load_timeline(self, path=None):
        if path is None:
            path, _filter = QFileDialog.getOpenFileName(
                self, "Load timeline", self.source_controls[0][1].text(), "Timeline JSON (*.json)"
            )
        if not path or not self._confirm_discard():
            return False
        try:
            self.document.load(path, self.source_controls[1][1].text().strip())
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.status_label.setText(f"Could not load timeline: {error}")
            return False
        self._redraw()
        return True

    def save_timeline(self, path=None):
        self._commit_pending_edits()
        if path is None:
            path = self.document.path
        if path is None:
            path, _filter = QFileDialog.getSaveFileName(
                self, "Save timeline",
                str(Path(self.source_controls[0][1].text()) / "timeline.json"),
                "Timeline JSON (*.json)",
            )
        if not path:
            return False
        if not Path(path).suffix:
            path = f"{path}.json"
        try:
            folder = self.source_controls[1][1].text().strip()
            directory = (
                Path(folder).expanduser().resolve() if folder else self.document.snapshot_directory
            )
            self.document.save(
                path, absolute_references=directory != self.document.snapshot_directory
            )
        except (OSError, ValueError, TypeError, KeyError) as error:
            self.status_label.setText(f"Could not save timeline: {error}")
            return False
        self.refresh_sources()
        self._state_changed()
        return True

    def closeEvent(self, event):
        self.playback.close()
        if self._confirm_discard():
            super().closeEvent(event)
        else:
            event.ignore()

    def eventFilter(self, watched, event):
        # Qt closes the main window without sending closeEvent to each child tab.
        if watched is self._watched_window and event.type() == QEvent.Type.Close:
            if not self._confirm_discard():
                event.ignore()
                return True
            self.playback.close()
        return super().eventFilter(watched, event)


def create_tab(ros_manager=None):
    """Build the offline TimeLine editor."""
    return TimeLineTab(ros_manager=ros_manager)
