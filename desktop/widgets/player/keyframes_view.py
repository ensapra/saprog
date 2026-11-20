"""UI for browsing and selecting saved keyframes."""

from pathlib import Path

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import (
    QFileDialog,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QVBoxLayout,
    QWidget,
)


def default_snapshot_directory():
    """Return the workspace-local snapshots folder."""
    return str(Path(__file__).resolve().parents[3] / "snapshots")


class KeyframesView(QWidget):
    """Browse snapshot files and request loading or playing a keyframe."""

    load_requested = Signal(str)
    play_requested = Signal(str)

    def __init__(self, parent=None, *, directory=None):
        super().__init__(parent)
        self._directory = directory or default_snapshot_directory()

        layout = QVBoxLayout(self)
        folder_row = QHBoxLayout()
        folder_row.addWidget(QLabel("Keyframes"))
        self.directory_input = QLineEdit(self._directory)
        self.directory_input.setClearButtonEnabled(True)
        self.directory_input.setToolTip("Folder containing keyframe JSON files")
        self.directory_input.editingFinished.connect(self.refresh)
        folder_row.addWidget(self.directory_input, stretch=1)
        browse_button = QPushButton("Browse…")
        browse_button.clicked.connect(self.browse_directory)
        folder_row.addWidget(browse_button)
        layout.addLayout(folder_row)

        self.keyframe_list = QListWidget()
        self.keyframe_list.setSelectionMode(QListWidget.SelectionMode.SingleSelection)
        layout.addWidget(self.keyframe_list, stretch=1)
        self.status_label = QLabel()
        self.status_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignVCenter)
        layout.addWidget(self.status_label)
        self.refresh()

    @property
    def directory(self):
        return self.directory_input.text().strip()

    def browse_directory(self):
        chosen = QFileDialog.getExistingDirectory(
            self, "Choose keyframe folder", self.directory or default_snapshot_directory()
        )
        if chosen:
            self.directory_input.setText(chosen)
            self.refresh()

    def refresh(self):
        """Reload JSON keyframes from the entered folder."""
        directory = Path(self.directory).expanduser()
        self.keyframe_list.clear()
        if not directory.is_dir():
            self.status_label.setText("Folder not found")
            return

        paths = sorted(directory.glob("*.json"), key=lambda path: path.name.casefold())
        for path in paths:
            item = QListWidgetItem(self.keyframe_list)
            row = QWidget(self.keyframe_list)
            row_layout = QHBoxLayout(row)
            row_layout.setContentsMargins(6, 3, 6, 3)
            name = QLabel(path.stem)
            name.setToolTip(str(path))
            row_layout.addWidget(name, stretch=1)
            load_button = QPushButton("Load")
            load_button.clicked.connect(
                lambda _checked=False, p=path: self.load_requested.emit(str(p))
            )
            row_layout.addWidget(load_button)
            play_button = QPushButton("Play")
            play_button.clicked.connect(
                lambda _checked=False, p=path: self.play_requested.emit(str(p))
            )
            row_layout.addWidget(play_button)
            item.setSizeHint(row.sizeHint())
            self.keyframe_list.setItemWidget(item, row)
            item.setData(Qt.ItemDataRole.UserRole, str(path))

        count = len(paths)
        self.status_label.setText(f"{count} keyframe{'s' if count != 1 else ''} found")


__all__ = ["KeyframesView", "default_snapshot_directory"]
