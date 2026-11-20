"""Compose the shared Static Animator controls and its two views."""

import json

from PySide6.QtCore import Qt, Signal
from PySide6.QtWidgets import QSplitter, QVBoxLayout, QWidget

from .creator_view import CreatorView
from .keyframes_view import KeyframesView


class StaticAnimatorWidget(QWidget):
    """Place Creator and Keyframes side by side under shared motion controls."""

    load_requested = Signal(str)
    play_requested = Signal(str)

    def __init__(self, parent=None, *, directory=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        self.splitter = QSplitter(Qt.Orientation.Horizontal, self)
        self.creator_view = CreatorView(self.splitter)
        self.keyframes_view = KeyframesView(self.splitter, directory=directory)
        self.splitter.addWidget(self.creator_view)
        self.splitter.addWidget(self.keyframes_view)
        self.splitter.setStretchFactor(0, 2)
        self.splitter.setStretchFactor(1, 1)
        self.splitter.setSizes([660, 330])
        layout.addWidget(self.splitter)

        self.keyframes_view.load_requested.connect(self._load_keyframe)
        self.keyframes_view.play_requested.connect(self.play_requested)
        self.creator_view.save_requested.connect(self._save_keyframe)

    def _load_keyframe(self, path):
        try:
            count = self.creator_view.load_keyframe_fields(path)
        except (OSError, json.JSONDecodeError, ValueError) as error:
            self.keyframes_view.status_label.setText(f"Could not load keyframe: {error}")
            return
        self.keyframes_view.status_label.setText(f"Loaded {count} servo targets")
        self.load_requested.emit(path)

    def _save_keyframe(self):
        try:
            path = self.creator_view.save_targets(self.keyframes_view.directory)
        except (OSError, ValueError) as error:
            self.keyframes_view.status_label.setText(f"Could not save keyframe: {error}")
            return
        if path is None:
            return
        self.keyframes_view.refresh()
        self.keyframes_view.status_label.setText(f"Saved {path.name}")


__all__ = ["CreatorView", "KeyframesView", "StaticAnimatorWidget"]
