"""Shared Qt presentation helpers and palette for the standalone desktop."""

BACKGROUND = "#141b26"
FOREGROUND = "#e5edf7"
MUTED = "#a5b5ca"
DISABLED = "#778394"
PANEL = "#1d2939"
BUTTON = "#26364b"
BUTTON_HOVER = "#354c68"
BORDER = "#34465d"
FOCUS = "#498bd1"
ACCENT = "#245c96"
ACCENT_HOVER = "#3478bb"
ACCENT_LIGHT = "#70b5f5"
WHITE = "#ffffff"
GRID = "#2b394d"

# IMU scene colors, carried over from the previous desktop visualization.
ROBOT_GROUND = "#26384a"
ROBOT_GROUND_EDGE = "#64748b"
ROBOT_GROUND_AXIS = "#71869c"
ROBOT_GROUND_GRID = "#3b5065"
ROBOT_SHADOW = "#172431"
ROBOT_FACE_FRONT = "#315276"
ROBOT_FACE_TOP = "#4b8ac2"
ROBOT_FACE_LEFT = "#3a6996"
ROBOT_FACE_RIGHT = "#27618d"
ROBOT_FACE_BOTTOM = "#254b70"
ROBOT_FACE_BACK = "#3f78a8"
ROBOT_EDGE = "#a8d8ff"
ROBOT_TEXT = "#d6e7f7"


def apply_style(app):
    """Apply the shared dark palette to a QApplication."""
    from PySide6.QtGui import QColor, QPalette

    app.setStyle("Fusion")
    palette = QPalette()
    palette.setColor(QPalette.ColorRole.Window, QColor(BACKGROUND))
    palette.setColor(QPalette.ColorRole.WindowText, QColor(FOREGROUND))
    palette.setColor(QPalette.ColorRole.Base, QColor(PANEL))
    palette.setColor(QPalette.ColorRole.AlternateBase, QColor(BACKGROUND))
    palette.setColor(QPalette.ColorRole.ToolTipBase, QColor(BUTTON))
    palette.setColor(QPalette.ColorRole.ToolTipText, QColor(FOREGROUND))
    palette.setColor(QPalette.ColorRole.Text, QColor(FOREGROUND))
    palette.setColor(QPalette.ColorRole.Button, QColor(BUTTON))
    palette.setColor(QPalette.ColorRole.ButtonText, QColor(FOREGROUND))
    palette.setColor(QPalette.ColorRole.BrightText, QColor(WHITE))
    palette.setColor(QPalette.ColorRole.Highlight, QColor(ACCENT))
    palette.setColor(QPalette.ColorRole.HighlightedText, QColor(WHITE))
    palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.WindowText, QColor(DISABLED))
    palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, QColor(DISABLED))
    palette.setColor(QPalette.ColorGroup.Disabled, QPalette.ColorRole.ButtonText, QColor(DISABLED))
    app.setPalette(palette)
    app.setStyleSheet(
        f"""
        QMainWindow, QWidget {{ background: {BACKGROUND}; color: {FOREGROUND}; }}
        QLabel#title {{ font-size: 20pt; font-weight: bold; color: {ACCENT_LIGHT}; }}
        QPushButton {{
            background: {BUTTON}; color: {FOREGROUND}; border: 2px solid {BORDER};
            border-radius: 0; padding: 6px 10px;
        }}
        QPushButton:hover {{ background: {BUTTON_HOVER}; border-color: {FOCUS}; }}
        QPushButton:focus {{ border-color: {ACCENT_LIGHT}; }}
        QPushButton:pressed {{ background: {ACCENT}; border-color: {ACCENT_LIGHT}; }}
        QPushButton:disabled {{ color: {DISABLED}; background: {BACKGROUND}; }}
        QPushButton#compactAction {{ padding: 2px 6px; font-size: 9pt; border-width: 1px; }}
        QLabel#compactStatus {{ color: {MUTED}; font-size: 9pt; }}
        QLineEdit, QTextEdit, QPlainTextEdit, QComboBox {{
            background: {PANEL}; color: {FOREGROUND}; border: 2px solid {BORDER};
            border-radius: 0; padding: 5px;
        }}
        QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus, QComboBox:focus {{
            border-color: {ACCENT_LIGHT};
        }}
        QToolTip {{ background: {BUTTON}; color: {FOREGROUND}; border: 1px solid {ACCENT_LIGHT}; }}
        QTabWidget::pane {{ border: 2px solid {BORDER}; top: -1px; }}
        QTabBar::tab {{
            background: #202a39; color: {MUTED}; border: 1px solid {BORDER};
            padding: 7px 12px; margin-right: 3px;
        }}
        QTabBar::tab:hover {{
            background: {BUTTON_HOVER}; color: {FOREGROUND}; border-color: {FOCUS};
        }}
        QTabBar::tab:selected {{
            background: {BUTTON}; color: {FOREGROUND};
            border-top: 3px solid {ACCENT_LIGHT}; border-bottom-color: {BUTTON};
        }}
        QTabBar::tab:disabled {{ color: {DISABLED}; }}
        QGroupBox {{ border: 2px solid {BORDER}; margin-top: 10px; padding: 8px; }}
        QGroupBox::title {{ subcontrol-origin: margin; left: 8px; padding: 0 4px; color: {ACCENT_LIGHT}; }}
        QScrollBar:vertical {{ background: {BACKGROUND}; width: 12px; margin: 0; }}
        QScrollBar::handle:vertical {{ background: {BORDER}; min-height: 24px; }}
        QScrollBar::handle:vertical:hover {{ background: {FOCUS}; }}
        QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{ height: 0; }}
        """
    )
