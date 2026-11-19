"""Shared presentation helpers for the desktop tools."""

# Shared desktop palette. Keep view-specific drawing colors here as well so
# the UI has one source of truth for color choices.
BACKGROUND = "#141b26"
FOREGROUND = "#e5edf7"
MUTED = "#a5b5ca"
DISABLED = "#778394"
DISABLED_BORDER = "#303c4d"
PANEL = "#1d2939"
BUTTON = "#26364b"
BUTTON_HOVER = "#354c68"
BUTTON_BORDER = "#435873"
BORDER = "#34465d"
FOCUS = "#498bd1"
ACCENT = "#245c96"
ACCENT_HOVER = "#3478bb"
ACCENT_LIGHT = "#70b5f5"
ACCENT_DARK = "#19436e"
CAPTION = "#91c4ff"
WHITE = "#ffffff"
FIX = "#86e3ad"
FIX_DISABLED = "#718f7d"
RELEASE = "#ff9c96"
RELEASE_DISABLED = "#a47d7b"
TAB = "#202a39"
GRID = "#2b394d"
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


def apply_style(root):
    from tkinter import font, ttk

    style = ttk.Style(root)
    if "clam" in style.theme_names():
        style.theme_use("clam")
    family = "Segoe UI" if "Segoe UI" in font.families(root) else "DejaVu Sans"
    root.option_add("*Font", (family, 10))
    root.configure(background=BACKGROUND)
    style.configure(".", font=(family, 10), background=BACKGROUND, foreground=FOREGROUND)
    style.configure("TFrame", background=BACKGROUND)
    style.configure("TLabel", background=BACKGROUND)
    style.configure("Title.TLabel", font=(family, 20, "bold"))
    style.configure("Muted.TLabel", foreground=MUTED)
    style.configure("TLabelframe", borderwidth=1, relief="solid")
    style.configure("TLabelframe.Label", font=(family, 11, "bold"), foreground=CAPTION)
    style.configure(
        "TNotebook",
        background=BACKGROUND,
        bordercolor=BORDER,
        lightcolor=BACKGROUND,
        darkcolor=BORDER,
    )
    style.configure(
        "TNotebook.Tab",
        background=TAB,
        foreground=MUTED,
        bordercolor=BORDER,
        lightcolor=BORDER,
        darkcolor=BORDER,
    )
    style.map(
        "TNotebook.Tab",
        background=[("disabled", TAB), ("selected", BUTTON), ("active", BUTTON_HOVER)],
        foreground=[("disabled", DISABLED), ("selected", FOREGROUND), ("active", FOREGROUND)],
        bordercolor=[("selected", FOCUS), ("active", FOCUS)],
        lightcolor=[("selected", BUTTON), ("active", BUTTON_HOVER)],
        darkcolor=[("selected", BUTTON), ("active", BUTTON_HOVER)],
    )
    style.configure(
        "TButton",
        padding=(10, 7),
        background=BUTTON,
        foreground=FOREGROUND,
        borderwidth=1,
        bordercolor=BUTTON_BORDER,
        lightcolor=BORDER,
        darkcolor=PANEL,
        focuscolor=FOCUS,
    )
    style.configure("Icon.TButton", padding=(3, 3), anchor="center", width=3, font=(family, 12))
    style.configure("Tooltip.TLabel", background=BUTTON, foreground=FOREGROUND, relief="solid", borderwidth=1)
    style.map(
        "TButton",
        background=[("disabled", TAB), ("pressed", PANEL), ("active", BUTTON_HOVER)],
        foreground=[("disabled", DISABLED), ("!disabled", FOREGROUND)],
        bordercolor=[("disabled", DISABLED_BORDER), ("pressed", FOCUS), ("active", FOCUS)],
        lightcolor=[("disabled", TAB), ("pressed", PANEL), ("active", BUTTON_HOVER)],
        darkcolor=[("disabled", TAB), ("pressed", PANEL), ("active", BUTTON_HOVER)],
    )
    style.configure(
        "Accent.TButton",
        background=ACCENT,
        foreground=WHITE,
        bordercolor=ACCENT_HOVER,
        lightcolor=ACCENT_HOVER,
        darkcolor=ACCENT_DARK,
    )
    style.map(
        "Accent.TButton",
        background=[("disabled", TAB), ("pressed", ACCENT_DARK), ("active", ACCENT_HOVER)],
        foreground=[("disabled", DISABLED), ("!disabled", WHITE)],
        bordercolor=[("disabled", DISABLED_BORDER), ("pressed", ACCENT_HOVER), ("active", ACCENT_LIGHT)],
        lightcolor=[("disabled", TAB), ("pressed", ACCENT_DARK), ("active", ACCENT_HOVER)],
        darkcolor=[("disabled", TAB), ("pressed", ACCENT_DARK), ("active", ACCENT)],
    )
    style.configure("Fix.TButton", foreground=FIX)
    style.map("Fix.TButton", foreground=[("disabled", FIX_DISABLED), ("!disabled", FIX)])
    style.configure("Release.TButton", foreground=RELEASE)
    style.map("Release.TButton", foreground=[("disabled", RELEASE_DISABLED), ("!disabled", RELEASE)])
    style.configure("TEntry", padding=5, fieldbackground=PANEL, insertcolor=FOREGROUND)
    style.configure("TCombobox", padding=5, fieldbackground=PANEL, arrowcolor=FOREGROUND)
    style.map(
        "TCombobox",
        fieldbackground=[("readonly", PANEL), ("disabled", TAB)],
        foreground=[("disabled", DISABLED), ("readonly", FOREGROUND)],
        bordercolor=[("disabled", DISABLED_BORDER), ("focus", FOCUS)],
    )
    style.configure("TCheckbutton", padding=(0, 5))
    style.map(
        "TCheckbutton",
        background=[("active", BACKGROUND)],
        foreground=[("disabled", DISABLED)],
        indicatorbackground=[
            ("disabled", TAB),
            ("selected", FOCUS),
            ("!selected", BORDER),
        ],
        indicatorforeground=[("disabled", DISABLED), ("selected", WHITE)],
    )
    style.configure("Horizontal.TScale", background=FOCUS, troughcolor=BUTTON)
    style.configure(
        "Vertical.TScrollbar", background=BORDER, troughcolor=BACKGROUND, arrowcolor=FOREGROUND
    )
    root.option_add("*TCombobox*Listbox.background", PANEL)
    root.option_add("*TCombobox*Listbox.foreground", FOREGROUND)
    root.option_add("*TCombobox*Listbox.selectBackground", ACCENT)
    root.option_add("*Text.background", PANEL)
    root.option_add("*Text.foreground", FOREGROUND)
    root.option_add("*Text.selectBackground", ACCENT)
    root.option_add("*Text.selectForeground", WHITE)
    root.option_add("*Text.relief", "flat")
    root.option_add("*Text.padX", 10)
    root.option_add("*Text.padY", 8)


def configure_changed(widget, **options):
    previous = getattr(widget, "_display_options", {})
    changed = {key: value for key, value in options.items() if previous.get(key) != value}
    if changed:
        widget.configure(**changed)
        previous.update(changed)
        widget._display_options = previous


def status_box(parent, variable, *, tk_module=None, **options):
    """Build a selectable, read-only status display shared by desktop windows.

    Keep the existing text when a repeated ROS status arrives so selections
    survive polling. Ctrl+C copies the selection, or the whole message.
    """
    if tk_module is None:
        import tkinter as tk_module

    widget = tk_module.Text(parent, wrap="word", state="disabled", **options)
    displayed = None

    def refresh(*_args):
        nonlocal displayed
        text = variable.get()
        if text != displayed:
            displayed = text
            widget.configure(state="normal")
            widget.delete("1.0", "end")
            widget.insert("1.0", text)
            widget.configure(state="disabled")

    def select_all(_event):
        widget.tag_add("sel", "1.0", "end-1c")
        return "break"

    def copy(_event):
        try:
            text = widget.get("sel.first", "sel.last")
        except tk_module.TclError:
            text = widget.get("1.0", "end-1c")
        widget.clipboard_clear()
        widget.clipboard_append(text)
        return "break"

    trace = variable.trace_add("write", refresh)
    # The variable can outlive a destroyed window; detach its Tcl callback.
    widget.bind("<Destroy>", lambda _event: variable.trace_remove("write", trace))
    for key in ("<Control-a>", "<Control-A>"):
        widget.bind(key, select_all)
    for key in ("<Control-c>", "<Control-C>"):
        widget.bind(key, copy)
    refresh()
    return widget
