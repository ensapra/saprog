"""Shared presentation helpers for the desktop tools."""

BACKGROUND = "#141b26"
FOREGROUND = "#e5edf7"


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
    style.configure("Muted.TLabel", foreground="#a5b5ca")
    style.configure("TLabelframe", borderwidth=1, relief="solid")
    style.configure("TLabelframe.Label", font=(family, 11, "bold"), foreground="#91c4ff")
    style.configure(
        "TNotebook",
        background=BACKGROUND,
        bordercolor="#34465d",
        lightcolor=BACKGROUND,
        darkcolor="#34465d",
    )
    style.configure(
        "TNotebook.Tab",
        background="#202a39",
        foreground="#a5b5ca",
        bordercolor="#34465d",
        lightcolor="#34465d",
        darkcolor="#34465d",
    )
    style.map(
        "TNotebook.Tab",
        background=[("disabled", "#202a39"), ("selected", "#26364b"), ("active", "#354c68")],
        foreground=[("disabled", "#778394"), ("selected", FOREGROUND), ("active", FOREGROUND)],
        bordercolor=[("selected", "#498bd1"), ("active", "#498bd1")],
        lightcolor=[("selected", "#26364b"), ("active", "#354c68")],
        darkcolor=[("selected", "#26364b"), ("active", "#354c68")],
    )
    style.configure(
        "TButton",
        padding=(10, 7),
        background="#26364b",
        foreground=FOREGROUND,
        borderwidth=1,
        bordercolor="#435873",
        lightcolor="#34465d",
        darkcolor="#1d2939",
        focuscolor="#498bd1",
    )
    style.map(
        "TButton",
        background=[("disabled", "#202a39"), ("pressed", "#1d2939"), ("active", "#354c68")],
        foreground=[("disabled", "#778394"), ("!disabled", FOREGROUND)],
        bordercolor=[("disabled", "#303c4d"), ("pressed", "#498bd1"), ("active", "#498bd1")],
        lightcolor=[("disabled", "#202a39"), ("pressed", "#1d2939"), ("active", "#354c68")],
        darkcolor=[("disabled", "#202a39"), ("pressed", "#1d2939"), ("active", "#354c68")],
    )
    style.configure(
        "Accent.TButton",
        background="#245c96",
        foreground="#ffffff",
        bordercolor="#3478bb",
        lightcolor="#3478bb",
        darkcolor="#19436e",
    )
    style.map(
        "Accent.TButton",
        background=[("disabled", "#202a39"), ("pressed", "#19436e"), ("active", "#3478bb")],
        foreground=[("disabled", "#778394"), ("!disabled", "#ffffff")],
        bordercolor=[("disabled", "#303c4d"), ("pressed", "#3478bb"), ("active", "#70b5f5")],
        lightcolor=[("disabled", "#202a39"), ("pressed", "#19436e"), ("active", "#3478bb")],
        darkcolor=[("disabled", "#202a39"), ("pressed", "#19436e"), ("active", "#245c96")],
    )
    style.configure("Fix.TButton", foreground="#ff9c96")
    style.map("Fix.TButton", foreground=[("disabled", "#a47d7b"), ("!disabled", "#ff9c96")])
    style.configure("Release.TButton", foreground="#86e3ad")
    style.map("Release.TButton", foreground=[("disabled", "#718f7d"), ("!disabled", "#86e3ad")])
    style.configure("TEntry", padding=5, fieldbackground="#1d2939", insertcolor=FOREGROUND)
    style.configure("TCombobox", padding=5, fieldbackground="#1d2939", arrowcolor=FOREGROUND)
    style.map(
        "TCombobox",
        fieldbackground=[("readonly", "#1d2939"), ("disabled", "#202a39")],
        foreground=[("disabled", "#778394"), ("readonly", FOREGROUND)],
        bordercolor=[("disabled", "#303c4d"), ("focus", "#498bd1")],
    )
    style.configure("TCheckbutton", padding=(0, 5))
    style.map(
        "TCheckbutton",
        background=[("active", BACKGROUND)],
        foreground=[("disabled", "#778394")],
        indicatorbackground=[
            ("disabled", "#202a39"),
            ("selected", "#498bd1"),
            ("!selected", "#34465d"),
        ],
        indicatorforeground=[("disabled", "#778394"), ("selected", "#ffffff")],
    )
    style.configure("Horizontal.TScale", background="#498bd1", troughcolor="#26364b")
    style.configure(
        "Vertical.TScrollbar", background="#34465d", troughcolor=BACKGROUND, arrowcolor=FOREGROUND
    )
    root.option_add("*TCombobox*Listbox.background", "#1d2939")
    root.option_add("*TCombobox*Listbox.foreground", FOREGROUND)
    root.option_add("*TCombobox*Listbox.selectBackground", "#245c96")
    root.option_add("*Text.background", "#1d2939")
    root.option_add("*Text.foreground", FOREGROUND)
    root.option_add("*Text.selectBackground", "#245c96")
    root.option_add("*Text.selectForeground", "#ffffff")
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
