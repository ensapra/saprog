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
    style.configure("TButton", padding=(10, 7), background="#26364b", borderwidth=1)
    style.map("TButton", background=[("disabled", "#202a39"), ("active", "#354c68")],
              foreground=[("disabled", "#778394")])
    style.configure("Accent.TButton", background="#245c96", foreground="#ffffff")
    style.map("Accent.TButton", background=[("disabled", "#202a39"), ("active", "#3478bb")],
              foreground=[("disabled", "#778394"), ("!disabled", "#ffffff")])
    style.configure("Fix.TButton", foreground="#ff9c96")
    style.configure("Release.TButton", foreground="#86e3ad")
    style.configure("TEntry", padding=5, fieldbackground="#1d2939", insertcolor=FOREGROUND)
    style.configure("TCombobox", padding=5, fieldbackground="#1d2939", arrowcolor=FOREGROUND)
    style.map("TCombobox", fieldbackground=[("readonly", "#1d2939"), ("disabled", "#202a39")],
              foreground=[("disabled", "#778394"), ("readonly", FOREGROUND)])
    style.configure("TCheckbutton", padding=(0, 5))
    style.map("TCheckbutton", background=[("active", BACKGROUND)],
              indicatorbackground=[("selected", "#498bd1"), ("!selected", "#34465d")])
    style.configure("Horizontal.TScale", background="#498bd1", troughcolor="#26364b")
    style.configure("Vertical.TScrollbar", background="#34465d", troughcolor=BACKGROUND,
                    arrowcolor=FOREGROUND)
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
