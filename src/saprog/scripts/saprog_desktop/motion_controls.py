"""Common speed and strength controls for position commands."""

import math

SPEED_UNIT = 0.111 * 2.0 * math.pi / 60.0


def add_slider(parent, text, variable, minimum, command=None):
    from tkinter import ttk

    row = ttk.Frame(parent)
    row.pack(fill="x", pady=4)
    ttk.Label(row, text=text, width=16).pack(side="left")
    value_label = ttk.Label(row, width=12, anchor="e")

    def changed(value):
        rounded = max(minimum, min(1023, round(float(value))))
        variable.set(rounded)
        value_label.configure(text=f"{rounded} / 1023")
        if command is not None:
            command(str(rounded))

    slider = ttk.Scale(row, from_=minimum, to=1023, variable=variable, command=changed)
    slider.pack(side="left", fill="x", expand=True, padx=8)
    value_label.pack(side="left")
    value_label.configure(text=f"{variable.get()} / 1023")
    return slider


def set_motion(message, speed, strength):
    message.velocity = [float(speed) * SPEED_UNIT] * len(message.name)
    message.effort = [float(strength)] * len(message.name)
