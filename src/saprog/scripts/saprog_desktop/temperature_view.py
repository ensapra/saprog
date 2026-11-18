"""Canvas drawing for temperature history plots."""

import math

from saprog_desktop.desktop_style import FOREGROUND, configure_changed
from saprog_desktop.temperature_model import STALE_SECONDS


def draw_temperature(canvas, history, colors, labels, now):
    canvas.delete("all")
    visible, fresh = history.visible(now)
    for name, points in history.points.items():
        recent = bool(points) and now - points[-1][0] <= STALE_SECONDS
        configure_changed(
            labels[name],
            text=(f"{points[-1][1]:.0f} °C" + (" · stale" if not recent else ""))
            if points
            else "waiting",
        )
    status = f"{len(visible)}/{len(history.points)} selected · {fresh} live"
    left, top = 65, 25
    right = max(left + 1, canvas.winfo_width() - 20)
    bottom = max(top + 1, canvas.winfo_height() - 45)
    values = [value for _name, points in visible for _stamp, value in points]
    low = math.floor((min(values) - 5) / 5) * 5 if values else 15
    high = max(low + 10, math.ceil((max(values) + 5) / 5) * 5) if values else 50
    for index in range(6):
        y = bottom - index * (bottom - top) / 5
        canvas.create_line(left, y, right, y, fill="#2b394d")
        canvas.create_text(
            left - 10,
            y,
            text=f"{low + index * (high - low) / 5:.0f}",
            anchor="e",
            fill=FOREGROUND,
        )
        x = left + index * (right - left) / 5
        label = f"{-history.window_seconds + index * history.window_seconds / 5:g}s" if index < 5 else "now"
        canvas.create_text(x, bottom + 18, text=label, fill=FOREGROUND)
    canvas.create_text(10, top, text="°C", anchor="w", fill=FOREGROUND)
    cutoff = now - history.window_seconds
    for name, points in visible:
        segment, previous = [], None
        for stamp, value in points:
            if previous is not None and stamp - previous > STALE_SECONDS:
                _draw_segment(canvas, segment, colors[name])
                segment = []
            segment.extend(
                (
                    left + (stamp - cutoff) / history.window_seconds * (right - left),
                    bottom - (value - low) / (high - low) * (bottom - top),
                )
            )
            previous = stamp
        _draw_segment(canvas, segment, colors[name])
    if not values:
        canvas.create_text(
            (left + right) / 2,
            (top + bottom) / 2,
            text="Waiting for selected servo temperatures" if visible else "Select servos to graph",
            fill=FOREGROUND,
        )
    return status


def _draw_segment(canvas, coordinates, color):
    if len(coordinates) >= 4:
        canvas.create_line(*coordinates, fill=color, width=2)
    elif coordinates:
        x, y = coordinates
        canvas.create_oval(x - 2, y - 2, x + 2, y + 2, fill=color, outline=color)
