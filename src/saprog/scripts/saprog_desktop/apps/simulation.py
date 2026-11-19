"""Standalone simulation controls, requiring no ROS or hardware backend."""

import tkinter as tk
from pathlib import Path
from tkinter import ttk

from saprog_desktop.desktop_style import apply_style
from saprog_desktop.views.simulation import SimulationView


def main():
    root = tk.Tk()
    apply_style(root)
    root.title("Saprog - Simulation")
    root.geometry("900x650")
    root.minsize(700, 500)
    content = ttk.Frame(root, padding=16)
    content.pack(fill="both", expand=True)
    view = SimulationView(content, Path(__file__).resolve().parents[5])

    def close():
        view.close()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", close)
    # Home terminates owned launcher processes on close; honor that as a UI close.
    import signal
    signal.signal(signal.SIGTERM, lambda *_: root.after(0, close))
    root.mainloop()


if __name__ == "__main__":
    main()
