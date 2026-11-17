#!/usr/bin/env python3
"""Discover and launch the desktop tools in the workspace tabs directory."""
import os
from pathlib import Path
import signal
import subprocess
import tkinter as tk
from tkinter import messagebox, ttk
from desktop_style import apply_style


WORKSPACE = Path(__file__).resolve().parents[3]
TABS = WORKSPACE / "tabs"


class Home:
    def __init__(self, root):
        self.root = root
        self.children = {}
        self.backend = None
        self._scripts = None
        apply_style(root)
        root.title("Saprog home")
        root.geometry("420x420")
        outer = ttk.Frame(root, padding=16)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="Saprog tools", style="Title.TLabel").pack(anchor="w")
        ttk.Label(outer, text="Choose a window to open", style="Muted.TLabel").pack(anchor="w", pady=(2, 16))
        self.buttons = ttk.Frame(outer)
        self.buttons.pack(fill="both", expand=True)
        controls = ttk.Frame(outer)
        controls.pack(fill="x", pady=(8, 0))
        ttk.Button(controls, text="Refresh", command=self.refresh).pack(side="left")
        self.status = tk.StringVar(value="")
        ttk.Label(outer, textvariable=self.status, wraplength=380).pack(anchor="w", pady=(10, 0))
        root.protocol("WM_DELETE_WINDOW", self.close)
        self.refresh()
        self.root.after(1000, self.watch)

    @staticmethod
    def title(script):
        return script.stem.removeprefix("launch_").replace("_", " ").title()

    def refresh(self):
        scripts = sorted((item for item in TABS.glob("*.sh") if item.is_file()), key=lambda item: item.name.casefold())
        if scripts == self._scripts:
            return
        self._scripts = scripts
        for widget in self.buttons.winfo_children():
            widget.destroy()
        if not scripts:
            ttk.Label(self.buttons, text="No launchers found in tabs/").pack(anchor="w")
            return
        for script in scripts:
            ttk.Button(self.buttons, text=self.title(script), command=lambda path=script: self.launch(path)).pack(fill="x", pady=3)

    def watch(self):
        self.refresh()
        for script, (process, notified) in list(self.children.items()):
            result = process.poll()
            if result is not None and not notified:
                if result:
                    self.status.set(f"{self.title(script)} failed to launch (exit {result}); see launcher output")
                else:
                    self.status.set(f"{self.title(script)} closed")
                self.children[script] = (process, True)
        if self.backend is not None and self.backend.poll() is not None:
            self.backend = None
            self.status.set("Shared controller stopped")
        if self.backend is not None and self.children and not any(
            process.poll() is None for process, _notified in self.children.values()
        ):
            self._stop_backend()
        self.root.after(1000, self.watch)

    def _ensure_backend(self):
        if self.backend is not None and self.backend.poll() is None:
            return True
        helper = TABS / "_launch_window.bash"
        env = os.environ.copy()
        env.pop("SAPROG_HOME_BACKEND_PID", None)
        try:
            self.backend = subprocess.Popen(["bash", str(helper), "backend"], cwd=str(WORKSPACE), env=env)
            return True
        except OSError as error:
            self.backend = None
            self.status.set(f"Could not start the controller: {error}")
            messagebox.showerror("Could not start controller", str(error), parent=self.root)
            return False

    def _stop_backend(self):
        if self.backend is None:
            return
        if self.backend.poll() is None:
            self.backend.terminate()
            try:
                self.backend.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.backend.kill()
                self.backend.wait()
        self.backend = None

    def launch(self, script):
        process, _notified = self.children.get(script, (None, False))
        if process and process.poll() is None:
            self.status.set(f"{self.title(script)} is already open")
            return
        if not self._ensure_backend():
            return
        try:
            env = os.environ.copy()
            env["SAPROG_HOME_BACKEND_PID"] = str(self.backend.pid)
            process = subprocess.Popen(["bash", str(script)], cwd=str(WORKSPACE), env=env)
            self.children[script] = (process, False)
            self.status.set(f"Opened {self.title(script)}")
        except OSError as error:
            self.status.set(f"Could not open {self.title(script)}: {error}")
            messagebox.showerror("Could not open window", str(error), parent=self.root)
            if not any(process.poll() is None for process, _notified in self.children.values()):
                self._stop_backend()

    def close(self):
        for process, _notified in self.children.values():
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    process.kill()
        self._stop_backend()
        self.root.destroy()


def main():
    helper = TABS / "_launch_window.bash"
    try:
        result = subprocess.run(["bash", str(helper), "build"], cwd=str(WORKSPACE), check=False)
    except OSError as error:
        print(f"Could not start the workspace build: {error}", file=sys.stderr)
        return 1
    if result.returncode:
        return result.returncode
    root = tk.Tk()
    app = Home(root)
    signal.signal(signal.SIGTERM, lambda *_args: app.close())
    signal.signal(signal.SIGINT, lambda *_args: app.close())
    root.mainloop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
