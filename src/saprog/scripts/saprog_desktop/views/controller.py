"""Game controller discovery, button mappings and playback dispatch."""

import json
import subprocess
from pathlib import Path

from saprog_desktop.desktop_style import ACCENT, WHITE
from saprog_desktop.snapshot_files import write_json_atomic


def controller_settings_path():
    """Return the per-user controller mapping file."""
    return Path.home() / ".config" / "saprog" / "controller.json"


class ControllerTab:
    """Poll one selected SDL controller and dispatch configured button edges."""

    POLL_MS = 30

    def __init__(self, parent, root, dependencies, snapshot_directory, sequence_directory,
                 *, usb_helper=None, on_snapshot, on_sequence, on_torque):
        self.root = root
        self.dependencies = dependencies
        self.ttk = dependencies.ttk
        self.directory = Path(snapshot_directory)
        self.sequence_directory = Path(sequence_directory)
        self.usb_helper = Path(usb_helper) if usb_helper is not None else None
        self.on_snapshot = on_snapshot
        self.on_sequence = on_sequence
        self.on_torque = on_torque
        self.closed = False
        self.joystick = None
        self.device_ids = []
        self.button_count = 0
        self.settings_path = controller_settings_path()
        self.settings = self._load_settings()
        self.settings.setdefault("button_actions", {})
        self.snapshot_paths = []
        self.sequence_paths = []
        self.mapping_vars = {}
        self.input_labels = {}
        self.available_buttons = []
        self.previous_pressed = set()
        self.after_id = None
        self.usb_process = None
        self.usb_error = ""
        self.bridge_process = None
        self.bridge_devices = []
        self.bridge_command_file = None
        self.bridge_output_file = None
        self.bridge_output_position = 0

        self.status = dependencies.tk.StringVar(value="Controller support is starting")
        self.device = dependencies.tk.StringVar()
        self._build(parent)
        self.refresh()
        self._poll()

    def _load_settings(self):
        try:
            with self.settings_path.open(encoding="utf-8") as stream:
                settings = json.load(stream)
            if not isinstance(settings, dict):
                return {}
            if not isinstance(settings.get("mappings"), dict):
                settings["mappings"] = {}
            if not isinstance(settings.get("button_actions"), dict):
                settings["button_actions"] = {}
            if not settings["button_actions"]:
                for action, button in settings["mappings"].items():
                    if isinstance(button, int):
                        settings["button_actions"][f"button:{button}"] = action
                torque_button = settings.get("torque_button")
                if isinstance(torque_button, int):
                    settings["button_actions"].setdefault(
                        f"button:{torque_button}", "torque"
                    )
            return settings
        except (OSError, ValueError, UnicodeError):
            return {}

    def _save_settings(self):
        try:
            self.settings_path.parent.mkdir(parents=True, exist_ok=True)
            write_json_atomic(self.settings_path, self.settings)
        except OSError as exc:
            self.status.set(f"Mapping changed but could not be saved: {exc}")

    def _build(self, parent):
        ttk = self.ttk
        ttk.Style(parent).configure(
            "Controller.Pressed.TLabel",
            background=ACCENT,
            foreground=WHITE,
            padding=(4, 2),
        )
        ttk.Label(parent, text="Controller", style="Title.TLabel").pack(anchor="w")
        ttk.Label(
            parent,
            text="Select a connected game controller. Press a control to highlight it, then choose its action from the dropdown.",
            style="Muted.TLabel",
            wraplength=760,
        ).pack(anchor="w", pady=(2, 10))
        row = ttk.Frame(parent)
        row.pack(fill="x", pady=(0, 8))
        ttk.Label(row, text="Device", width=12).pack(side="left")
        self.device_combo = ttk.Combobox(row, textvariable=self.device, state="readonly", width=42)
        self.device_combo.pack(side="left", padx=(0, 6))
        self.device_combo.bind("<<ComboboxSelected>>", self._select_device)
        ttk.Button(row, text="Refresh devices", command=self.refresh).pack(side="left")

        self.mapping_frame = ttk.LabelFrame(parent, text="Button mappings", padding=8)
        self.mapping_frame.pack(fill="both", expand=True)
        canvas = self.dependencies.tk.Canvas(self.mapping_frame, highlightthickness=0)
        scrollbar = ttk.Scrollbar(self.mapping_frame, orient="vertical", command=canvas.yview)
        self.rows = ttk.Frame(canvas)
        self.rows.bind("<Configure>", lambda _event: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.create_window((0, 0), window=self.rows, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")
        ttk.Label(parent, textvariable=self.status, wraplength=760).pack(anchor="w", pady=(8, 0))

    @staticmethod
    def _action_key(kind, path):
        return f"{kind}:{path.name}"

    def _action_values(self):
        values = ["Unmapped", "Toggle torque"]
        values += [f"Set snapshot: {path.stem}" for path in self.snapshot_paths]
        values += [f"Play sequence: {path.stem}" for path in self.sequence_paths]
        return values

    def _action_label(self, action):
        if action == "torque":
            return "Toggle torque"
        kind, separator, filename = action.partition(":")
        if not separator:
            return "Unmapped"
        path = next(
            (
                item
                for item in (self.snapshot_paths if kind == "snapshot" else self.sequence_paths)
                if item.name == filename
            ),
            None,
        )
        if path is None:
            return "Unmapped"
        if kind == "snapshot":
            return f"Set snapshot: {path.stem}"
        if kind == "sequence":
            return f"Play sequence: {path.stem}"
        return "Unmapped"

    def _action_key_from_label(self, label):
        if label == "Toggle torque":
            return "torque"
        if label.startswith("Set snapshot: "):
            name = label[len("Set snapshot: "):]
            path = next((item for item in self.snapshot_paths if item.stem == name), None)
            return f"snapshot:{path.name}" if path is not None else None
        if label.startswith("Play sequence: "):
            name = label[len("Play sequence: "):]
            path = next((item for item in self.sequence_paths if item.stem == name), None)
            return f"sequence:{path.name}" if path is not None else None
        return None

    @staticmethod
    def _native_windows_available():
        try:
            if Path("/proc/sys/kernel/osrelease").read_text(encoding="utf-8").lower().find("microsoft") >= 0:
                return True
            return bool(__import__("os").environ.get("WSL_DISTRO_NAME"))
        except OSError:
            return False

    def _start_native_bridge(self):
        if self.bridge_process is not None and self.bridge_process.poll() is None:
            self.status.set("Connecting to the native Windows controller...")
            return

        source = Path(__file__).resolve()
        repo = None
        for candidate in (source.parent, *source.parents):
            if (candidate / "launch_home.sh").is_file():
                repo = candidate
                break
        if repo is None:
            for candidate in (source.parent, *source.parents):
                if (candidate / ".venv" / "Scripts" / "python.exe").is_file():
                    repo = candidate
                    break

        windows_python = repo / ".venv" / "Scripts" / "python.exe"
        bridge = repo / "scripts" / "windows_controller_bridge.py"
        if not windows_python.is_file():
            self.status.set(
                "Native Windows controller support requires .venv/Scripts/python.exe "
                "with pygame installed."
            )
            self.device_combo.configure(values=[])
            self._build_mappings()
            return
        if not bridge.is_file():
            self.status.set("Native Windows controller bridge was not found.")
            self._build_mappings()
            return

        try:
            def windows_path(path):
                completed = subprocess.run(
                    ["wslpath", "-w", str(path)],
                    capture_output=True,
                    text=True,
                    check=True,
                )
                return completed.stdout.strip()

            self.bridge_command_file = repo / "controller-command.jsonl"
            self.bridge_output_file = repo / "controller-output.jsonl"
            self.bridge_output_position = 0
            self.bridge_command_file.write_text("", encoding="utf-8")
            self.bridge_output_file.write_text("", encoding="utf-8")

            self.bridge_process = subprocess.Popen(
                [
                    "powershell.exe",
                    "-NoProfile",
                    "-Command",
                    (
                        f"& '{windows_path(windows_python)}' "
                        f"'{windows_path(bridge)}' "
                        f"--command-file '{windows_path(self.bridge_command_file)}' "
                        f"--output-file '{windows_path(self.bridge_output_file)}'"
                    ),
                ],
                stdin=subprocess.PIPE,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.bridge_devices = []
            self.status.set("Connecting to the native Windows controller...")
        except OSError as exc:
            self.bridge_process = None
            self.status.set(f"Could not start native Windows Pygame: {exc}")

    def _poll_native_bridge(self):
        process = self.bridge_process
        if process is None:
            return
        if process.poll() is not None:
            error = ""
            if process.stderr is not None:
                error = process.stderr.read().strip()
            self._update_pressed_inputs(set())
            self.status.set(error or "Native Windows controller bridge stopped.")
            return
        if self.bridge_output_file is None:
            return
        try:
            text = self.bridge_output_file.read_text(encoding="utf-8")
        except OSError:
            return
        if len(text) < self.bridge_output_position:
            self.bridge_output_position = 0
        new_text = text[self.bridge_output_position:]
        self.bridge_output_position = len(text)
        for line in new_text.splitlines():
            if not line:
                continue
            try:
                message = json.loads(line)
            except json.JSONDecodeError:
                continue
            if message.get("type") == "devices":
                self.bridge_devices = message.get("devices", [])
                labels = [f"{device['name']} ({device['index']})" for device in self.bridge_devices]
                self.device_combo.configure(values=labels)
                selected = self.settings.get("device_index")
                if selected is not None:
                    for position, device in enumerate(self.bridge_devices):
                        if device["index"] == selected:
                            self.device_combo.current(position)
                            break
                elif labels:
                    self.device_combo.current(0)
                self._select_device()
            elif message.get("type") == "selected":
                self._update_pressed_inputs(set())
                self.button_count = int(message["button_count"])
                hat_count = int(message["hat_count"])
                self.available_buttons = [
                    {"key": f"button:{button}", "label": f"Button {button}"}
                    for button in range(self.button_count)
                ]
                for hat in range(hat_count):
                    for direction in ("Left", "Right", "Up", "Down"):
                        self.available_buttons.append({
                            "key": f"hat:{hat}:{direction.lower()}",
                            "label": f"D-pad {hat + 1} {direction}",
                        })
                self.status.set(
                    f"Connected: native Windows controller — "
                    f"{self.button_count} buttons and {hat_count * 4} D-pad directions"
                )
                self._build_mappings()
            elif message.get("type") == "edge":
                self._dispatch(message["input"])
            elif message.get("type") == "input_state":
                self._set_input_pressed(message["input"], message["pressed"])

    def refresh(self):
        if self.closed:
            return
        self.snapshot_paths = sorted(self.directory.glob("*.json"))
        self.sequence_paths = sorted(self.sequence_directory.glob("*.json"))
        if self._native_windows_available():
            self._start_native_bridge()
            return
        if self.usb_helper is not None:
            if self.usb_process is not None:
                if self.usb_process.poll() is None:
                    self.status.set(
                        "Checking and attaching the USB controller to WSL; approve the Windows prompt if shown..."
                    )
                else:
                    self._finish_usb_preflight()
                return
            try:
                self.usb_process = subprocess.Popen(
                    ["bash", str(self.usb_helper)],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                )
                self.usb_error = ""
                self.status.set(
                    "Checking and attaching the USB controller to WSL; approve the Windows prompt if shown..."
                )
                self.root.after(100, self._finish_usb_preflight)
                return
            except OSError as exc:
                self.usb_error = f"Could not start USB discovery: {exc}"
        self._enumerate_devices()

    def _finish_usb_preflight(self):
        process = self.usb_process
        if self.closed or process is None:
            return
        if process.poll() is None:
            self.root.after(100, self._finish_usb_preflight)
            return
        output, _ = process.communicate()
        self.usb_process = None
        self.usb_error = output.strip() if process.returncode else ""
        self._enumerate_devices()

    def _enumerate_devices(self):
        try:
            import pygame

            pygame.joystick.init()
            self.pygame = pygame
            self.device_ids = list(range(pygame.joystick.get_count()))
            labels = []
            for index in self.device_ids:
                joystick = pygame.joystick.Joystick(index)
                try:
                    joystick.init()
                    labels.append(f"{joystick.get_name()} ({index})")
                finally:
                    joystick.quit()
            self.device_combo.configure(values=labels)
            selected = self.settings.get("device_index")
            if selected in self.device_ids:
                self.device_combo.current(self.device_ids.index(selected))
            elif labels:
                self.device_combo.current(0)
            self._select_device()
            if not labels:
                self.status.set(self.usb_error or "No controller found. Connect it and choose Refresh devices.")
        except ImportError:
            self.pygame = None
            self.device_combo.configure(values=[])
            self.status.set("Controller support is unavailable (install python3-pygame in the ROS environment).")
        except Exception as exc:
            self.pygame = None
            self.device_combo.configure(values=[])
            self.status.set(f"Could not enumerate controllers: {exc}")
        self._build_mappings()

    def _select_device(self, _event=None):
        if self.bridge_process is not None:
            current = self.device_combo.current()
            if current < 0 or current >= len(self.bridge_devices):
                self.button_count = 0
                self.available_buttons = []
                self._update_pressed_inputs(set())
                self._build_mappings()
                return
            device = self.bridge_devices[current]
            index = int(device["index"])
            self.settings["device_index"] = index
            self._save_settings()
            if self.bridge_command_file is not None:
                with self.bridge_command_file.open("a", encoding="utf-8") as stream:
                    stream.write(f"select:{index}\n")
                    stream.flush()
            return
        if self.joystick is not None:
            try:
                self.joystick.quit()
            except Exception:
                pass
            self.joystick = None
        current = self.device_combo.current()
        if self.pygame is None or current < 0 or current >= len(self.device_ids):
            self.button_count = 0
            self.available_buttons = []
            self._update_pressed_inputs(set())
            self._button_values()
            self._build_mappings()
            return
        index = self.device_ids[current]
        try:
            self.joystick = self.pygame.joystick.Joystick(index)
            self.joystick.init()
            self.button_count = self.joystick.get_numbuttons()
            self.available_buttons = [
                {"key": f"button:{button}", "label": f"Button {button}"}
                for button in range(self.button_count)
            ]
            for hat in range(self.joystick.get_numhats()):
                for direction in ("Left", "Right", "Up", "Down"):
                    self.available_buttons.append({
                        "key": f"hat:{hat}:{direction.lower()}",
                        "label": f"D-pad {hat + 1} {direction}",
                    })
            self.settings["device_index"] = index
            self._save_settings()
            press_count = len(self.available_buttons)
            self.status.set(
                f"Connected: {self.joystick.get_name()} — "
                f"{self.button_count} buttons and {press_count - self.button_count} D-pad directions"
            )
        except Exception as exc:
            self.joystick = None
            self.button_count = 0
            self.available_buttons = []
            self.status.set(f"Could not open controller: {exc}")
        self._update_pressed_inputs(set())
        self._build_mappings()

    def _button_values(self):
        values = self._action_values()
        for variable in self.mapping_vars.values():
            action = variable.get()
            variable.set(action if action in values else "Unmapped")

    def _build_mappings(self):
        for child in self.rows.winfo_children():
            child.destroy()
        self.mapping_vars.clear()
        self.input_labels.clear()
        ttk = self.ttk
        ttk.Label(self.rows, text="Button", style="Muted.TLabel").grid(row=0, column=0, sticky="w", padx=4)
        ttk.Label(self.rows, text="Action", style="Muted.TLabel").grid(row=0, column=1, sticky="w", padx=4)
        values = self._action_values()
        button_actions = self.settings.setdefault("button_actions", {})
        for row, button in enumerate(self.available_buttons, start=1):
            key = button["key"]
            input_label = ttk.Label(
                self.rows,
                text=button["label"],
                style=(
                    "Controller.Pressed.TLabel"
                    if key in self.previous_pressed
                    else "TLabel"
                ),
            )
            input_label.grid(row=row, column=0, sticky="w", padx=4, pady=2)
            self.input_labels[key] = input_label
            variable = self.dependencies.tk.StringVar(value="Unmapped")
            action = button_actions.get(key)
            if isinstance(action, str) and self._action_label(action) in values:
                variable.set(self._action_label(action))
            self.mapping_vars[key] = variable
            combo = ttk.Combobox(
                self.rows,
                textvariable=variable,
                values=values,
                state="readonly",
                width=36,
            )
            combo.grid(row=row, column=1, sticky="w", padx=4, pady=2)
            combo.bind(
                "<<ComboboxSelected>>",
                lambda _event, k=key, v=variable: self._save_mapping(k, v),
            )
        self._button_values()
        if not self.available_buttons:
            ttk.Label(self.rows, text="No press buttons or D-pad directions found.", style="Muted.TLabel").grid(
                row=1, column=0, sticky="w", padx=4, pady=4
            )

    def _update_pressed_inputs(self, pressed):
        pressed = set(pressed)
        newly_pressed = pressed - self.previous_pressed
        changed = pressed ^ self.previous_pressed
        for key in changed:
            label = self.input_labels.get(key)
            if label is not None:
                label.configure(
                    style=(
                        "Controller.Pressed.TLabel"
                        if key in pressed
                        else "TLabel"
                    )
                )
        self.previous_pressed = pressed
        return newly_pressed

    def _set_input_pressed(self, key, is_pressed):
        pressed = set(self.previous_pressed)
        if is_pressed:
            pressed.add(key)
        else:
            pressed.discard(key)
        if is_pressed and key in self._update_pressed_inputs(pressed):
            self._dispatch(key)
        elif not is_pressed:
            self._update_pressed_inputs(pressed)

    def _save_mapping(self, key, variable):
        mappings = self.settings.setdefault("button_actions", {})
        action = self._action_key_from_label(variable.get())
        if action is None:
            mappings.pop(key, None)
        else:
            mappings[key] = action
        self.settings["mappings"] = {}
        self._save_settings()

    def _poll(self):
        if self.closed:
            return
        if self.bridge_process is not None:
            self._poll_native_bridge()
        if self.joystick is not None:
            try:
                self.pygame.event.pump()
                pressed = {
                    f"button:{index}"
                    for index in range(self.button_count)
                    if self.joystick.get_button(index)
                }
                for hat, state in enumerate(self.joystick.get_hats()):
                    x, y = state
                    if x < 0:
                        pressed.add(f"hat:{hat}:left")
                    if x > 0:
                        pressed.add(f"hat:{hat}:right")
                    if y < 0:
                        pressed.add(f"hat:{hat}:down")
                    if y > 0:
                        pressed.add(f"hat:{hat}:up")
                edges = self._update_pressed_inputs(pressed)
                for button in sorted(edges, key=lambda item: str(item)):
                    self._dispatch(button)
            except Exception as exc:
                self.status.set(f"Controller disconnected: {exc}")
                self.joystick = None
                self._update_pressed_inputs(set())
        self.after_id = self.root.after(self.POLL_MS, self._poll)

    def _dispatch(self, button):
        action = self.settings.get("button_actions", {}).get(
            f"button:{button}" if isinstance(button, int) else button
        )
        if action == "torque":
            self.on_torque()
            return
        if not isinstance(action, str):
            return
        kind, separator, filename = action.partition(":")
        if not separator:
            return
        paths = self.snapshot_paths if kind == "snapshot" else self.sequence_paths
        path = next((item for item in paths if item.name == filename), None)
        if path is None:
            self.status.set("Mapped file is no longer available; refresh the tab")
        elif kind == "snapshot":
            self.on_snapshot(str(path))
        elif kind == "sequence":
            self.on_sequence(str(path))

    def close(self):
        self.closed = True
        if self.after_id is not None:
            try:
                self.root.after_cancel(self.after_id)
            except Exception:
                pass
            self.after_id = None
        if self.bridge_process is not None:
            if self.bridge_command_file is not None:
                try:
                    with self.bridge_command_file.open("a", encoding="utf-8") as stream:
                        stream.write("quit\n")
                        stream.flush()
                except Exception:
                    pass
            if self.bridge_process.poll() is None:
                try:
                    self.bridge_process.terminate()
                    self.bridge_process.wait(timeout=1)
                except Exception:
                    self.bridge_process.kill()
                    self.bridge_process.wait()
            self.bridge_process = None
        if self.joystick is not None:
            try:
                self.joystick.quit()
            except Exception:
                pass
            self.joystick = None
        if self.usb_process is not None:
            if self.usb_process.poll() is None:
                self.usb_process.terminate()
                try:
                    self.usb_process.wait(timeout=1)
                except subprocess.TimeoutExpired:
                    self.usb_process.kill()
                    self.usb_process.wait()
            if self.usb_process.stdout is not None:
                self.usb_process.stdout.close()
            self.usb_process = None
