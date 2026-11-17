"""Timeline editor that submits playback to the ROS action executor."""
import copy
import json
from pathlib import Path
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from desktop_style import apply_style, configure_changed
from motion_controls import add_slider
import snapshot_player
from snapshot_player import SnapshotPlayer
from timeline_model import (CONTROL_MODES, LEGS, control_groups, regroup_motion,
                            motion_settings, save_timeline, validate_timeline,
                            timeline_rows, wait_seconds, lifecycle_settings, transition_duration,
                            timeline_action_data, make_snapshot_reference,
                            load_snapshot_reference, resolve_timeline_steps)


class TimelineWindow(SnapshotPlayer):
    def __init__(self, root, node, directory):
        self.steps = []
        self.run_steps = None
        self.run_index = 0
        self.editor_index = None
        self.selected_index = None
        self.dirty = False
        self.edit_widgets = []
        super().__init__(root, node, directory)
        self.timeline_client = snapshot_player.ActionClient(
            node, snapshot_player.PlayTimeline, "/saprog/play_timeline")
        self.timeline_goal_handle = None
        self.timeline_goal_future = None
        self.cancel_when_accepted = False
        self.close_after_cancel = False
        root.protocol("WM_DELETE_WINDOW", self.close)

    def _build(self):
        apply_style(self.root)
        self.root.title("Timeline")
        self.root.geometry("1200x900")
        outer = ttk.Frame(self.root, padding=14)
        outer.pack(fill="both", expand=True)
        ttk.Label(outer, text="Timeline", style="Title.TLabel").pack(anchor="w")
        ttk.Label(outer, text="Transition → Snapshot → Transition → Snapshot. Click a transition to edit it.\nChoose Global, Leg, or Custom Control for speed and torque strength.").pack(anchor="w")
        row = ttk.Frame(outer)
        row.pack(fill="x", pady=8)
        for label, callback in (("New", self.new), ("Load timeline", self.load), ("Save timeline", self.save),
                                ("Add snapshot…", self.add), ("Add wait", self.add_wait), ("Refresh", self.refresh)):
            self.button(row, label, callback)
        self.available = ttk.Combobox(outer, state="readonly")
        self.available.pack(fill="x")
        self.edit_widgets.append(self.available)
        self.button(outer, "Add available snapshot", self.add_available, side="top")
        body = ttk.Frame(outer)
        body.pack(fill="both", expand=True)
        sequence_panel = ttk.Frame(body)
        sequence_panel.pack(side="left", fill="both", expand=True, padx=(0, 10))
        self.sequence = tk.Listbox(sequence_panel, height=9, exportselection=False)
        self.sequence.pack(fill="both", expand=True, pady=8)
        self.sequence.bind("<<ListboxSelect>>", self.select)
        row = ttk.Frame(sequence_panel)
        row.pack(fill="x")
        for label, callback in (("Up", lambda: self.move(-1)), ("Down", lambda: self.move(1)),
                                ("Duplicate", self.duplicate), ("Remove", self.remove)):
            self.button(row, label, callback)
        self.editor = ttk.LabelFrame(body, text="Select a transition", padding=8)
        self.editor.pack(side="right", fill="both", expand=True, pady=8)
        self.transition_controls = ttk.Frame(self.editor)
        self.transition_controls.pack(fill="x")
        self.transition_widgets = []
        self.servo_vars = {}
        lifecycle_row = ttk.Frame(outer)
        lifecycle_row.pack(fill="x", pady=4)
        self.on_end = tk.StringVar(value="Torque on")
        for label, variable in (("After timeline", self.on_end),):
            ttk.Label(lifecycle_row, text=label).pack(side="left", padx=4)
            picker = ttk.Combobox(lifecycle_row, textvariable=variable,
                                  values=("Torque on", "Torque off"), state="readonly", width=12)
            picker.pack(side="left", padx=4)
            picker.bind("<<ComboboxSelected>>", lambda _: setattr(self, "dirty", True))
            self.edit_widgets.append(picker)
        self.loop = tk.BooleanVar(value=False)
        loop_toggle = ttk.Checkbutton(lifecycle_row, text="Loop timeline", variable=self.loop,
                                      command=lambda: setattr(self, "dirty", True))
        loop_toggle.pack(side="left", padx=8)
        self.edit_widgets.append(loop_toggle)
        row = ttk.Frame(outer)
        row.pack(fill="x")
        self.play_button = self.button(row, "Play timeline", self.play_timeline)
        self.test_button = self.button(row, "Test selected step", lambda: self.play_timeline(True))
        ttk.Button(row, text="Stop sequence", command=self.stop).pack(side="left", padx=4)
        ttk.Label(outer, text="Stop prevents further commands; already sent motion or torque changes may finish.").pack(anchor="w", pady=5)
        self.status_box = tk.Text(outer, height=4, wrap="word", state="disabled")
        self.status_box.pack(fill="x")
        self.status.trace_add("write", lambda *_: self._refresh_status())
        self._refresh_status()

    def button(self, parent, label, callback, side="left"):
        button = ttk.Button(parent, text=label, command=callback)
        button.pack(side=side, padx=3, pady=3)
        self.edit_widgets.append(button)
        return button

    def refresh(self, force=True):
        if self.run_steps is not None:
            return
        self.paths = sorted(Path(self.directory).glob("*.json"))
        self.available.configure(values=[p.name for p in self.paths])
        if self.paths and not self.available.get():
            self.available.current(0)

    def redraw(self, index=None):
        self.sequence.delete(0, "end")
        self.rows = timeline_rows(self.steps)
        for i, kind in self.rows:
            step = self.steps[i]
            if kind == "wait":
                self.sequence.insert("end", "Wait")
            elif kind == "transition":
                self.sequence.insert("end", f"Transition {i + 1}  →  {self._snapshot(step)['name']}")
            else:
                self.sequence.insert("end", f"    Snapshot: {self._snapshot(step)['name']}")
        self.editor_index = None
        self.selected_index = None
        self._clear_editor()
        if self.steps:
            index = min(index or 0, len(self.steps) - 1)
            self.sequence.selection_set(next(row for row, (entry, _) in enumerate(self.rows) if entry == index))
            self.select()

    def _clear_editor(self):
        for child in self.transition_controls.winfo_children():
            child.destroy()
        self.transition_widgets = []
        self.servo_vars = {}

    def _motion_controls(self, parent, motion):
        variables = {key: tk.IntVar(value=motion[key]) for key in ("speed", "torque")}
        for key, label in (("speed", "Speed"), ("torque", "Torque strength")):
            self.transition_widgets.append(add_slider(parent, label, variables[key], 1))
        return variables

    def _show_transition(self, step):
        snapshot = self._snapshot(step)
        self.editor.configure(text=f"Transition {self.editor_index + 1} -> {snapshot['name']}")
        duration = transition_duration(step)
        duration_row = ttk.Frame(self.transition_controls)
        duration_row.pack(fill="x", pady=4)
        ttk.Label(duration_row, text="Duration").pack(side="left", padx=4)
        self.duration_mode = tk.StringVar(value="Until reached" if duration["mode"] == "until_reached" else "X seconds")
        self.duration_seconds = tk.StringVar(value=str(duration["seconds"]))
        duration_picker = ttk.Combobox(duration_row, textvariable=self.duration_mode,
                                       values=("Until reached", "X seconds"), state="readonly", width=16)
        duration_picker.pack(side="left", padx=4)
        seconds_row = ttk.Frame(duration_row)
        seconds_entry = ttk.Entry(seconds_row, textvariable=self.duration_seconds, width=8)
        seconds_entry.pack(side="left")
        ttk.Label(seconds_row, text="seconds").pack(side="left", padx=4)
        def show_seconds(_event=None):
            if self.duration_mode.get() == "X seconds":
                seconds_row.pack(side="left")
            else:
                seconds_row.pack_forget()
        duration_picker.bind("<<ComboboxSelected>>", show_seconds)
        show_seconds()
        self.transition_widgets.extend((duration_picker, seconds_entry))
        self.edit_mode = step.get("control_mode", "Custom Control" if step.get("overrides") else "Global Control")
        self.mode_var = tk.StringVar(value=self.edit_mode)
        self.mode_picker = ttk.Combobox(self.transition_controls, textvariable=self.mode_var,
                                       values=CONTROL_MODES, state="readonly")
        self.mode_picker.pack(fill="x", pady=4)
        self.mode_picker.bind("<<ComboboxSelected>>", self._change_mode)
        self.transition_widgets.append(self.mode_picker)
        ttk.Label(self.transition_controls, text="Switching to finer controls copies current values.\nCombining different values uses their rounded average.", wraplength=380).pack(anchor="w")
        panel = ttk.Frame(self.transition_controls)
        panel.pack(fill="both", expand=True)
        custom = self.edit_mode == "Custom Control"
        canvas = tk.Canvas(panel, height=430 if custom else 300, highlightthickness=0,
                           width=740 if custom else 380)
        scrollbar = ttk.Scrollbar(panel, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        content = ttk.Frame(canvas)
        window = canvas.create_window((0, 0), window=content, anchor="nw")
        content.bind("<Configure>", lambda _: canvas.configure(scrollregion=canvas.bbox("all")))
        canvas.bind("<Configure>", lambda event: canvas.itemconfigure(window, width=event.width))
        resolved = motion_settings(step, snapshot["positions"])
        self.group_vars = {}
        self.groups = control_groups(self.edit_mode, snapshot["positions"])
        servo_parents = {}
        if custom:
            for column in range(2):
                content.columnconfigure(column, weight=1, uniform="leg-column")
            for index, (leg, ids) in enumerate(LEGS.items()):
                leg_frame = ttk.LabelFrame(content, text=f"{leg.title()} Leg", padding=8)
                leg_frame.grid(row=index // 2, column=index % 2, sticky="nsew", padx=5, pady=5)
                for servo in ids:
                    servo_parents[f"servo_{servo}"] = leg_frame
            for row in range(2):
                content.rowconfigure(row, weight=1, uniform="leg-row")
            unknown = set(self.groups) - set(servo_parents)
            if unknown:
                extra = ttk.LabelFrame(content, text="Other servos", padding=8)
                extra.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
                servo_parents.update({joint: extra for joint in unknown})
        for label, names in self.groups.items():
            frame = ttk.LabelFrame(servo_parents[label] if custom else content, text=label, padding=5)
            frame.pack(fill="x", pady=3)
            pair = {key: round(sum(resolved[j][key] for j in names) / len(names))
                    for key in ("speed", "torque")}
            self.group_vars[label] = self._motion_controls(frame, pair)

    def _change_mode(self, _event=None):
        mode = self.mode_var.get()
        if mode == self.edit_mode:
            return
        if not self.apply():
            self.mode_var.set(self.edit_mode)
            return
        snapshot = self._snapshot(self.steps[self.editor_index])
        self.steps[self.editor_index] = regroup_motion(
            self.steps[self.editor_index], mode, list(snapshot["positions"]))
        self.dirty = True
        self._clear_editor()
        self._show_transition(self.steps[self.editor_index])

    def apply(self):
        if self.editor_index is None:
            return True
        try:
            step = copy.deepcopy(self.steps[self.editor_index])
            if step.get("kind") == "wait":
                step["seconds"] = float(self.wait_var.get())
                wait_seconds(step)
                if step != self.steps[self.editor_index]:
                    self.dirty = True
                self.steps[self.editor_index] = step
                return True
            step["control_mode"] = self.edit_mode
            step["duration"] = {"mode": "seconds" if self.duration_mode.get() == "X seconds" else "until_reached",
                                "seconds": float(self.duration_seconds.get())}
            transition_duration(step)
            step["motion"] = {key: step["motion"][key] for key in ("speed", "torque")}
            step["overrides"] = {}
            for label, names in self.groups.items():
                pair = {key: var.get() for key, var in self.group_vars[label].items()}
                if self.edit_mode == "Global Control":
                    step["motion"] = pair
                else:
                    for joint in names:
                        step["overrides"][joint] = dict(pair)
            motion_settings(step, self._snapshot(step)["positions"])
            if step != self.steps[self.editor_index]:
                self.dirty = True
            self.steps[self.editor_index] = step
            return True
        except (ValueError, TypeError) as exc:
            self.status.set(f"Invalid step settings: {exc}")
            return False

    def select(self, _event=None):
        if self.run_steps is not None:
            return
        selection = self.sequence.curselection()
        if not selection:
            return
        if not self.apply():
            self.sequence.selection_clear(0, "end")
            self.sequence.selection_set(next(row for row, (entry, _) in enumerate(self.rows) if entry == self.editor_index))
            return
        self.selected_index, kind = self.rows[selection[0]]
        self.editor_index = self.selected_index if kind != "snapshot" else None
        self._clear_editor()
        step = self.steps[self.selected_index]
        if kind == "wait":
            self.editor.configure(text="Wait")
            ttk.Label(self.transition_controls, text="Duration (seconds)").pack(anchor="w")
            self.wait_var = tk.StringVar(value=str(step["seconds"]))
            entry = ttk.Entry(self.transition_controls, textvariable=self.wait_var)
            entry.pack(fill="x", pady=5)
            self.transition_widgets.append(entry)
            ttk.Label(self.transition_controls, text="Decimal seconds are supported. The wait starts after the previous move finishes.\nNo servo commands are sent during the wait; torque stays unchanged.", wraplength=380).pack(anchor="w")
        elif self.editor_index is not None:
            self._show_transition(step)
        else:
            self.editor.configure(text=f"Snapshot: {self._snapshot(step)['name']}")
            ttk.Label(self.transition_controls, text="Select the transition above this snapshot to change its motion settings.").pack(anchor="w")

    def add_available(self):
        index = self.available.current()
        if index >= 0:
            self.add(self.paths[index])

    def add_wait(self):
        if not self.apply():
            return
        index = self.selected_index + 1 if self.selected_index is not None else len(self.steps)
        self.steps.insert(index, {"kind": "wait", "seconds": 1.0})
        self.dirty = True
        self.redraw(index)

    def add(self, path=None):
        if not self.apply():
            return
        path = path or filedialog.askopenfilename(parent=self.root, initialdir=self.directory, filetypes=[("Snapshot JSON", "*.json")])
        if not path:
            return
        try:
            reference = make_snapshot_reference(path, self.directory)
            self._load_snapshot(reference)
            step = {"snapshot": reference, "motion": {"speed": 1023, "torque": 1023}, "overrides": {}}
            self.steps.append(step)
            self.dirty = True
            self.redraw(len(self.steps) - 1)
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            self.status.set(f"Snapshot rejected: {exc}")

    def move(self, delta):
        if self.selected_index is None or not self.apply():
            return
        index = self.selected_index
        target = index + delta
        if 0 <= target < len(self.steps):
            self.steps[index], self.steps[target] = self.steps[target], self.steps[index]
            self.dirty = True
            self.redraw(target)

    def duplicate(self):
        if self.selected_index is not None and self.apply():
            index = self.selected_index + 1
            self.steps.insert(index, copy.deepcopy(self.steps[self.selected_index]))
            self.dirty = True
            self.redraw(index)

    def _load_snapshot(self, reference):
        return load_snapshot_reference(
            reference, self.directory,
            self.joints if self.joints else None,
            self.bounds if self.joints else None)

    def _snapshot(self, step):
        snapshot = step["snapshot"]
        return snapshot if isinstance(snapshot, dict) else self._load_snapshot(snapshot)

    def _reference_for_legacy_snapshot(self, snapshot):
        name = snapshot.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ValueError("Legacy timeline snapshot has no name")
        for path in Path(self.directory).glob("*.json"):
            try:
                candidate = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, ValueError, TypeError):
                continue
            if isinstance(candidate, dict) and candidate.get("name") == name:
                return make_snapshot_reference(path, self.directory)
        raise ValueError(f"Snapshot '{name}' is not present in {self.directory}")

    def remove(self):
        if self.selected_index is not None:
            index = self.selected_index
            del self.steps[index]
            self.dirty = True
            self.redraw(index)

    def discard(self):
        return self.apply() and (not self.dirty or messagebox.askyesno("Unsaved timeline", "Discard unsaved changes?", parent=self.root))

    def new(self):
        if self.discard():
            self.steps = []
            self.on_end.set("Torque on")
            self.loop.set(False)
            self.dirty = False
            self.redraw()

    def save(self):
        if not self.apply():
            return
        path = filedialog.asksaveasfilename(parent=self.root, defaultextension=".json", initialfile="timeline.json", filetypes=[("Timeline JSON", "*.json")])
        if path:
            try:
                save_timeline(path, self.steps, self.on_end.get().removeprefix("Torque "),
                              self.loop.get(), snapshot_resolver=self._load_snapshot)
                self.dirty = False
                self.redraw(self.editor_index)
                self.status.set(f"Saved {path}")
            except (OSError, ValueError) as exc:
                self.status.set(f"Save failed: {exc}")

    def load(self):
        if not self.discard():
            return
        path = filedialog.askopenfilename(parent=self.root, filetypes=[("Timeline JSON", "*.json")])
        if path:
            try:
                document = json.loads(Path(path).read_text(encoding="utf-8"))
                if document.get("schema_version") == 1:
                    document["steps"] = [
                        {**step, "snapshot": self._reference_for_legacy_snapshot(step["snapshot"])}
                        if isinstance(step, dict) and isinstance(step.get("snapshot"), dict)
                        else step
                        for step in document.get("steps", [])]
                    document["schema_version"] = 2
                steps = validate_timeline(document, snapshot_resolver=self._load_snapshot)
                lifecycle = lifecycle_settings(document)
                self.steps = steps
                self.on_end.set("Torque " + lifecycle["on_end"])
                self.loop.set(document.get("loop", False))
                self.dirty = False
                self.redraw()
                self.status.set(f"Loaded {path}")
            except (OSError, ValueError, TypeError, AttributeError) as exc:
                self.status.set(f"Load failed: {exc}")

    def play_timeline(self, selected=False):
        if self.run_steps is not None or not self._ready() or not self.apply():
            return
        try:
            steps = resolve_timeline_steps(self.steps, self._load_snapshot)
            validate_timeline(
                {"kind": "saprog_timeline", "schema_version": 1, "steps": steps},
                self.joints, self.bounds)
            if selected:
                if self.selected_index is None:
                    return
                steps = [steps[self.selected_index]]
            looping = self.loop.get() and not selected
            fields = timeline_action_data(
                steps, self.joints, loop=looping,
                run_end_action=not selected and not looping,
                end_torque_enabled=self.on_end.get() == "Torque on")
            goal = snapshot_player.PlayTimeline.Goal()
            for name, value in fields.items():
                setattr(goal, name, value)
            self.run_steps, self.run_index = steps, 0
            self.cancel_when_accepted = False
            self._update_buttons()
            self.status.set("Submitting timeline to ROS executor...")
            self.timeline_goal_future = self.timeline_client.send_goal_async(
                goal, feedback_callback=self._timeline_feedback)
            self.timeline_goal_future.add_done_callback(self._timeline_goal_response)
        except (OSError, ValueError, TypeError, RuntimeError) as exc:
            self.status.set(f"Timeline rejected; no commands sent: {exc}")

    def _timeline_feedback(self, feedback_message):
        if self.run_steps is None:
            return
        feedback = feedback_message.feedback
        self.run_index = max(0, min(len(self.run_steps) - 1, feedback.current_step - 1))
        detail = f"Timeline step {feedback.current_step}/{feedback.step_count}: {feedback.status}"
        if feedback.seconds_remaining > 0:
            detail += f" ({feedback.seconds_remaining:.2f} seconds remaining)"
        self.status.set(detail)

    def _timeline_goal_response(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            self._finish_timeline_action(f"Could not submit timeline action: {exc}")
            return
        self.timeline_goal_future = None
        if not handle.accepted:
            self._finish_timeline_action("ROS timeline executor rejected the goal; no commands sent")
            return
        self.timeline_goal_handle = handle
        self.timeline_result_future = handle.get_result_async()
        self.timeline_result_future.add_done_callback(self._timeline_result)
        if self.cancel_when_accepted:
            self._request_timeline_cancel()

    def _timeline_result(self, future):
        try:
            response = future.result()
            result = response.result
            if result.success:
                status = result.message
            else:
                status = result.message or "ROS timeline execution failed"
        except Exception as exc:
            status = f"Could not read timeline result: {exc}"
        self._finish_timeline_action(status)

    def _finish_timeline_action(self, status):
        self.run_steps = None
        self.run_index = 0
        self.timeline_goal_handle = None
        self.timeline_goal_future = None
        self.cancel_when_accepted = False
        self.status.set(status)
        self._update_buttons()
        if self.close_after_cancel:
            self.close_after_cancel = False
            self.root.destroy()

    def _request_timeline_cancel(self):
        if self.timeline_goal_handle is None:
            self.cancel_when_accepted = True
            return
        try:
            future = self.timeline_goal_handle.cancel_goal_async()
            future.add_done_callback(self._timeline_cancel_response)
            self.status.set("Cancel requested; ROS will stop before the next timeline command")
        except Exception as exc:
            self.status.set(f"Could not request timeline cancellation: {exc}")

    def _timeline_cancel_response(self, future):
        try:
            response = future.result()
            if not response.goals_canceling:
                self.status.set("Cancellation rejected; the timeline may already be applying its final torque action")
        except Exception as exc:
            self.status.set(f"Could not confirm timeline cancellation: {exc}")

    def _ready(self):
        return super()._ready() and self.timeline_client.server_is_ready()

    def _playback_tick(self):
        return

    def stop(self):
        if self.run_steps is None:
            return
        self._request_timeline_cancel()

    def _update_buttons(self):
        locked = self.run_steps is not None
        for widget in self.edit_widgets + self.transition_widgets:
            state = "disabled" if locked else ("readonly" if isinstance(widget, ttk.Combobox) else "normal")
            configure_changed(widget, state=state)
        self.sequence.configure(state="disabled" if locked else "normal")
        for button in (self.play_button, self.test_button):
            configure_changed(button, state="normal" if not locked and self._ready() and self.steps else "disabled")

    def close(self):
        if self.run_steps is not None:
            if not self.discard():
                return
            if not messagebox.askyesno(
                "Stop timeline", "Cancel the running timeline before closing?", parent=self.root):
                return
            self.close_after_cancel = True
            self.stop()
            return
        if self.discard():
            self.root.destroy()
