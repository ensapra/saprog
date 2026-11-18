"""Timeline editor that submits playback to the ROS action executor."""

import copy
from pathlib import Path

from saprog_desktop.desktop_style import apply_style, configure_changed, status_box
from saprog_desktop.motion_controls import add_slider
from saprog_desktop.snapshot_config import SnapshotConfigurationClient
from saprog_desktop.snapshot_units import LEG_JOINTS
from saprog_desktop.timeline_storage import load_timeline
from saprog_desktop.timeline_presenter import TimelineEditorPresenter
from timeline_model import (
    CONTROL_MODES,
    control_groups,
    lifecycle_settings,
    load_snapshot_reference,
    make_snapshot_reference,
    motion_settings,
    regroup_motion,
    resolve_timeline_steps,
    save_timeline,
    timeline_action_data,
    timeline_rows,
    transition_duration,
    validate_timeline,
    wait_seconds,
)


class TimelineWindow:
    def __init__(
        self,
        root,
        node,
        directory,
        container=None,
        on_close=None,
        dependencies=None,
        communication_timeout_ms=8000,
    ):
        if type(communication_timeout_ms) is not int or communication_timeout_ms < 1:
            raise ValueError(
                "Timeline communication timeout must be a positive integer in milliseconds"
            )
        if dependencies is None:
            raise ValueError("TimelineWindow requires injected desktop runtime dependencies")
        self.root, self.node, self.directory = root, node, directory
        self.dependencies = dependencies
        self.container = container or root
        self.joints, self.bounds = [], {}
        self.presenter = TimelineEditorPresenter()
        self.run_steps = None
        self.run_index = 0
        self.editor_index = None
        self.selected_index = None
        self.edit_widgets = []
        self.on_close = on_close
        self.status = dependencies.tk.StringVar(
            value="Waiting for separately started saprog controller"
        )
        self.configuration = SnapshotConfigurationClient(
            node.create_client(dependencies.Trigger, "/saprog/refresh_limits"),
            dependencies.AsyncParameterClient(node, "/saprog_position_controller"),
            dependencies.Trigger,
            self._configuration_result,
        )
        self.configuration.refresh()
        self.timeline_client = dependencies.ActionClient(
            node, dependencies.PlayTimeline, "/saprog/play_timeline"
        )
        self.timeline_goal_handle = None
        self.timeline_goal_future = None
        self.cancel_when_accepted = False
        self.timeline_session_id = 0
        self.timeline_state = "idle"
        self.timeline_timeout_ms = communication_timeout_ms
        self.timeline_timeout_callbacks = {}
        self.timeline_close_state = "open"
        self.view_closed = False
        self.configuration_after_id = root.after(100, self._configuration_tick)
        self._build()
        self.refresh()
        self.refresh_after_id = root.after(2000, self._periodic_refresh)
        if container is None:
            root.protocol("WM_DELETE_WINDOW", self.close)

    @property
    def steps(self):
        if not hasattr(self, "presenter"):
            self.presenter = TimelineEditorPresenter()
        return self.presenter.steps

    @steps.setter
    def steps(self, value):
        if not hasattr(self, "presenter"):
            self.presenter = TimelineEditorPresenter()
        self.presenter.steps = value

    @property
    def dirty(self):
        if not hasattr(self, "presenter"):
            self.presenter = TimelineEditorPresenter()
        return self.presenter.dirty

    @dirty.setter
    def dirty(self, value):
        if not hasattr(self, "presenter"):
            self.presenter = TimelineEditorPresenter()
        self.presenter.dirty = value

    def _configuration_result(self, config, error):
        if config is None:
            if not self.joints:
                self.status.set(f"Controller configuration unavailable: {error}")
            return
        self.joints, self.bounds = list(config.joints), config.bounds
        if self.timeline_state == "idle":
            self.status.set(f"Configured {len(self.joints)} joints; {self._readiness_message()}")

    def _configuration_tick(self):
        self.configuration.poll()
        self.configuration_after_id = self.root.after(100, self._configuration_tick)

    def _build(self):
        if self.container is self.root:
            apply_style(self.root)
            self.root.title("Timeline")
            self.root.geometry("1200x900")
        outer = self.dependencies.ttk.Frame(self.container, padding=14)
        outer.pack(fill="both", expand=True)
        self.dependencies.ttk.Label(outer, text="Timeline", style="Title.TLabel").pack(anchor="w")
        self.dependencies.ttk.Label(
            outer,
            text="Transition → Snapshot → Transition → Snapshot. Click a transition to edit it.\nChoose Global, Leg, or Custom Control for speed and torque strength.",
        ).pack(anchor="w")
        row = self.dependencies.ttk.Frame(outer)
        row.pack(fill="x", pady=8)
        for label, callback in (
            ("New", self.new),
            ("Load timeline", self.load),
            ("Save timeline", self.save),
            ("Add snapshot…", self.add),
            ("Add wait", self.add_wait),
            ("Refresh", self.refresh),
        ):
            self.button(row, label, callback)
        self.available = self.dependencies.ttk.Combobox(outer, state="readonly")
        self.available.pack(fill="x")
        self.edit_widgets.append(self.available)
        self.button(outer, "Add available snapshot", self.add_available, side="top")
        body = self.dependencies.ttk.Frame(outer)
        body.pack(fill="both", expand=True)
        sequence_panel = self.dependencies.ttk.Frame(body)
        sequence_panel.pack(side="left", fill="both", expand=True, padx=(0, 10))
        self.sequence = self.dependencies.tk.Listbox(
            sequence_panel, height=9, exportselection=False
        )
        self.sequence.pack(fill="both", expand=True, pady=8)
        self.sequence.bind("<<ListboxSelect>>", self.select)
        row = self.dependencies.ttk.Frame(sequence_panel)
        row.pack(fill="x")
        for label, callback in (
            ("Up", lambda: self.move(-1)),
            ("Down", lambda: self.move(1)),
            ("Duplicate", self.duplicate),
            ("Remove", self.remove),
        ):
            self.button(row, label, callback)
        self.editor = self.dependencies.ttk.LabelFrame(body, text="Select a transition", padding=8)
        self.editor.pack(side="right", fill="both", expand=True, pady=8)
        self.transition_controls = self.dependencies.ttk.Frame(self.editor)
        self.transition_controls.pack(fill="x")
        self.transition_widgets = []
        self.servo_vars = {}
        lifecycle_row = self.dependencies.ttk.Frame(outer)
        lifecycle_row.pack(fill="x", pady=4)
        self.on_end = self.dependencies.tk.StringVar(value="Torque on")
        for label, variable in (("After timeline", self.on_end),):
            self.dependencies.ttk.Label(lifecycle_row, text=label).pack(side="left", padx=4)
            picker = self.dependencies.ttk.Combobox(
                lifecycle_row,
                textvariable=variable,
                values=("Torque on", "Torque off"),
                state="readonly",
                width=12,
            )
            picker.pack(side="left", padx=4)
            picker.bind("<<ComboboxSelected>>", lambda _: setattr(self, "dirty", True))
            self.edit_widgets.append(picker)
        self.loop = self.dependencies.tk.BooleanVar(value=False)
        loop_toggle = self.dependencies.ttk.Checkbutton(
            lifecycle_row,
            text="Loop timeline",
            variable=self.loop,
            command=lambda: setattr(self, "dirty", True),
        )
        loop_toggle.pack(side="left", padx=8)
        self.edit_widgets.append(loop_toggle)
        row = self.dependencies.ttk.Frame(outer)
        row.pack(fill="x")
        self.play_button = self.button(row, "Play timeline", self.play_timeline)
        self.test_button = self.button(row, "Test selected step", lambda: self.play_timeline(True))
        self.dependencies.ttk.Button(row, text="Stop sequence", command=self.stop).pack(
            side="left", padx=4
        )
        self.dependencies.ttk.Label(
            outer,
            text="Stop prevents further commands; already sent motion or torque changes may finish.",
        ).pack(anchor="w", pady=5)
        self.status_box = status_box(outer, self.status, height=4, tk_module=self.dependencies.tk)
        self.status_box.pack(fill="x")

    def button(self, parent, label, callback, side="left"):
        button = self.dependencies.ttk.Button(parent, text=label, command=callback)
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

    def _periodic_refresh(self):
        if self.timeline_close_state != "closed":
            self.refresh(force=False)
            self.refresh_after_id = self.root.after(2000, self._periodic_refresh)

    def redraw(self, index=None):
        self.sequence.delete(0, "end")
        self.rows = timeline_rows(self.steps)
        for i, kind in self.rows:
            step = self.steps[i]
            if kind == "wait":
                self.sequence.insert("end", "Wait")
            elif kind == "transition":
                self.sequence.insert(
                    "end", f"Transition {i + 1}  →  {self._snapshot(step)['name']}"
                )
            else:
                self.sequence.insert("end", f"    Snapshot: {self._snapshot(step)['name']}")
        self.editor_index = None
        self.selected_index = None
        self._clear_editor()
        if self.steps:
            index = min(index or 0, len(self.steps) - 1)
            self.sequence.selection_set(
                next(row for row, (entry, _) in enumerate(self.rows) if entry == index)
            )
            self.select()

    def _clear_editor(self):
        for child in self.transition_controls.winfo_children():
            child.destroy()
        self.transition_widgets = []
        self.servo_vars = {}

    def _motion_controls(self, parent, motion):
        variables = {
            key: self.dependencies.tk.IntVar(value=motion[key]) for key in ("speed", "torque")
        }
        for key, label in (("speed", "Speed"), ("torque", "Torque strength")):
            self.transition_widgets.append(
                add_slider(
                    parent,
                    label,
                    variables[key],
                    1,
                    ttk_module=self.dependencies.ttk,
                )
            )
        return variables

    def _show_transition(self, step):
        snapshot = self._snapshot(step)
        self.editor.configure(text=f"Transition {self.editor_index + 1} -> {snapshot['name']}")
        duration = transition_duration(step)
        duration_row = self.dependencies.ttk.Frame(self.transition_controls)
        duration_row.pack(fill="x", pady=4)
        self.dependencies.ttk.Label(duration_row, text="Duration").pack(side="left", padx=4)
        self.duration_mode = self.dependencies.tk.StringVar(
            value="Until reached" if duration["mode"] == "until_reached" else "X seconds"
        )
        self.duration_seconds = self.dependencies.tk.StringVar(value=str(duration["seconds"]))
        duration_picker = self.dependencies.ttk.Combobox(
            duration_row,
            textvariable=self.duration_mode,
            values=("Until reached", "X seconds"),
            state="readonly",
            width=16,
        )
        duration_picker.pack(side="left", padx=4)
        seconds_row = self.dependencies.ttk.Frame(duration_row)
        seconds_entry = self.dependencies.ttk.Entry(
            seconds_row, textvariable=self.duration_seconds, width=8
        )
        seconds_entry.pack(side="left")
        self.dependencies.ttk.Label(seconds_row, text="seconds").pack(side="left", padx=4)

        def show_seconds(_event=None):
            if self.duration_mode.get() == "X seconds":
                seconds_row.pack(side="left")
            else:
                seconds_row.pack_forget()

        duration_picker.bind("<<ComboboxSelected>>", show_seconds)
        show_seconds()
        self.transition_widgets.extend((duration_picker, seconds_entry))
        self.edit_mode = step.get(
            "control_mode", "Custom Control" if step.get("overrides") else "Global Control"
        )
        self.mode_var = self.dependencies.tk.StringVar(value=self.edit_mode)
        self.mode_picker = self.dependencies.ttk.Combobox(
            self.transition_controls,
            textvariable=self.mode_var,
            values=CONTROL_MODES,
            state="readonly",
        )
        self.mode_picker.pack(fill="x", pady=4)
        self.mode_picker.bind("<<ComboboxSelected>>", self._change_mode)
        self.transition_widgets.append(self.mode_picker)
        self.dependencies.ttk.Label(
            self.transition_controls,
            text="Switching to finer controls copies current values.\nCombining different values uses their rounded average.",
            wraplength=380,
        ).pack(anchor="w")
        panel = self.dependencies.ttk.Frame(self.transition_controls)
        panel.pack(fill="both", expand=True)
        custom = self.edit_mode == "Custom Control"
        canvas = self.dependencies.tk.Canvas(
            panel, height=430 if custom else 300, highlightthickness=0, width=740 if custom else 380
        )
        scrollbar = self.dependencies.ttk.Scrollbar(panel, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        content = self.dependencies.ttk.Frame(canvas)
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
            for index, (leg, joints) in enumerate(LEG_JOINTS):
                leg_frame = self.dependencies.ttk.LabelFrame(content, text=leg, padding=8)
                leg_frame.grid(row=index // 2, column=index % 2, sticky="nsew", padx=5, pady=5)
                for joint in joints:
                    servo_parents[joint] = leg_frame
            for row in range(2):
                content.rowconfigure(row, weight=1, uniform="leg-row")
            unknown = set(self.groups) - set(servo_parents)
            if unknown:
                extra = self.dependencies.ttk.LabelFrame(content, text="Other servos", padding=8)
                extra.grid(row=2, column=0, columnspan=2, sticky="ew", padx=5, pady=5)
                servo_parents.update({joint: extra for joint in unknown})
        for label, names in self.groups.items():
            frame = self.dependencies.ttk.LabelFrame(
                servo_parents[label] if custom else content, text=label, padding=5
            )
            frame.pack(fill="x", pady=3)
            pair = {
                key: round(sum(resolved[j][key] for j in names) / len(names))
                for key in ("speed", "torque")
            }
            self.group_vars[label] = self._motion_controls(frame, pair)

    def _change_mode(self, _event=None):
        mode = self.mode_var.get()
        if mode == self.edit_mode:
            return
        if not self.apply():
            self.mode_var.set(self.edit_mode)
            return
        snapshot = self._snapshot(self.steps[self.editor_index])
        self.presenter.update_step(
            self.editor_index,
            regroup_motion(self.steps[self.editor_index], mode, list(snapshot["positions"])),
        )
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
                self.presenter.update_step(self.editor_index, step)
                return True
            step["control_mode"] = self.edit_mode
            step["duration"] = {
                "mode": "seconds" if self.duration_mode.get() == "X seconds" else "until_reached",
                "seconds": float(self.duration_seconds.get()),
            }
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
            self.presenter.update_step(self.editor_index, step)
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
            self.sequence.selection_set(
                next(row for row, (entry, _) in enumerate(self.rows) if entry == self.editor_index)
            )
            return
        self.selected_index, kind = self.rows[selection[0]]
        self.editor_index = self.selected_index if kind != "snapshot" else None
        self._clear_editor()
        step = self.steps[self.selected_index]
        if kind == "wait":
            self.editor.configure(text="Wait")
            self.dependencies.ttk.Label(self.transition_controls, text="Duration (seconds)").pack(
                anchor="w"
            )
            self.wait_var = self.dependencies.tk.StringVar(value=str(step["seconds"]))
            entry = self.dependencies.ttk.Entry(
                self.transition_controls, textvariable=self.wait_var
            )
            entry.pack(fill="x", pady=5)
            self.transition_widgets.append(entry)
            self.dependencies.ttk.Label(
                self.transition_controls,
                text="Decimal seconds are supported. The wait starts after the previous move finishes.\nNo servo commands are sent during the wait; torque stays unchanged.",
                wraplength=380,
            ).pack(anchor="w")
        elif self.editor_index is not None:
            self._show_transition(step)
        else:
            self.editor.configure(text=f"Snapshot: {self._snapshot(step)['name']}")
            self.dependencies.ttk.Label(
                self.transition_controls,
                text="Select the transition above this snapshot to change its motion settings.",
            ).pack(anchor="w")

    def add_available(self):
        index = self.available.current()
        if index >= 0:
            self.add(self.paths[index])

    def add_wait(self):
        if not self.apply():
            return
        index = self.selected_index + 1 if self.selected_index is not None else len(self.steps)
        self.presenter.add_wait(index)
        self.redraw(index)

    def add(self, path=None):
        if not self.apply():
            return
        path = path or self.dependencies.filedialog.askopenfilename(
            parent=self.root, initialdir=self.directory, filetypes=[("Snapshot JSON", "*.json")]
        )
        if not path:
            return
        try:
            reference = make_snapshot_reference(path, self.directory)
            self._load_snapshot(reference)
            index = self.presenter.add_snapshot(reference)
            self.redraw(index)
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            self.status.set(f"Snapshot rejected: {exc}")

    def move(self, delta):
        if self.selected_index is None or not self.apply():
            return
        index = self.selected_index
        target = self.presenter.move(index, delta)
        if target is not None:
            self.redraw(target)

    def duplicate(self):
        if self.selected_index is not None and self.apply():
            index = self.presenter.duplicate(self.selected_index)
            self.redraw(index)

    def _load_snapshot(self, reference):
        return load_snapshot_reference(
            reference,
            self.directory,
            self.joints if self.joints else None,
            self.bounds if self.joints else None,
        )

    def _snapshot(self, step):
        snapshot = step["snapshot"]
        return snapshot if isinstance(snapshot, dict) else self._load_snapshot(snapshot)

    def remove(self):
        if self.selected_index is not None:
            index = self.selected_index
            index = self.presenter.remove(index)
            self.redraw(index)

    def discard(self):
        return self.apply() and (
            not self.dirty
            or self.dependencies.messagebox.askyesno(
                "Unsaved timeline", "Discard unsaved changes?", parent=self.root
            )
        )

    def new(self):
        if self.discard():
            self.presenter.replace([])
            self.on_end.set("Torque on")
            self.loop.set(False)
            self.redraw()

    def save(self):
        if not self.apply():
            return
        path = self.dependencies.filedialog.asksaveasfilename(
            parent=self.root,
            defaultextension=".json",
            initialfile="timeline.json",
            filetypes=[("Timeline JSON", "*.json")],
        )
        if path:
            try:
                save_timeline(
                    path,
                    self.steps,
                    self.on_end.get().removeprefix("Torque "),
                    self.loop.get(),
                    snapshot_resolver=self._load_snapshot,
                )
                self.presenter.mark_saved()
                self.redraw(self.editor_index)
                self.status.set(f"Saved {path}")
            except (OSError, ValueError) as exc:
                self.status.set(f"Save failed: {exc}")

    def load(self):
        previous_steps, previous_dirty = copy.deepcopy(self.steps), self.dirty

        def restore_editor():
            self.steps = previous_steps
            self.dirty = previous_dirty

        if not self.discard():
            restore_editor()
            return
        path = self.dependencies.filedialog.askopenfilename(
            parent=self.root, filetypes=[("Timeline JSON", "*.json")]
        )
        if not path:
            restore_editor()
            return
        try:
            document = load_timeline(
                path,
                self.directory,
                self.joints if self.joints else None,
                self.bounds if self.joints else None,
            )
            lifecycle = lifecycle_settings(document)
        except (OSError, ValueError, TypeError, AttributeError) as exc:
            restore_editor()
            self.status.set(f"Load failed: {exc}")
            return
        self.presenter.replace(document["steps"])
        self.on_end.set("Torque " + lifecycle["on_end"])
        self.loop.set(document.get("loop", False))
        self.redraw()
        self.status.set(f"Loaded {path}")

    def play_timeline(self, selected=False):
        if self.run_steps is not None or not self._ready() or not self.apply():
            return
        try:
            steps = resolve_timeline_steps(self.steps, self._load_snapshot)
            validate_timeline(
                {"kind": "saprog_timeline", "schema_version": 1, "steps": steps},
                self.joints,
                self.bounds,
            )
            if selected:
                if self.selected_index is None:
                    return
                steps = [steps[self.selected_index]]
            looping = self.loop.get() and not selected
            fields = timeline_action_data(
                steps,
                self.joints,
                loop=looping,
                run_end_action=not selected and not looping,
                end_torque_enabled=self.on_end.get() == "Torque on",
            )
            goal = self.dependencies.PlayTimeline.Goal()
            for name, value in fields.items():
                setattr(goal, name, value)
            self.run_steps, self.run_index = steps, 0
            self.cancel_when_accepted = False
            self.timeline_session_id += 1
            session_id = self.timeline_session_id
            self.timeline_state = "goal_pending"
            self._update_buttons()
            self.status.set("Submitting timeline to ROS executor...")
            try:
                self.timeline_goal_future = self.timeline_client.send_goal_async(
                    goal,
                    feedback_callback=lambda feedback: self._timeline_feedback(
                        feedback, session_id
                    ),
                )
            except (OSError, RuntimeError, TypeError, ValueError) as exc:
                self._finish_timeline_action(f"Could not submit timeline: {exc}", session_id)
                return
            try:
                self._bound_timeline_wait("goal", session_id)
                self.timeline_goal_future.add_done_callback(
                    lambda future: self._timeline_goal_response(future, session_id)
                )
            except Exception as exc:
                # The request may already be in flight; keep the motion lock until
                # the goal response eventually identifies its backend state.
                self.timeline_state = "goal_unknown"
                self.status.set(
                    f"Goal was submitted but response tracking failed: {exc}. Outcome unknown; timeline remains locked."
                )
        except (OSError, ValueError, TypeError, RuntimeError) as exc:
            self._finish_timeline_action(f"Could not submit timeline: {exc}")

    def _timeline_feedback(self, feedback_message, session_id=None):
        session_id = self.timeline_session_id if session_id is None else session_id
        if (
            session_id != self.timeline_session_id
            or self.run_steps is None
            or self.timeline_state
            not in (
                "active",
                "cancel_pending",
                "cancel_acknowledged",
                "cancel_unknown",
                "result_unknown",
            )
        ):
            return
        feedback = feedback_message.feedback
        self.run_index = max(0, min(len(self.run_steps) - 1, feedback.current_step - 1))
        detail = f"Timeline step {feedback.current_step}/{feedback.step_count}: {feedback.status}"
        if feedback.seconds_remaining > 0:
            detail += f" ({feedback.seconds_remaining:.2f} seconds remaining)"
        self.status.set(detail)

    def _bound_timeline_wait(self, operation, session_id):
        self._clear_timeline_wait(operation)
        callback_id = self.root.after(
            self.timeline_timeout_ms,
            lambda: self._timeline_wait_expired(operation, session_id),
        )
        self.timeline_timeout_callbacks[operation] = callback_id

    def _clear_timeline_wait(self, operation):
        callback_id = self.timeline_timeout_callbacks.pop(operation, None)
        if callback_id is not None:
            try:
                self.root.after_cancel(callback_id)
            except Exception:
                pass

    def _timeline_wait_expired(self, operation, session_id):
        self.timeline_timeout_callbacks.pop(operation, None)
        if session_id != self.timeline_session_id or self.run_steps is None:
            return
        if operation == "goal" and self.timeline_state == "goal_pending":
            self.timeline_state = "goal_unknown"
            self.status.set(
                "Goal response timed out; outcome unknown. Timeline remains locked while ROS may still accept it."
            )
        elif operation == "cancel" and self.timeline_state == "cancel_pending":
            self.timeline_state = "cancel_unknown"
            self.status.set(
                "Cancellation response timed out; cancellation is unconfirmed and the timeline remains locked."
            )
        elif operation == "result" and self.timeline_state in (
            "active",
            "cancel_pending",
            "cancel_acknowledged",
            "cancel_unknown",
        ):
            self.timeline_state = "result_unknown"
            self.status.set(
                "No terminal result arrived within the communication timeout; outcome is unknown and the backend may still run. "
                "No execution deadline is applied; timeline remains locked."
            )

    def _timeline_goal_response(self, future, session_id=None):
        session_id = self.timeline_session_id if session_id is None else session_id
        if session_id != self.timeline_session_id or self.run_steps is None:
            return
        self._clear_timeline_wait("goal")
        try:
            handle = future.result()
        except Exception as exc:
            self.timeline_state = "goal_unknown"
            self.status.set(
                f"Could not confirm timeline goal response: {exc}. Outcome unknown; timeline remains locked."
            )
            return
        self.timeline_goal_future = None
        if not handle.accepted:
            self._finish_timeline_action(
                "ROS timeline executor rejected the goal; no commands sent", session_id
            )
            return
        self.timeline_goal_handle = handle
        try:
            self.timeline_state = "active"
            self.timeline_result_future = handle.get_result_async()
            self._bound_timeline_wait("result", session_id)
            self.timeline_result_future.add_done_callback(
                lambda result_future: self._timeline_result(result_future, session_id)
            )
        except Exception as exc:
            self.timeline_state = "result_unknown"
            self.status.set(
                f"Goal accepted, but result delivery could not be started: {exc}. Outcome unknown; timeline remains locked."
            )
        if self.cancel_when_accepted and self.timeline_state != "result_unknown":
            self._request_timeline_cancel()

    def _timeline_result(self, future, session_id=None):
        session_id = self.timeline_session_id if session_id is None else session_id
        if session_id != self.timeline_session_id or self.run_steps is None:
            return
        self._clear_timeline_wait("result")
        try:
            response = future.result()
            result = response.result
            if getattr(response, "status", None) == 5:
                status = result.message or "Timeline cancellation confirmed by ROS."
            elif result.success:
                status = result.message
            else:
                status = result.message or "ROS timeline execution failed"
        except Exception as exc:
            self.timeline_state = "result_unknown"
            self.status.set(
                f"Could not read timeline result: {exc}. Outcome unknown; timeline remains locked."
            )
            return
        self._finish_timeline_action(status, session_id)

    def _finish_timeline_action(self, status, session_id=None):
        session_id = self.timeline_session_id if session_id is None else session_id
        if session_id != self.timeline_session_id:
            return
        for operation in ("goal", "cancel", "result"):
            self._clear_timeline_wait(operation)
        self.run_steps = None
        self.run_index = 0
        self.timeline_goal_handle = None
        self.timeline_goal_future = None
        self.cancel_when_accepted = False
        self.timeline_state = "idle"
        self.timeline_session_id += 1
        self.status.set(status)
        self._update_buttons()
        if self.timeline_close_state == "pending":
            self.timeline_close_state = "closed"
            self._close_view()

    def _request_timeline_cancel(self):
        if self.timeline_goal_handle is None:
            self.cancel_when_accepted = True
            if self.timeline_state == "goal_unknown":
                self.status.set(
                    "Stop recorded; if ROS later accepts the goal, cancellation will be requested. Timeline remains locked."
                )
            return
        try:
            if self.timeline_state in ("cancel_pending", "cancel_acknowledged", "cancel_unknown"):
                return
            self.timeline_state = "cancel_pending"
            self._bound_timeline_wait("cancel", self.timeline_session_id)
            future = self.timeline_goal_handle.cancel_goal_async()
            session_id = self.timeline_session_id
            future.add_done_callback(
                lambda response_future: self._timeline_cancel_response(response_future, session_id)
            )
            self.status.set("Cancellation requested; waiting for the backend response.")
        except Exception as exc:
            self._clear_timeline_wait("cancel")
            self.timeline_state = "cancel_unknown"
            self.status.set(
                f"Could not confirm cancellation submission: {exc}; timeline may still be running. Editor remains locked."
            )

    def _timeline_cancel_response(self, future, session_id=None):
        session_id = self.timeline_session_id if session_id is None else session_id
        if session_id != self.timeline_session_id or self.run_steps is None:
            return
        self._clear_timeline_wait("cancel")
        try:
            response = future.result()
            if not response.goals_canceling:
                self.timeline_state = "active"
                self.status.set(
                    "Cancellation rejected; the timeline may still be running or applying its final torque action."
                )
            else:
                self.timeline_state = "cancel_acknowledged"
                self._bound_timeline_wait("result", session_id)
                self.status.set(
                    "Cancellation request acknowledged; waiting for the execution result to confirm the outcome."
                )
        except Exception as exc:
            self.timeline_state = "cancel_unknown"
            self.status.set(
                f"Could not confirm cancellation: {exc}; cancellation outcome is unknown and timeline may still be running."
            )

    def _ready(self):
        return bool(self.joints) and self.timeline_client.server_is_ready()

    def _readiness_message(self):
        return (
            "Ready"
            if self.timeline_client.server_is_ready()
            else "Waiting for ROS timeline executor"
        )

    def _playback_tick(self):
        return

    def stop(self):
        if self.run_steps is None:
            return
        self._request_timeline_cancel()

    def _update_buttons(self):
        locked = self.run_steps is not None
        for widget in self.edit_widgets + self.transition_widgets:
            state = (
                "disabled"
                if locked
                else (
                    "readonly" if isinstance(widget, self.dependencies.ttk.Combobox) else "normal"
                )
            )
            configure_changed(widget, state=state)
        self.sequence.configure(state="disabled" if locked else "normal")
        for button in (self.play_button, self.test_button):
            configure_changed(
                button,
                state="normal" if not locked and self._ready() and self.steps else "disabled",
            )

    def close(self):
        if self.timeline_close_state in ("pending", "closed"):
            return
        if self.run_steps is not None:
            if not self.discard():
                return
            if not self.dependencies.messagebox.askyesno(
                "Stop timeline", "Cancel the running timeline before closing?", parent=self.root
            ):
                return
            self.timeline_close_state = "pending"
            self.stop()
            return
        if self.discard():
            self.timeline_close_state = "closed"
            self._close_view()

    def _close_view(self):
        if self.view_closed:
            return
        self.view_closed = True
        self.timeline_close_state = "closed"
        self.configuration.close()
        for callback_id in (self.configuration_after_id, self.refresh_after_id):
            if callback_id is None:
                continue
            try:
                self.root.after_cancel(callback_id)
            except Exception:
                pass
        self.configuration_after_id = self.refresh_after_id = None
        if self.on_close:
            self.on_close()
        else:
            self.root.destroy()
