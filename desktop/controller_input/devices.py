"""Native SDL controller discovery and input polling for the Qt desktop."""

import math
import os

from PySide6.QtCore import QObject, QTimer, Signal


class ControllerMonitor(QObject):
    """Keep SDL on the GUI thread and identify devices by their instance ID."""

    devices_changed = Signal(object)
    selected_changed = Signal(object)
    inputs_changed = Signal(object)
    axes_changed = Signal(object)
    status_changed = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.devices = {}
        self.selected = None
        self._recover_selection = None
        self.pygame = None
        self._ticks = 0
        self.timer = QTimer(self)
        self.timer.setInterval(30)
        self.timer.timeout.connect(self.poll)

    def start(self):
        try:
            os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
            import pygame

            pygame.display.init()
            pygame.joystick.init()
            self.pygame = pygame
        except (ImportError, RuntimeError) as error:
            self.status_changed.emit(f"Controller support unavailable: {error}. Install pygame.")
            return
        self.refresh()
        self.timer.start()

    def refresh(self):
        if self.pygame is None:
            return
        try:
            self.pygame.event.pump()
            found = {}
            for index in range(self.pygame.joystick.get_count()):
                stick = self.pygame.joystick.Joystick(index)
                stick.init()
                found[stick.get_instance_id()] = stick
            changed = set(found) != set(self.devices)
            old_devices = self.devices
            self.devices = found
            if self.selected is not None and self.selected not in found:
                self.select(None)
            for instance, stick in old_devices.items():
                if instance not in found:
                    stick.quit()
            if changed:
                self.devices_changed.emit([
                    {"id": instance, "name": stick.get_name(), "guid": stick.get_guid()}
                    for instance, stick in found.items()
                ])
            if self.selected is None and self._recover_selection in found:
                self.select(self._recover_selection)
            if not found:
                self.status_changed.emit("No controller detected. Connect a USB or Bluetooth controller.")
        except self.pygame.error as error:
            self._read_failed(f"Controller discovery failed: {error}")

    def select(self, instance):
        self._recover_selection = None
        self.inputs_changed.emit(set())
        self.axes_changed.emit(())
        self.selected = instance if instance in self.devices else None
        stick = self.devices.get(self.selected)
        if stick is None:
            self.selected_changed.emit(None)
            return
        controls = [
            (f"button:{index}", f"Button {index + 1}")
            for index in range(stick.get_numbuttons())
        ]
        for index in range(stick.get_numhats()):
            controls.extend(
                (f"hat:{index}:{direction.lower()}", f"D-pad {index + 1} {direction}")
                for direction in ("Left", "Right", "Up", "Down")
            )
        for index in range(stick.get_numaxes()):
            controls.extend(
                (f"axis:{index}:{direction}", f"Axis {index + 1} {direction}")
                for direction in ("-", "+")
            )
        self.selected_changed.emit({
            "id": self.selected, "name": stick.get_name(), "guid": stick.get_guid(),
            "controls": controls,
            "axis_count": stick.get_numaxes(),
        })
        self.status_changed.emit(f"Connected: {stick.get_name()}")

    def _read_failed(self, message):
        previous = self.selected if self.selected is not None else self._recover_selection
        self.select(None)
        self._recover_selection = previous
        self.status_changed.emit(message)

    def read_state(self):
        """Read the current held state, even when SDL has no new axis events."""
        stick = self.devices.get(self.selected)
        if stick is None:
            return (), set()
        try:
            self.pygame.event.pump()
            if hasattr(stick, "get_attached") and not stick.get_attached():
                self._read_failed("Controller disconnected")
                return (), set()
            pressed = {
                f"button:{index}" for index in range(stick.get_numbuttons())
                if stick.get_button(index)
            }
            for index in range(stick.get_numhats()):
                x, y = stick.get_hat(index)
                for direction, active in (("left", x < 0), ("right", x > 0),
                                          ("up", y > 0), ("down", y < 0)):
                    if active:
                        pressed.add(f"hat:{index}:{direction}")
            axes = tuple(float(stick.get_axis(index)) for index in range(stick.get_numaxes()))
            if any(not math.isfinite(value) or abs(value) > 1.001 for value in axes):
                self._read_failed("Controller returned invalid stick values")
                return (), set()
            for index, value in enumerate(axes):
                if abs(value) >= 0.6:
                    pressed.add(f"axis:{index}:{'+' if value > 0 else '-'}")
            return axes, pressed
        except self.pygame.error as error:
            self._read_failed(f"Controller disconnected: {error}")
            return (), set()

    def poll(self):
        try:
            # Consume SDL hotplug events even when no controller is selected.
            events = self.pygame.event.get()
            self._ticks += 1
            if self._ticks % 33 == 0 or any(
                event.type in (self.pygame.JOYDEVICEADDED, self.pygame.JOYDEVICEREMOVED)
                for event in events
            ):
                self.refresh()
            axes, pressed = self.read_state()
            if self.selected is None:
                return
            self.inputs_changed.emit(pressed)
            self.axes_changed.emit(axes)
        except self.pygame.error as error:
            self._read_failed(f"Controller disconnected: {error}")

    def close(self):
        self.timer.stop()
        self.select(None)
        for stick in self.devices.values():
            stick.quit()
        self.devices.clear()
