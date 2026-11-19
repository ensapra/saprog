"""Native Windows Pygame bridge for the WSL Saprog desktop."""

import argparse
import json
import queue
import sys
import threading
import time
from pathlib import Path

import pygame

output_file = None


def emit(message):
    sys.stdout.write(json.dumps(message) + "\n")
    sys.stdout.flush()
    if output_file is not None:
        with output_file.open("a", encoding="utf-8") as stream:
            stream.write(json.dumps(message) + "\n")
            stream.flush()


def read_commands(commands):
    for line in sys.stdin:
        line = line.strip()
        if line.startswith("select:"):
            commands.put(int(line.partition(":")[2]))
        elif line == "quit":
            commands.put(None)


def read_command_file(commands):
    if command_file is None:
        return
    position = 0
    while True:
        try:
            text = command_file.read_text(encoding="utf-8")
        except OSError:
            time.sleep(0.03)
            continue
        if len(text) < position:
            position = 0
        new_text = text[position:]
        position = len(text)
        for line in new_text.splitlines():
            line = line.strip()
            if line.startswith("select:"):
                commands.put(int(line.partition(":")[2]))
            elif line == "quit":
                commands.put(None)
        time.sleep(0.03)


parser = argparse.ArgumentParser()
parser.add_argument("--command-file")
parser.add_argument("--output-file")
args = parser.parse_args()

command_file = Path(args.command_file) if args.command_file else None
output_file = Path(args.output_file) if args.output_file else None
if output_file is not None:
    output_file.parent.mkdir(parents=True, exist_ok=True)
    output_file.write_text("", encoding="utf-8")
if command_file is not None:
    command_file.parent.mkdir(parents=True, exist_ok=True)
    command_file.write_text("", encoding="utf-8")

pygame.init()
pygame.joystick.init()
commands = queue.Queue()
threading.Thread(target=read_commands, args=(commands,), daemon=True).start()
threading.Thread(target=read_command_file, args=(commands,), daemon=True).start()

joystick = None
selected_index = None
previous_pressed = set()

devices = []
for index in range(pygame.joystick.get_count()):
    device = pygame.joystick.Joystick(index)
    try:
        device.init()
        devices.append({"index": index, "name": device.get_name()})
    finally:
        device.quit()
emit({"type": "devices", "devices": devices})

while True:
    while not commands.empty():
        requested = commands.get_nowait()
        if requested is None:
            raise SystemExit(0)
        if joystick is not None:
            joystick.quit()
            joystick = None
        selected_index = requested
        previous_pressed = set()
        if 0 <= requested < pygame.joystick.get_count():
            joystick = pygame.joystick.Joystick(requested)
            joystick.init()
            emit({
                "type": "selected",
                "index": requested,
                "button_count": joystick.get_numbuttons(),
                "hat_count": joystick.get_numhats(),
            })

    if joystick is not None:
        pygame.event.pump()
        pressed = {
            f"button:{index}"
            for index in range(joystick.get_numbuttons())
            if joystick.get_button(index)
        }
        for hat in range(joystick.get_numhats()):
            x, y = joystick.get_hat(hat)
            if x < 0:
                pressed.add(f"hat:{hat}:left")
            if x > 0:
                pressed.add(f"hat:{hat}:right")
            if y < 0:
                pressed.add(f"hat:{hat}:down")
            if y > 0:
                pressed.add(f"hat:{hat}:up")
        for input_key in sorted(pressed ^ previous_pressed):
            emit({
                "type": "input_state",
                "input": input_key,
                "pressed": input_key in pressed,
            })
        previous_pressed = pressed

    pygame.time.wait(30)
