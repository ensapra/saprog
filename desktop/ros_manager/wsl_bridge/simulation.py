"""Acknowledge Gazebo commands only after the simulation node completes them."""

import json
import re
import subprocess
import time

from .protocol import emit


class SimulationCommands:
    def __init__(self, node, string_type, *, operation="simulation", namespace="/saprog/simulation"):
        self.node = node
        self.string_type = string_type
        self.operation = operation
        self.namespace = namespace
        self.publisher = None
        self.subscription = None
        self.pending = {}

    def handle(self, command):
        if command.get("op") != self.operation:
            return False
        if self.publisher is None:
            self.publisher = self.node.create_publisher(self.string_type, f"{self.namespace}/command", 10)
            self.subscription = self.node.create_subscription(
                self.string_type, f"{self.namespace}/result", self._result, 10,
            )
        self.pending[command["id"]] = {
            "payload": dict(command["payload"], id=command["id"]),
            "queued_at": time.monotonic(), "sent": False,
        }
        return True

    def _result(self, message):
        try:
            result = json.loads(message.data)
            request_id = result["id"]
            if request_id not in self.pending:
                return
            self.pending.pop(request_id)
            emit("command_result", id=request_id, success=result["success"] is True,
                 message=str(result.get("message", "")))
        except (ValueError, TypeError, KeyError):
            return

    def process_pending(self):
        for request_id, pending in list(self.pending.items()):
            if time.monotonic() - pending["queued_at"] > 90:
                self.pending.pop(request_id)
                emit("command_result", id=request_id, success=False,
                     message="Simulation node did not respond within 90 seconds")
                continue
            if pending["sent"] or self.publisher.get_subscription_count() == 0:
                continue
            payload = pending["payload"]
            try:
                path = payload.get("urdf_path", "")
                if re.match(r"^[A-Za-z]:[\\/]", path) or path.startswith("\\\\"):
                    result = subprocess.run(
                        ["wslpath", "-u", path], capture_output=True, text=True, timeout=10, check=True,
                    )
                    payload["urdf_path"] = result.stdout.strip()
                message = self.string_type()
                message.data = json.dumps(payload, allow_nan=False)
                self.publisher.publish(message)
                pending["sent"] = True
            except (OSError, ValueError, subprocess.SubprocessError) as error:
                self.pending.pop(request_id)
                emit("command_result", id=request_id, success=False, message=str(error))
