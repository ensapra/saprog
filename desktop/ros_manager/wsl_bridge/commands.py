"""Pending position commands and asynchronous robot control services."""

import time

from .protocol import emit


def cancel_pending_commands(pending_commands, command_ids):
    """Drop only unpublished commands owned by the caller."""
    cancelled = set(command_ids)
    for pending in pending_commands[:]:
        if pending["id"] in cancelled:
            pending_commands.remove(pending)
            emit(
                "command_result", id=pending["id"], success=False,
                message="Cancelled by controller release or selection change",
            )


def publish_position_command(node, publisher, command, message_type):
    """Retain a position command until this publisher has a matched subscriber."""
    if publisher is None:
        publisher = node.create_publisher(message_type, "/saprog/position_command", 10)
    if publisher.get_subscription_count() == 0:
        if time.monotonic() - command["_queued_at"] > 30.0:
            emit(
                "command_result", id=command["id"], success=False,
                message="Position controller unavailable on /saprog/position_command",
            )
            return publisher, True
        return publisher, False
    message = message_type()
    message.header.stamp = node.get_clock().now().to_msg()
    message.name = command["joints"]
    message.position = command["positions"]
    speed = command["speed"] * (0.111 * 2.0 * 3.141592653589793 / 60.0)
    message.velocity = [speed] * len(message.name)
    message.effort = [float(command["torque"])] * len(message.name)
    publisher.publish(message)
    emit("command_result", id=command["id"], success=True, message="Position command published")
    return publisher, True


class RobotCommands:
    """Retry commands until ROS endpoints are ready and acknowledge each result."""

    def __init__(self, node, joint_state_type, set_bool_type, trigger_type):
        self.node = node
        self.joint_state_type = joint_state_type
        self.set_bool_type = set_bool_type
        self.trigger_type = trigger_type
        self.pending_commands = []
        self.torque_clients = {}
        self.release_all_client = None
        self.limits_client = None
        self.command_publisher = None

    def handle(self, command):
        """Return whether this is a robot command, queueing or cancelling it."""
        op = command.get("op")
        if op == "cancel_commands":
            cancel_pending_commands(self.pending_commands, command.get("ids", ()))
            return True
        if op == "release_all_torque":
            for pending in self.pending_commands:
                emit(
                    "command_result", id=pending["id"], success=False,
                    message="Cancelled by Release all torque",
                )
            self.pending_commands.clear()
        elif op not in ("publish_position", "set_torque", "refresh_servo_limits"):
            return False
        self.pending_commands.append(dict(command, _queued_at=time.monotonic()))
        return True

    def process_pending(self):
        remaining_commands = []
        for command in self.pending_commands:
            if command["op"] == "refresh_servo_limits":
                if self.limits_client is None:
                    self.limits_client = self.node.create_client(self.trigger_type, "/saprog/read_servo_limits")
                if not self.limits_client.service_is_ready():
                    if time.monotonic() - command["_queued_at"] > 30.0:
                        emit(
                            "command_result", id=command["id"], success=False,
                            message="Servo limit service is unavailable",
                        )
                    else:
                        remaining_commands.append(command)
                    continue
                future = self.limits_client.call_async(self.trigger_type.Request())

                def limits_done(result, request_id=command["id"]):
                    try:
                        response = result.result()
                        emit(
                            "command_result", id=request_id,
                            success=response.success, message=response.message,
                        )
                    except Exception as error:
                        emit("command_result", id=request_id, success=False, message=str(error))

                future.add_done_callback(limits_done)
            elif command["op"] == "release_all_torque":
                if self.release_all_client is None:
                    self.release_all_client = self.node.create_client(
                        self.trigger_type, "/saprog/release_all_torque"
                    )
                if not self.release_all_client.service_is_ready():
                    if time.monotonic() - command["_queued_at"] > 30.0:
                        emit(
                            "command_result", id=command["id"], success=False,
                            message="Release all torque service is unavailable",
                        )
                    else:
                        remaining_commands.append(command)
                    continue

                request = self.trigger_type.Request()
                future = self.release_all_client.call_async(request)

                def torque_release_done(future, request_id=command["id"]):
                    try:
                        response = future.result()
                        emit(
                            "command_result", id=request_id,
                            success=response.success, message=response.message,
                        )
                    except Exception as error:
                        emit("command_result", id=request_id, success=False, message=str(error))

                future.add_done_callback(torque_release_done)
            elif command["op"] == "publish_position":
                self.command_publisher, completed = publish_position_command(
                    self.node, self.command_publisher, command, self.joint_state_type,
                )
                if not completed:
                    remaining_commands.append(command)
            elif command["op"] == "set_torque":
                service_name = f"/saprog/{command['joint']}/torque"
                client = self.torque_clients.get(service_name)
                if client is None:
                    client = self.node.create_client(self.set_bool_type, service_name)
                    self.torque_clients[service_name] = client
                if client.service_is_ready():
                    request = self.set_bool_type.Request()
                    request.data = command["enabled"]
                    future = client.call_async(request)
                    def torque_done(result, cmd=command):
                        try:
                            response = result.result()
                            emit("command_result", id=cmd["id"], success=response.success, message=response.message)
                        except Exception as error:
                            emit("command_result", id=cmd["id"], success=False, message=str(error))
                    future.add_done_callback(torque_done)
                elif time.monotonic() - command["_queued_at"] > 30.0:
                    emit("command_result", id=command["id"], success=False, message=f"Torque service unavailable: {service_name}")
                else:
                    remaining_commands.append(command)
        self.pending_commands = remaining_commands
