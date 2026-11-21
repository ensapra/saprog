"""Dedicated ROS supervisor; owns no hardware, Gazebo process, or graphics context."""

import importlib.util
import json
import os
import sys
import time
from pathlib import Path

from .controller import LegTestController

NAMESPACE = "/saprog/leg_test"


def main(args=None):
    # colcon may generate a system-Python entry point even when dependencies
    # live in a ROS-compatible venv. Re-exec just this isolated node there.
    runtime = Path(os.environ.get("SAPROG_LEG_TEST_PYTHON", Path.home() / "saprog-leg-env/bin/python"))
    if (importlib.util.find_spec("mujoco") is None and runtime.is_file()
            and os.path.abspath(runtime) != os.path.abspath(sys.executable)):
        os.execv(str(runtime), [str(runtime), "-m", "saprog_leg_test.node", *sys.argv[1:]])

    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    class LegTestNode(Node):
        def __init__(self):
            super().__init__("saprog_leg_test")
            self.rig = LegTestController()
            self.status_publisher = self.create_publisher(String, f"{NAMESPACE}/status", 10)
            self.result_publisher = self.create_publisher(String, f"{NAMESPACE}/result", 10)
            self.subscription = self.create_subscription(String, f"{NAMESPACE}/command", self.command, 10)
            self.last_tick = time.monotonic()
            self.physics_timer = self.create_timer(0.01, self.tick)
            self.status_timer = self.create_timer(0.05, self.publish_state)

        def send(self, publisher, values):
            message = String()
            message.data = json.dumps(values, allow_nan=False)
            publisher.publish(message)

        def command(self, message):
            request_id = None
            try:
                command = json.loads(message.data)
                if not isinstance(command, dict):
                    raise ValueError("Command must be an object")
                request_id = command.get("id")
                result = self.rig.command(command)
                self.send(self.result_publisher, {"id": request_id, "success": True, "message": result})
            except (ValueError, TypeError, RuntimeError) as error:
                self.rig.error = str(error)
                self.send(self.result_publisher, {
                    "id": request_id, "success": False, "message": str(error),
                })
            self.publish_state()

        def tick(self):
            now = time.monotonic()
            self.rig.advance(now - self.last_tick)
            self.last_tick = now

        def publish_state(self):
            self.send(self.status_publisher, self.rig.state())

    rclpy.init(args=args)
    node = LegTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
