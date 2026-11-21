"""ROS 2 supervisor; import and process work stays off the ROS executor."""

import json
from concurrent.futures import ThreadPoolExecutor

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from .session import GazeboSession

PREFIX = "/saprog/simulation"


class SimulationNode(Node):
    def __init__(self):
        super().__init__("saprog_simulation")
        self.session = GazeboSession()
        self.worker = ThreadPoolExecutor(max_workers=1, thread_name_prefix="gazebo")
        self.future = None
        self.request_id = None
        self.message = "Simulation stopped"
        self.error = ""
        self.status_publisher = self.create_publisher(String, f"{PREFIX}/status", 10)
        self.result_publisher = self.create_publisher(String, f"{PREFIX}/result", 10)
        self.subscription = self.create_subscription(String, f"{PREFIX}/command", self.command, 10)
        self.timer = self.create_timer(0.5, self.tick)

    def reply(self, request_id, success, message):
        self.result_publisher.publish(String(data=json.dumps({
            "id": request_id, "success": success, "message": message,
        })))

    def command(self, message):
        request_id = ""
        try:
            command = json.loads(message.data)
            request_id = command["id"]
            action = command["action"]
            if self.future is not None:
                raise ValueError("Simulation is busy; wait for the current action")
            operations = {
                "start": lambda: self.session.start(command),
                "stop": self.session.stop,
                "pause": lambda: self.session.pause(True),
                "resume": lambda: self.session.pause(False),
                "reset": self.session.reset,
                "show_viewer": self.session.show_viewer,
                "hide_viewer": self.session.hide_viewer,
            }
            if action not in operations:
                raise ValueError(f"Unknown simulation action: {action}")
            self.error = ""
            self.message = f"{action.replace('_', ' ').capitalize()} in progress"
            self.request_id = request_id
            self.future = self.worker.submit(operations[action])
        except (ValueError, TypeError, KeyError) as error:
            self.reply(request_id, False, str(error))

    def tick(self):
        if self.future is not None and self.future.done():
            try:
                self.message = self.future.result()
                success = True
            except Exception as error:
                self.error = self.message = str(error)
                self.get_logger().error(self.error)
                success = False
            self.reply(self.request_id, success, self.message)
            self.future = None
        # The worker mutates process state during start/stop; read it only when
        # idle. Gazebo stats are copied under their own short callback lock.
        state = self.session.status() if self.future is None else {}
        if self.future is None and not state["running"] and self.session.server is not None:
            self.error = f"Gazebo exited: {self.session.log_tail()}"
        state.update(busy=self.future is not None, message=self.message, error=self.error)
        self.status_publisher.publish(String(data=json.dumps(state, allow_nan=False)))

    def close(self):
        self.session.closing.set()
        self.worker.shutdown(wait=True, cancel_futures=True)
        self.session.stop()


def main(args=None):
    rclpy.init(args=args)
    node = SimulationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
