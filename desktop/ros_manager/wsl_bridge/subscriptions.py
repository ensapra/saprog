"""Topic ownership, ROS subscriptions, message forwarding, and status tracking."""

import time

from .protocol import emit


class TopicSubscriptions:
    def __init__(self, node):
        self.node = node
        self.requests = {}
        self.subscriptions = {}
        self.last_status = {}
        self.last_message_at = {}

    def set_status(self, topic, status):
        if self.last_status.get(topic) != status:
            self.last_status[topic] = status
            emit("status", topic=topic, status=status)

    def topic_status(self, topic, publishers):
        last_message = self.last_message_at.get(topic)
        if last_message is not None:
            return "Stale IMU data" if time.monotonic() - last_message > 2.0 else "Connected"
        return "Waiting for IMU data" if publishers > 0 else "Subscribed; waiting for publisher"

    def handle(self, command):
        op = command.get("op")
        token = command.get("token")
        if op == "release":
            self.requests.pop(token, None)
            return
        if op != "subscribe" or not token:
            return

        from rclpy.qos import QoSProfile, qos_profile_sensor_data
        from rosidl_runtime_py.convert import message_to_ordereddict
        from rosidl_runtime_py.utilities import get_message

        request = {
            "topic": command["topic"],
            "message_type": command["message_type"],
            "package": command["package"],
            "executable": command["executable"],
        }
        previous = next(
            (entry for entry in self.requests.values() if entry["topic"] == request["topic"]),
            None,
        )
        if previous and previous != request:
            self.set_status(request["topic"], "Topic requested with a conflicting message type/provider")
            return
        self.requests[token] = request
        if request["topic"] in self.subscriptions:
            return

        try:
            message_class = get_message(request["message_type"])
        except (ImportError, AttributeError, ValueError) as error:
            self.set_status(request["topic"], f"Unknown ROS message type: {error}")
            return

        def receive(message, topic=request["topic"]):
            self.last_message_at[topic] = time.monotonic()
            emit("message", topic=topic, data=dict(message_to_ordereddict(message)))

        # Temperatures arrive as one burst of per-servo messages per sweep.
        # The sensor-data profile keeps only five, dropping early servo IDs
        # before this loop can take them. Match the publisher's reliable queue.
        qos = (
            QoSProfile(depth=100)
            if request["message_type"] == "sensor_msgs/msg/Temperature"
            else qos_profile_sensor_data
        )
        self.subscriptions[request["topic"]] = self.node.create_subscription(
            message_class, request["topic"], receive, qos
        )
        self.set_status(request["topic"], "Subscribed; waiting for publisher")

    def active_requests(self):
        """Remove released subscriptions and return one request per active topic."""
        active_topics = {request["topic"] for request in self.requests.values()}
        for topic in set(self.subscriptions) - active_topics:
            self.node.destroy_subscription(self.subscriptions.pop(topic))
            self.last_message_at.pop(topic, None)
            self.last_status.pop(topic, None)
        return [
            next(item for item in self.requests.values() if item["topic"] == topic)
            for topic in active_topics
        ]
