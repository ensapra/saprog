"""Translate DYNAMIXEL ROS topics into plain telemetry samples."""

import time


class DynamixelRosAdapter:
    """Own graph topic requests and convert their messages for the widget."""

    def __init__(self, manager, on_sample, on_status, owner):
        from ros_manager.config import DYNAMIXEL_TOPICS

        self._manager = manager
        self._on_sample = on_sample
        self._on_status = on_status
        self._topics = DYNAMIXEL_TOPICS
        self._statuses = {}
        self._tokens = []
        manager.message_received.connect(self._handle_message)
        manager.status_changed.connect(self._handle_status)
        manager.destroyed.connect(self._manager_destroyed)
        try:
            for config in self._topics:
                self._tokens.append(manager.request_topic(
                    config.topic,
                    config.message_type,
                    package=config.package,
                    executable=config.executable,
                ))
        except Exception:
            self.release()
            raise
        owner.destroyed.connect(lambda _obj=None: self.release())

    @property
    def connected_count(self):
        return sum(self._statuses.get(config.topic) == "Connected" for config in self._topics)

    def release(self):
        """Release the topic requests owned by this adapter."""
        tokens, self._tokens = self._tokens, []
        manager, self._manager = self._manager, None
        if manager is not None:
            for token in tokens:
                manager.release(token)

    def _manager_destroyed(self, _object=None):
        """Forget the manager before the owning widget is destroyed."""
        self._manager = None
        self._tokens.clear()

    def _handle_message(self, topic, message):
        from ros_manager.config import DYNAMIXEL_SPEED, DYNAMIXEL_TEMPERATURE, DYNAMIXEL_VOLTAGE

        if topic not in {config.topic for config in self._topics}:
            return
        stamp = time.monotonic()
        if topic == DYNAMIXEL_TEMPERATURE.topic:
            name = message.get("header", {}).get("frame_id", "")
            self._on_sample(name, "temperature", message.get("temperature"), stamp)
        else:
            config = DYNAMIXEL_VOLTAGE if topic == DYNAMIXEL_VOLTAGE.topic else DYNAMIXEL_SPEED
            field = "position" if config is DYNAMIXEL_VOLTAGE else "velocity"
            for name, value in zip(message.get("name", ()), message.get(field, ())):
                self._on_sample(name, config.data_type, value, stamp)
        self._set_status(topic, "Connected")

    def _handle_status(self, topic, status):
        if topic in {config.topic for config in self._topics}:
            self._set_status(topic, status)

    def _set_status(self, topic, status):
        self._statuses[topic] = status
        self._on_status(topic, status, self.connected_count)
