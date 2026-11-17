"""Refresh limits through the serial-port owner before reading its parameters."""
import time
from rclpy.parameter_client import AsyncParameterClient
from std_srvs.srv import Trigger


class HardwareParameterClient(AsyncParameterClient):
    def __init__(self, node, remote_node_name):
        super().__init__(node, remote_node_name)
        self.limits_client = node.create_client(Trigger, "/saprog/refresh_limits")
        self.limits_future = None
        self.limits_ready = False
        self.retry_at = 0.0
        self.requested_at = 0.0
        self.limits_error = "Waiting to read hardware limits"

    def services_are_ready(self):
        if not super().services_are_ready() or not self.limits_client.service_is_ready():
            return False
        if self.limits_ready:
            return True
        if self.limits_future is not None:
            if not self.limits_future.done():
                if time.monotonic() - self.requested_at > 8:
                    self.limits_future.cancel()
                    self.limits_future = None
                    self.retry_at = time.monotonic() + 3
                    self.limits_error = "Hardware limit read timed out; retrying"
                return False
            try:
                response = self.limits_future.result()
                self.limits_ready = response.success
                self.limits_error = response.message
            except Exception as exc:
                self.limits_error = str(exc)
            self.limits_future = None
            self.retry_at = time.monotonic() + 3
            return self.limits_ready
        if time.monotonic() >= self.retry_at:
            self.limits_future = self.limits_client.call_async(Trigger.Request())
            self.requested_at = time.monotonic()
            self.limits_error = "Reading hardware limits from servos..."
        return False
