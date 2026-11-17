from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from snapshot_limits_client import HardwareParameterClient, AsyncParameterClient


class LimitsClientTest(unittest.TestCase):
    def test_each_window_requests_a_new_hardware_read(self):
        future = Mock()
        future.done.return_value = True
        future.result.return_value = NS(success=True, message="Read limits")
        service = Mock()
        service.service_is_ready.return_value = True
        service.call_async.return_value = future
        node = Mock()
        node.create_client.return_value = service
        with patch.object(AsyncParameterClient, "__init__", return_value=None), patch.object(AsyncParameterClient, "services_are_ready", return_value=True):
            for _ in range(2):
                client = HardwareParameterClient(node, "/controller")
                self.assertFalse(client.services_are_ready())
                self.assertTrue(client.services_are_ready())
                self.assertTrue(client.services_are_ready())
        self.assertEqual(service.call_async.call_count, 2)

    def test_failed_read_is_not_ready(self):
        future = Mock()
        future.done.return_value = True
        future.result.return_value = NS(success=False, message="servo_13 missing")
        service = Mock()
        service.service_is_ready.return_value = True
        service.call_async.return_value = future
        node = Mock()
        node.create_client.return_value = service
        with patch.object(AsyncParameterClient, "__init__", return_value=None), patch.object(AsyncParameterClient, "services_are_ready", return_value=True):
            client = HardwareParameterClient(node, "/controller")
            self.assertFalse(client.services_are_ready())
            self.assertFalse(client.services_are_ready())
            self.assertEqual(client.limits_error, "servo_13 missing")


if __name__ == "__main__":
    unittest.main()
