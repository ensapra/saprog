import math
import unittest
from unittest.mock import Mock

from saprog_desktop.imu_model import orientation_matrix, quaternion_to_euler
from saprog_desktop.imu_view import draw_orientation
from saprog_desktop.temperature_model import TemperatureHistory
from imu_monitor import ImuMonitor


class TemperatureModelTests(unittest.TestCase):
    def test_history_trims_independently_of_selected_time_window(self):
        history = TemperatureHistory()
        history.add_sample("servo", 0, 20)
        history.add_sample("servo", 299, 21)
        history.add_sample("servo", 301, 22)
        history.set_window(5)
        visible, fresh = history.visible(301)
        self.assertEqual(list(history.points["servo"]), [(299, 21), (301, 22)])
        self.assertEqual(visible, [("servo", [(299, 21), (301, 22)])])
        self.assertEqual(fresh, 1)

    def test_window_validation_zoom_limits_and_selection_defaults(self):
        history = TemperatureHistory()
        history.add_servo("one")
        history.select_all(False)
        history.add_servo("two")
        self.assertNotIn("two", history.selected)
        self.assertFalse(history.set_window(float("nan")))
        self.assertFalse(history.set_window(0))
        self.assertTrue(history.set_window(5))
        self.assertEqual(history.zoom(0.5), 2.5)
        self.assertEqual(history.zoom(0.01), 1)
        self.assertEqual(history.zoom(1000), 300)


class FakeCanvas:
    def __init__(self, width, height):
        self.width, self.height, self.items = width, height, []

    def winfo_width(self):
        return self.width

    def winfo_height(self):
        return self.height

    def delete(self, *_args):
        self.items.clear()

    def create_polygon(self, *args, **kwargs):
        self.items.append(("polygon", args, kwargs))

    def create_line(self, *args, **kwargs):
        self.items.append(("line", args, kwargs))

    def create_text(self, *args, **kwargs):
        self.items.append(("text", args, kwargs))


class ImuDisplayMathTests(unittest.TestCase):
    def test_quaternion_projection_and_rotation_basis(self):
        self.assertEqual(quaternion_to_euler((0, 0, 0, 1)), (0.0, 0.0, 0.0))
        self.assertAlmostEqual(quaternion_to_euler((0, 0, math.sin(0.25), math.cos(0.25)))[2], 0.5)
        matrix = orientation_matrix((0, 0, 0))
        self.assertEqual(matrix, ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)))
        canvas = FakeCanvas(800, 500)
        draw_orientation(canvas, (0, 0, 0))
        self.assertGreaterEqual(sum(item[0] == "polygon" for item in canvas.items), 8)
        self.assertTrue(any(item[0] == "line" for item in canvas.items))

    def test_projection_adapts_to_resize_and_shows_absent_state(self):
        small, large = FakeCanvas(300, 240), FakeCanvas(900, 700)
        draw_orientation(small, None)
        draw_orientation(large, None)
        def first_polygon(canvas):
            return next(item for item in canvas.items if item[0] == "polygon")
        self.assertNotEqual(first_polygon(small)[1], first_polygon(large)[1])
        self.assertTrue(any("Waiting for IMU samples" in item[2].get("text", "") for item in large.items))

    def test_mounted_box_does_not_remap_sensor_labels(self):
        canvas = FakeCanvas(800, 500)
        draw_orientation(canvas, (math.pi / 2, 0.0, 0.0))
        labels = [item[2].get("text", "") for item in canvas.items if item[0] == "text"]
        self.assertTrue(any("Roll   90.0" in text and "Pitch    0.0" in text for text in labels))

    def test_unavailable_service_and_close_retire_callbacks(self):
        monitor = ImuMonitor.__new__(ImuMonitor)
        monitor.closed = False
        monitor.status = Mock()
        client = Mock()
        client.service_is_ready.return_value = False
        monitor.clients = {"zero": client}
        monitor.request("zero")
        monitor.status.set.assert_called_with(
            "IMU ROS service unavailable; start the C++ IMU backend."
        )
        client.call_async.assert_not_called()

        future = Mock()
        pending_client = Mock()
        monitor.requests = [(future, 9.0, pending_client)]
        monitor.close()
        pending_client.remove_pending_request.assert_called_once_with(future)
        future.cancel.assert_called_once_with()
        monitor.latest_raw = None
        monitor.receive_raw(object())
        self.assertIsNone(monitor.latest_raw)
        self.assertEqual(monitor.requests, [])


if __name__ == "__main__":
    unittest.main()
