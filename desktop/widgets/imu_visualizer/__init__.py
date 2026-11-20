"""IMU readings and orientation widget with optional ROS data requests."""

from PySide6.QtCore import Signal, Slot
from PySide6.QtWidgets import QGroupBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from ros_manager.config import IMU_MAGNETOMETER, IMU_ORIENTATION, IMU_RAW, IMU_TOPICS

from .imu_cube import IMUCube


class IMUVisualizer(QWidget):
    """Display IMU readings and a box rotated by the fused orientation.

    The widget can receive plain values through :meth:`update_data`, or request
    the raw sensor and estimated orientation topics through a shared manager.
    """

    _data_received = Signal(object)

    def __init__(self, parent=None, *, ros_manager=None):
        super().__init__(parent)
        self._ros_manager = ros_manager
        self._ros_statuses = {}
        self._data_received.connect(self._apply_data)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(5, 5, 5, 5)
        layout.setSpacing(4)
        readings_layout = QHBoxLayout()
        readings_layout.setContentsMargins(0, 0, 0, 0)
        readings_layout.setSpacing(4)
        layout.addLayout(readings_layout)

        self._reading_labels = {}
        for key, title, units in (
            ("acceleration", "Accel", "m/s²"),
            ("gyroscope", "Gyro", "rad/s"),
            ("magnetometer", "Mag", "µT"),
        ):
            group = QGroupBox(f"{title} ({units})")
            group.setStyleSheet(
                "QGroupBox { margin-top: 5px; padding: 3px 4px 4px 4px; }"
                "QGroupBox::title { left: 5px; padding: 0 2px; }"
            )
            group_layout = QVBoxLayout(group)
            group_layout.setContentsMargins(3, 4, 3, 2)
            group_layout.setSpacing(1)
            value = QLabel("—")
            value.setObjectName("imuValue")
            value.setStyleSheet("font-size: 9pt;")
            group_layout.addWidget(value)
            readings_layout.addWidget(group, stretch=1)
            self._reading_labels[key] = value

        self.time_label = QLabel("Sensor time: — ms")
        self.time_label.setStyleSheet("font-size: 9pt;")
        layout.addWidget(self.time_label)

        self.connection_label = QLabel(
            "ROS: waiting" if ros_manager is not None else "ROS: disconnected"
        )
        self.connection_label.setStyleSheet("font-size: 9pt;")
        layout.addWidget(self.connection_label)

        orientation_group = QGroupBox("3D orientation")
        orientation_group.setStyleSheet(
            "QGroupBox { margin-top: 5px; padding: 3px 4px 4px 4px; }"
            "QGroupBox::title { left: 5px; padding: 0 2px; }"
        )
        orientation_layout = QVBoxLayout(orientation_group)
        orientation_layout.setContentsMargins(3, 4, 3, 2)
        self.cube = IMUCube(orientation_group)
        orientation_layout.addWidget(self.cube)
        layout.addWidget(orientation_group, stretch=1)

        if ros_manager is not None:
            self._connect_ros(ros_manager)

    def _connect_ros(self, ros_manager):
        ros_manager.message_received.connect(self._on_ros_message)
        ros_manager.status_changed.connect(self._on_ros_status)
        tokens = [
            ros_manager.request_topic(
                topic_config.topic,
                topic_config.message_type,
                package=topic_config.package,
                executable=topic_config.executable,
            )
            for topic_config in IMU_TOPICS
        ]
        # Keep the manager and tokens in the closure, not the widget, so Qt can
        # release the requests when this page is destroyed.
        self.destroyed.connect(
            lambda _object=None, manager=ros_manager, request_tokens=tokens:
                [manager.release(token) for token in request_tokens]
        )

    @Slot(str, object)
    def _on_ros_message(self, topic, message):
        if topic == IMU_RAW.topic:
            acceleration = message.get("linear_acceleration", {})
            gyroscope = message.get("angular_velocity", {})
            self.update_data(
                acceleration=self._vector(acceleration),
                gyroscope=self._vector(gyroscope),
            )
        elif topic == IMU_MAGNETOMETER.topic:
            self.update_data(magnetometer=self._vector(message.get("magnetic_field", {})))
        elif topic == IMU_ORIENTATION.topic:
            quaternion = message.get("quaternion", {})
            try:
                values = tuple(float(quaternion[axis]) for axis in ("x", "y", "z", "w"))
            except (KeyError, TypeError, ValueError):
                return
            self.update_data(orientation_quaternion=values)

    @staticmethod
    def _vector(vector):
        try:
            return tuple(float(vector[axis]) for axis in ("x", "y", "z"))
        except (KeyError, TypeError, ValueError):
            return None

    @Slot(str, str)
    def _on_ros_status(self, topic, status):
        if topic == "":
            self.connection_label.setText(f"ROS: {status}")
            return
        self._ros_statuses[topic] = status
        if all(
            self._ros_statuses.get(topic_config.topic) == "Connected"
            for topic_config in IMU_TOPICS
        ):
            self.connection_label.setText("ROS: connected")
        else:
            self.connection_label.setText(f"ROS: {status.lower()}")

    def update_data(
        self,
        *,
        sensor_time_ms=None,
        acceleration=None,
        gyroscope=None,
        magnetometer=None,
        orientation_quaternion=None,
    ):
        """Queue a sample using plain values (no ROS imports required).

        Vectors are ``(x, y, z)`` tuples. Acceleration is in m/s², gyroscope
        in rad/s, magnetometer in tesla (converted here to µT), and the
        quaternion is ``(x, y, z, w)``. Omitted values leave the display as-is.
        Safe to call from a ROS callback thread; widget updates run on Qt's UI
        thread.
        """
        self._data_received.emit({
            "sensor_time_ms": sensor_time_ms,
            "acceleration": acceleration,
            "gyroscope": gyroscope,
            "magnetometer": magnetometer,
            "orientation_quaternion": orientation_quaternion,
        })

    @Slot(object)
    def _apply_data(self, sample):
        if sample["sensor_time_ms"] is not None:
            self.time_label.setText(f"Sensor time: {sample['sensor_time_ms']:.0f} ms")

        for key in ("acceleration", "gyroscope", "magnetometer"):
            vector = sample[key]
            if vector is None:
                continue
            scale = 1e6 if key == "magnetometer" else 1.0
            values = tuple(float(component) * scale for component in vector)
            self._reading_labels[key].setText(
                f"X  {values[0]:.3f}\nY  {values[1]:.3f}\nZ  {values[2]:.3f}"
            )

        quaternion = sample["orientation_quaternion"]
        if quaternion is not None:
            self.cube.set_orientation(tuple(float(value) for value in quaternion))
