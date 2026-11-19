"""ROS-independent IMU readings and orientation widget."""

from PySide6.QtCore import Signal, Slot
from PySide6.QtWidgets import QGroupBox, QHBoxLayout, QLabel, QVBoxLayout, QWidget

from .imu_cube import IMUCube


class IMUVisualizer(QWidget):
    """Display IMU readings and a box rotated by the fused orientation.

    A future data adapter can call :meth:`update_data` with ordinary Python
    numbers and tuples, without making this widget depend on ROS message types.
    """

    _data_received = Signal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
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
