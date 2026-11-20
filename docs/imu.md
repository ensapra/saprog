# IMU contract

The current provider is [imu_publisher_node](../ros2/saprog_imu/src/imu_publisher_node.cpp)
in saprog_imu. The [Statistics widget](../desktop/widgets/imu_visualizer/__init__.py)
requests its three topics through the shared desktop ROS manager.

## Serial input

[SerialImuReader](../ros2/saprog_imu/src/serial_imu_reader.cpp) opens a nonblocking
8N1 serial connection and extracts newline-delimited records, accepting CRLF.
The [parser](../ros2/saprog_imu/src/imu_csv_parser.cpp) requires exactly ten finite
numeric CSV fields:

```text
time_ms, ax, ay, az, gx, gy, gz, mx, my, mz
```

Time must be nonnegative. Acceleration is in g, angular velocity in degrees/s,
and magnetic field in microtesla. Extra fields, nonnumeric values and trailing
commas are rejected. Headers are not data. The reader discards a pending buffer
larger than 4096 bytes.

| Startup parameter | Default |
| --- | --- |
| device | SAPROG_IMU_DEVICE, otherwise /dev/ttyACM0 |
| baud_rate | 115200 |
| frame_id | imu_link |
| use_magnetometer | false |

Supported baud rates are 9600, 19200, 38400, 57600 and 115200. The node polls every
5 ms, retries opening at two-second intervals and reconnects after three seconds
without serial bytes. It resets orientation after three seconds without a valid
sample. A silent newly opened device receives one space character after about
1.5 seconds to toggle firmware streaming.

## Published data

[ImuMessagePublisher](../ros2/saprog_imu/src/imu_message_publisher.cpp) uses
sensor-data QoS and ROS receipt timestamps.

| Topic | Type | Contents |
| --- | --- | --- |
| /saprog/imu/data_raw | sensor_msgs/msg/Imu | Acceleration in m/s² and angular velocity in rad/s; orientation_covariance[0] is -1 |
| /saprog/imu/mag | sensor_msgs/msg/MagneticField | Magnetic field in tesla |
| /saprog/imu/orientation | geometry_msgs/msg/QuaternionStamped | Estimated orientation, published when valid, with frame imu_world |

Raw and magnetic messages use frame_id. Quaternion fields are x, y, z, w on the
ROS interface; the estimator internally stores w, x, y, z.

## Orientation and limitations

The [estimator](../ros2/saprog_imu/src/imu_orientation_estimator.cpp) initializes
tilt from nonzero acceleration and integrates body-frame gyro rates using
firmware timestamps. It applies gravity correction when acceleration magnitude
and rotation rate satisfy its reliability checks. Yaw starts at zero by default
and can drift with gyro bias.

use_magnetometer enables magnetic heading correction at startup. Magnetic values
are passed through without calibration or axis remapping, so this mode requires
suitable input measurements. See the [package guide](../ros2/saprog_imu/README.md).
There are no current gyro-calibration, magnetic-calibration, connect or zeroing
ROS services, and no persisted calibration profile. The desktop draws a sensor
orientation box; it does not apply a measured sensor-to-robot mounting transform.

## Direct inspection

After building the package and sourcing ROS plus the ros2 workspace overlay:

```sh
ros2 run saprog_imu imu_publisher_node --ros-args -p device:=/dev/ttyACM0
```

In another sourced shell:

```sh
ros2 topic echo /saprog/imu/data_raw sensor_msgs/msg/Imu --qos-reliability best_effort --once
```

For WSL USB selection and desktop launch, see [operation](operation.md).
