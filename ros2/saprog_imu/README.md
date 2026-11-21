# Saprog serial IMU

[imu_publisher_node](src/imu_publisher_node.cpp) reads ten-column CSV samples from
a serial IMU, publishes raw readings and magnetic field, and estimates orientation.
See the [IMU contract](../../docs/imu.md) for input units, topics, serial recovery
and startup parameters.

After building the package and sourcing ROS and the workspace overlay:

```sh
ros2 run saprog_imu imu_publisher_node --ros-args -p device:=/dev/ttyACM0
```

The default serial port is SAPROG_IMU_DEVICE, otherwise /dev/ttyACM0, at
115200 baud. The desktop WSL bridge can attach and select the sensor before
launch; a direct invocation requires the device to be available already.

The [estimator](src/imu_orientation_estimator.cpp) integrates body-frame gyro
rates and corrects tilt drift using gravity under reliability checks. Yaw starts
at zero by default and remains relative to startup; gyro bias can cause drift.
Magnetic heading correction is disabled by default. To opt in at startup:

```sh
ros2 run saprog_imu imu_publisher_node --ros-args -p use_magnetometer:=true
```

Magnetic heading requires suitable calibrated measurements in the same body
frame as acceleration and gyro. The node does not calibrate or remap magnetic
measurements. Magnetic-field messages are published in either mode. There are
no current calibration or zeroing services.

[Estimator tests](test/imu_orientation_estimator_test.cpp) cover rotations,
hold, gravity correction, reset and opt-in magnetic heading. See
[testing](../../docs/testing.md) for their scope.
