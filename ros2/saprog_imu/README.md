# Serial IMU orientation

The [estimator](src/imu_orientation_estimator.cpp) integrates body-frame gyro
rates and corrects roll/pitch drift using gravity. By default, yaw starts at zero
and remains relative to startup. It can drift with gyro bias over time.

Magnetic heading correction is disabled by default. Previously, every nonzero
raw magnetic reading corrected yaw toward magnetic north with a 0.5-second time
constant. A stuck or biased magnetometer could pull the estimate back toward the
same heading after a real rotation. Disabling magnetic correction prevents that
input from overriding gyro turns; it does not calibrate the magnetometer.

To enable absolute magnetic heading at startup:

```sh
ros2 run saprog_imu imu_publisher_node --ros-args -p use_magnetometer:=true
```

Enable this only with calibrated magnetic measurements expressed in the same
body frame as acceleration and angular velocity. This package currently passes
the serial magnetic measurements through without calibration or axis remapping.
The magnetic-field topic is published in either mode. Gravity correction remains
active in either mode and requires acceleration in g and angular velocity in
degrees/s in the ten-column serial input.

[Regression tests](test/imu_orientation_estimator_test.cpp) cover rotation and
hold on all three axes, gravity drift correction, startup/reset, missing magnetic
data, and opt-in magnetic heading.
