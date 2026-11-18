# Validation layers

Run ROS and native checks in Ubuntu/WSL with the target ROS 2 distribution
sourced. The package uses Linux serial and PTY APIs; a Windows-native build does
not validate the hardware backend.

## Core build without the embedded RViz view

The embedded renderer is optional for core, IMU, and Python validation. This
build avoids RViz2, Qt Widgets, and X11 development dependencies:

```sh
source /opt/ros/jazzy/setup.bash
colcon build --packages-select saprog --cmake-args -DSAPROG_BUILD_ROBOT_VIEW=OFF
source install/setup.bash
```

The default `SAPROG_BUILD_ROBOT_VIEW=ON` build includes the RViz2 embedded view
and requires its ROS, Qt Widgets, and X11 development packages.

## Pure Python and UI/ROS Python checks

Run each CTest layer independently:

```sh
colcon test --packages-select saprog --ctest-args -R saprog_pure_python_tests
colcon test --packages-select saprog --ctest-args -R saprog_ros_tk_python_tests
```

The pure suite covers storage/preferences and unit/domain validation. The
ROS/Tk suite imports the application-facing clients and views; it uses mocks and
does not open windows or connect to ROS hardware. Run the combined suite with
`colcon test --packages-select saprog` after sourcing the built workspace.

## Native and message-boundary checks

The filter replay, SI/message mapping, config, and bus tests are registered with
CTest. `imu_transport_test` uses a pseudo-terminal and fake helper scripts to
exercise serial exclusivity, bounded line framing, output caps, and subprocess
cleanup without an IMU or WSL USB device.

```sh
colcon test --packages-select saprog --ctest-args -R 'imu_filter_test|imu_message_adapter_test|imu_transport_test|config_test|dynamixel_bus_test'
```

## Device validation

Hardware checks remain separate from automated tests. With the intended IMU
attached, verify device selection, ten-column samples, reconnect/disconnect,
the one-time Space resume, and calibration/zero service behavior. Do not use a
servo adapter as an IMU. Robot mounting, magnetic environment, sensor accuracy,
clipping, and live RViz behavior require the physical setup and must not be
inferred from pure estimator tests.

Build and test results only establish the validation layer that was run. They
do not certify hardware timing or sensor calibration.
