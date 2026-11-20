# Project documentation

These guides describe the current Qt desktop and the ROS packages in this
checkout. Source code and configuration take precedence when behavior changes.

- [Getting started](../README.md): install dependencies and launch the desktop.
- [Desktop](../desktop/README.md): available tabs and extension points.
- [Operation and file formats](operation.md): controls, controller mappings,
  snapshots and timelines.
- [Code guide](code-guide.md): ownership, data flow and reusable modules.
- [Desktop style](desktop-style.md): current Qt palette and presentation helpers.
- [IMU](imu.md): serial input, topics, estimation and limitations.
- [DYNAMIXEL](../ros2/saprog_dynamixel/README.md): servo configuration and ROS interfaces.
- [Robot description](../urdf/README.md): model assets, joint mapping and approximations.
- [Testing](testing.md): current suites and their scope.

The desktop ROS worker currently supports Windows with WSL. The Qt shell and
local document editing can run without ROS; the ROS executables can also be
started directly in Linux. There is no current desktop robot-view or simulation
tab, ROS timeline action, or IMU calibration/zeroing service.
