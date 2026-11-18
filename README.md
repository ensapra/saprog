# Saprog

ROS 2 Jazzy tools for a twelve-servo robot: pose creation/playback, timelines,
temperature monitoring, IMU orientation and an RViz2 robot view. Runtime requires
Linux or WSL with ROS and access to the configured USB devices.

Start Home from the workspace with `bash ./launch_home.sh`; it builds before opening.

Project documentation lives in [docs/](docs/README.md):

- [Maintainer context](docs/README.md): code map, behavior to preserve, open issues and validation.
- [Operation](docs/operation.md): dependencies, USB setup, controls and file contracts.
- [IMU](docs/imu.md): sensor format, ROS interface, calibration and limitations.
