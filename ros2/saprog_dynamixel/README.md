# Saprog DYNAMIXEL control node

This ROS 2 node owns one AX-series bus connection for telemetry and commands.
It publishes calibrated positions and torque-enabled state on
`/saprog/joint_states` (`sensor_msgs/JointState`), temperatures on
`/saprog/temperatures` (`sensor_msgs/Temperature`), and voltage/speed telemetry
on `/saprog/voltages` and `/saprog/speeds`.

It accepts `sensor_msgs/JointState` position commands on
`/saprog/position_command`. `name` contains `servo_<id>` joint names,
`position` uses radians, optional `velocity` uses radians/second (AX speed
units), and optional `effort` contains raw AX torque limits from 0 to 1023.
Each configured servo also has a `/saprog/servo_<id>/torque` `std_srvs/SetBool`
service. `/saprog/release_all_torque` is a `std_srvs/Trigger` service that
disables torque for every configured servo and rejects position messages queued
before the release request. Position commands write synchronized torque
limits, speeds and goals, then enable torque for the commanded servos.

After release, position commands must have a valid, nonzero timestamp newer
than the release; unstamped or older commands are discarded. Invalid timestamps
are rejected without stopping the node. Torque-enable requests are checked
against their DDS source timestamps, so delayed requests cannot undo a release.
Middleware that does not provide a source timestamp cannot enable torque through
the service after release; use a fresh stamped position command instead.
Torque-disable requests remain available. Torque services are handled between
servo reads, with at most one request per servo per poll.

Defaults match the current robot: `/dev/ttyUSB0`, 1 Mbps, Protocol 1.0, servo
IDs 11–13, 21–23, 31–33 and 41–43, zero positions of 512 and Saprog's direction
mapping. `poll_rate` defaults to 20 full sweeps per second. Override
`device_name`, `baud_rate`, `protocol_version`, `servo_ids`, `zero_positions`,
`directions` or `poll_rate` with ROS parameters as needed.

Run from the `ros2` colcon workspace with:

```bash
bash ../scripts/ensure_usb.bash install/saprog_dynamixel/share/saprog_dynamixel/config/saprog.yaml
ros2 run saprog_dynamixel dynamixel_control_node --ros-args --params-file \
  "$(ros2 pkg prefix --share saprog_dynamixel)/config/saprog.yaml"
```

The desktop ROS manager runs the USB preflight before starting this executable
and loads the installed `config/saprog.yaml`.

Only one process can own the serial bus at a time. Do not run this alongside
the legacy `saprog_nodes` executable or another DYNAMIXEL bus owner.
