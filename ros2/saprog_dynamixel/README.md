# Saprog DYNAMIXEL readback node

This read-only ROS 2 node connects to the AX-series bus and follows Saprog's
existing telemetry interface: calibrated servo positions and torque-enabled
state on `/saprog/joint_states` (`sensor_msgs/JointState`), plus each servo's
temperature on `/saprog/temperatures` (`sensor_msgs/Temperature`). Position is
reported in radians and each temperature message identifies its servo in
`header.frame_id`.

Defaults match the current robot: `/dev/ttyUSB0`, 1 Mbps, Protocol 1.0, servo
IDs 11–13, 21–23, 31–33 and 41–43, zero positions of 512 and Saprog's direction
mapping. `poll_rate` defaults to 20 full sweeps per second. Override
`device_name`, `baud_rate`, `protocol_version`, `servo_ids`, `zero_positions`,
`directions` or `poll_rate` with ROS parameters as needed.

Run from the `ros2` colcon workspace with:

```bash
ros2 run saprog_dynamixel dynamixel_readback_node --ros-args --params-file \
  "$(ros2 pkg prefix --share saprog_dynamixel)/config/saprog.yaml"
```

The desktop ROS manager also loads the installed `config/saprog.yaml` when it
starts this package.

Only one process can own the serial bus at a time. Stop the legacy `saprog_nodes`
process before starting this node.
