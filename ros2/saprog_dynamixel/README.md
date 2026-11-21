# Saprog DYNAMIXEL control node

[dynamixel_control_node](src/main.cpp) owns one AX-series servo bus for telemetry,
position commands, torque and EEPROM angle-limit reads. Run only one owner of
the configured serial port.

## Configuration and startup

The executable requires parameter overrides at startup. The desktop bridge
loads the installed [saprog.yaml](config/saprog.yaml); a direct launch must pass
it explicitly. After building this package and sourcing ROS and the workspace
overlay, run from the repository's ros2 directory:

```sh
bash ../scripts/ensure_usb.bash "$(ros2 pkg prefix --share saprog_dynamixel)/config/saprog.yaml"
ros2 run saprog_dynamixel dynamixel_control_node --ros-args --params-file \
  "$(ros2 pkg prefix --share saprog_dynamixel)/config/saprog.yaml"
```

The shipped configuration selects /dev/ttyUSB0, 1 Mbps, Protocol 1.0, servo IDs
11-13, 21-23, 31-33 and 41-43, zero positions of 512, and per-servo directions.
poll_rate is 20 requested full sweeps per second; one servo is read per timer
callback, leaving command handling between reads. moving_speed and torque_limit
are command defaults. status_return_level must match the servo EEPROM setting:
1 uses transmit-only writes, 2 waits for write replies. Opening the port does not
enable torque or rewrite EEPROM.

Settings are read into node state at construction; restart to apply changes.
The YAML's target_tolerance and verify_goal_registers entries are not consumed
by the current node and do not enable arrival checks or goal-register verification.

## Published topics

[ServoReadingsPublisher](src/servo_readings_publisher.cpp) encodes the data as:

| Topic | Type | Fields and units |
| --- | --- | --- |
| /saprog/joint_states | sensor_msgs/msg/JointState | name is servo_N; position is calibrated radians; effort is torque-enabled state as 0 or 1 |
| /saprog/temperatures | sensor_msgs/msg/Temperature | One message per servo; header.frame_id is servo_N; temperature is degrees Celsius |
| /saprog/voltages | sensor_msgs/msg/JointState | name identifies servos; position contains volts |
| /saprog/speeds | sensor_msgs/msg/JointState | name identifies servos; velocity contains signed raw AX speed magnitude |

These use reliable QoS. Temperature messages use a depth of 100 to retain the
per-servo burst. Speed telemetry is not radians/second, and joint-state effort
is not measured physical torque. Failed reads are omitted from the published
sweep. Position conversion is direction * (raw - zero) * (5*pi/3) / 1023.

## Position commands

/saprog/position_command accepts sensor_msgs/msg/JointState. name must contain
unique configured servo_N names; position must contain matching finite radian
targets. Optional velocity values are positive radians/second mapping to AX
speed values 1-1023; one raw speed unit equals 0.111 * 2*pi / 60 rad/s. Optional
effort values are raw torque-strength limits from 0 to 1023. Supplied arrays must
match name in length. Omitted settings use the configured defaults.

The node rejects targets outside the raw 0-1023 encoder range. It does not use
the desktop's stored measured-limit JSON for command validation. The bus writes
synchronized torque limits, speeds and goals, then enables torque for the target
servos. A successful transmission does not establish physical arrival.

## Services

| Service | Type | Behavior |
| --- | --- | --- |
| /saprog/servo_N/torque | std_srvs/srv/SetBool | Enable or disable one configured servo |
| /saprog/release_all_torque | std_srvs/srv/Trigger | Disable all configured servos and reject older queued position commands |
| /saprog/read_servo_limits | std_srvs/srv/Trigger | Read all configured CW/CCW limits without register writes |

After release-all, position commands need a valid nonzero timestamp newer than
the release. Torque-enable requests are checked against DDS source timestamps;
requests with missing or older timestamps cannot re-enable torque after release.
Torque-disable requests remain available. At most one torque request per servo
is handled between reads.

A successful limit read returns JSON in response.message with a servos array;
each item has servo_id, zero_position, direction, raw_lower and raw_upper. Any
failed read, wheel-mode/locked range or out-of-range register value fails the
whole request. Static Animator validates this response, converts to radians and
updates its stored bounds. See [operation](../../docs/operation.md).

[Command tests](test/dynamixel_command_test.cpp) use SDK stubs; see
[testing](../../docs/testing.md) for scope and execution.
