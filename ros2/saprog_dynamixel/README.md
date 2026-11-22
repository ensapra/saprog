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

### Parameters

The shipped [saprog.yaml](config/saprog.yaml) uses the ROS 2 `/**:` wildcard
with settings under `ros__parameters`:

| Parameter | Shipped value | Meaning |
| --- | --- | --- |
| `device_name` | `/dev/ttyUSB0` | Serial device for the servo bus |
| `baud_rate` | `1000000` | Bus baud rate; must match the servos |
| `protocol_version` | `1.0` | DYNAMIXEL packet protocol |
| `poll_rate` | `20.0` | Requested full servo sweeps per second; must be greater than 0 and at most 1000 |
| `moving_speed` | `1023` | Default raw AX moving-speed value, 0-1023, when a command omits velocity; 0 means maximum speed in joint mode |
| `torque_limit` | `1023` | Default raw torque-strength limit, 0-1023, when a command omits effort |
| `status_return_level` | `1` | Must match the servos' EEPROM setting: 1 uses transmit-only writes; 2 waits for write replies |
| `servo_ids` | `[11, 12, 13, 21, 22, 23, 31, 32, 33, 41, 42, 43]` | Nonempty list of unique bus IDs, each from 0 to 253 |
| `directions` | See calibration table below | One sign per servo, either -1 or 1 |
| `zero_positions` | Twelve entries of `512` | Raw encoder position for zero radians, one per servo; each must be 0-1023 |
| `target_tolerance` | `0.1` | Currently unused; does not enable target-arrival checks |
| `verify_goal_registers` | `false` | Currently unused; setting it to true does not enable goal-register verification |

One servo is read per timer callback, leaving command handling between reads.
The requested sweep rate is not a guarantee of the achieved telemetry rate.
Opening the port does not enable torque or rewrite EEPROM, including the
status-return-level setting.

### Servo calibration

The `servo_ids`, `directions` and `zero_positions` arrays match by index and
must have the same length. The shipped calibration is:

| Servo IDs (in order) | Directions (in the same order) | Zero positions |
| --- | --- | --- |
| 11, 12, 13 | -1, -1, -1 | 512, 512, 512 |
| 21, 22, 23 | 1, -1, -1 | 512, 512, 512 |
| 31, 32, 33 | 1, 1, 1 | 512, 512, 512 |
| 41, 42, 43 | -1, 1, 1 | 512, 512, 512 |

For each servo, calibrated position in radians is
`direction * (raw - zero_position) * (5*pi/3) / 1023`. Changing a direction
changes the sign used for both telemetry and position commands; changing a zero
position shifts the raw encoder position corresponding to zero radians.

Settings are read into node state at construction; restart to apply changes.
The startup command above uses the installed configuration. After editing the
source [saprog.yaml](config/saprog.yaml), rebuild/install the package to update
that copy, or pass the source file directly when launching from the repository's
[ros2 directory](..):

```sh
ros2 run saprog_dynamixel dynamixel_control_node --ros-args --params-file \
  saprog_dynamixel/config/saprog.yaml
```

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
