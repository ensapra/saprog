# IMU contract and limitations

[imu_node.cpp](../src/saprog/src/imu_node.cpp) owns the serial device, USB helper,
parsing and ROS services. [imu_filter.hpp](../src/saprog/include/saprog/imu_filter.hpp)
owns attitude, gyro bias and zero. The [monitor](../src/saprog/scripts/imu_monitor.py)
only displays data/calls services; closing it preserves backend state. Reconnect
or backend restart resets calibration and zero. Home starts/stops its own backend.

## Input and estimation

SparkFun calculated output defaults to 115200 baud and exactly ten finite columns:

```text
time_ms, ax, ay, az, gx, gy, gz, mx, my, mz
```

Firmware units are g, degrees/s and microtesla. Startup `accel_scale`, `gyro_scale`
and `mag_scale` multiply incoming values to those units. Raw-count scales depend
on firmware settings; do not guess them. Extra/missing columns are rejected.
If valid samples do not arrive, the backend sends one Space resume toggle.
WSL attachment uses [ensure_imu_usb.bash](../scripts/ensure_imu_usb.bash), default
hardware ID `1B4F:9D0F`; override with `SAPROG_IMU_USB_BUSID` or
`SAPROG_IMU_USB_HARDWARE_ID`. Explicit device selection uses `SAPROG_IMU_DEVICE`
or the launch argument. Servo ports are never selected automatically.

Gyro integration uses sensor timestamps and quaternions. Gravity correction is
suppressed above 45 degrees/s or acceleration magnitude more than 8% from resting
gravity; it resumes after 250 ms with a 0.5 s time constant. These heuristics cannot
recover clipped rates/missing rotations or distinguish all translation from tilt.
Magnetic heading is optional/off by default; without it yaw is relative and drifts.
Timestamp gaps, peak gyro rate and sample frequency are diagnostics, not calibration.

**Zero** stores `q_zero`; display rotation is `inverse(q_zero) * q_current`. It
changes the reference, not accuracy. **Calibrate gyro + zero** needs four seconds
and at least 20 stationary samples, rejects noise/motion/bad timing, averages gyro
bias and saves zero. It does not calibrate magnetic distortion or sensor scales.
Heading-mode changes reset the estimate/zero while retaining gyro bias.

## ROS interface

Node: `/saprog_imu`. Topics below share `/saprog/imu/`; sensor topics use
best-effort sensor-data QoS, status is reliable at 5 Hz.

| Suffix | Type | Contract |
| --- | --- | --- |
| `data_raw` | `sensor_msgs/Imu` | Uncorrected gyro/acceleration; orientation unavailable |
| `data` | `sensor_msgs/Imu` | Absolute estimated attitude, bias-corrected gyro, measured acceleration |
| `mag` | `sensor_msgs/MagneticField` | Magnetic field in tesla |
| `orientation` | `geometry_msgs/QuaternionStamped` | Rotation relative to zero, or local world without zero |
| `status` | [ImuStatus](../src/saprog/msg/ImuStatus.msg) | Connection, calibration, reference and timing diagnostics |
| `serial` | `std_msgs/String` | Original lines, including rejected ones |

Standard sensor headers use ROS reception time and `imu_link`; data uses SI units
(m/s^2, rad/s, tesla). Acceleration includes the gravity response. Covariances are
unknown (zero); unavailable raw orientation has covariance element 0 set to -1.
The relative quaternion names `imu_zero` or `imu_world`; these are local references.

Services under the same prefix: `connect` ([ConnectImu](../src/saprog/srv/ConnectImu.srv)),
`use_magnetometer` (`SetBool`), and `disconnect`, `refresh_ports`, `toggle_readings`,
`zero`, `clear_zero`, `calibrate` (`Trigger`). Empty device detects the SparkFun IMU;
baud 0 uses its parameter. Runtime changes use services; parameters configure startup:
`device`, `baud_rate`, `frame_id`, `auto_connect`, `use_magnetometer`, `usb_helper`,
`accel_scale`, `gyro_scale`, `mag_scale`.

## Run and verify

After building/sourcing the workspace:

```sh
ros2 launch saprog imu.launch.py
```

Alternatively, start the backend alone and open a separate display:

```sh
ros2 launch saprog imu.launch.py gui:=false device:=/dev/ttyACM0 baud_rate:=115200
ros2 run saprog imu_monitor
ros2 topic echo /saprog/imu/status
ros2 service call /saprog/imu/zero std_srvs/srv/Trigger '{}'
```

Use one backend per device. Closing a display leaves the launch/backend running;
stop standalone launch with Ctrl+C. Use `auto_connect:=false` for manual connection.

RViz2 currently copies display orientation to the robot base. It does not apply
a measured IMU mounting transform; zero is not a substitute. Floor/height are
illustrations, not measured position. See [#4](https://github.com/ensapra/saprog/issues/4).
[Filter tests](../src/saprog/test/imu_filter_test.cpp) cover parsing, rotations,
zero and calibration; physical units, mounting, clipping and serial lifecycle still
need validation. Library/transport follow-up: [#5](https://github.com/ensapra/saprog/issues/5).
