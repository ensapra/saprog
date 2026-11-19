# IMU contract and limitations

[imu_node.cpp](../src/saprog/src/imu_node.cpp) owns the serial device, USB helper,
parsing and ROS services. [imu_filter.hpp](../src/saprog/include/saprog/imu_filter.hpp)
owns attitude, gyro bias and zero. The [monitor](../src/saprog/scripts/saprog_desktop/views/imu_monitor.py)
only displays data/calls services; closing it preserves backend state. Reconnect
or backend restart resets gyro calibration and zero; saved magnetometer calibration
persists. Home starts/stops its own backend.

## Input and estimation

SparkFun calculated output defaults to 115200 baud and exactly ten finite columns:

```text
time_ms, ax, ay, az, gx, gy, gz, mx, my, mz
```

Firmware units are g, degrees/s and microtesla. Startup `accel_scale`, `gyro_scale`
and `mag_scale` multiply incoming values to those units. Raw-count scales depend
on firmware settings; do not guess them. Extra/missing columns are rejected.
The backend keeps retrying discovery and connection while auto-connect is enabled.
It reconnects after serial errors, a stopped stream, or a connection that produces
no valid samples; on WSL it also retries USB attachment. A manual `disconnect`
service disables this recovery until a new connection is requested. If valid
samples do not arrive, the backend sends one Space resume toggle per connection.
WSL attachment uses [ensure_imu_usb.bash](../scripts/ensure_imu_usb.bash), default
hardware ID `1B4F:9D0F`; if usbipd reports the IMU is not shared, it requests
the one-time `usbipd bind` through Windows UAC before attaching. Approve the
administrator prompt; if it is unavailable or denied, bind the device manually
in elevated PowerShell. Recovery retries after 30 seconds to avoid repeating
the prompt continuously. Override selection with `SAPROG_IMU_USB_BUSID` or
`SAPROG_IMU_USB_HARDWARE_ID`. Explicit device selection uses `SAPROG_IMU_DEVICE`
or the launch argument. Servo ports are never selected automatically.

Gyro integration uses sensor timestamps and quaternions. Gravity correction is
suppressed above 45 degrees/s or acceleration magnitude more than 8% from resting
gravity; it resumes after 250 ms with a 0.5 s time constant. These heuristics cannot
recover clipped rates/missing rotations or distinguish all translation from tilt.
Magnetic heading is optional/off by default; without it yaw is relative and drifts.
Timestamp gaps, peak gyro rate and sample frequency are diagnostics, not calibration.

**Zero** stores `q_zero`; display rotation is `inverse(q_zero) * q_current`. It
changes the reference, not accuracy. **Clear zero** removes that offset and
returns to the filter's local world reference; it does not calibrate sensor
mounting. **Calibrate gyro + zero** needs four seconds
and at least 20 stationary samples, rejects noise/motion/bad timing, averages gyro
bias and saves zero. It does not calibrate magnetic distortion or sensor scales.
Heading-mode changes reset the estimate/zero while retaining gyro bias.

## Magnetometer calibration

In the IMU monitor, click **Calibrate magnetometer**. Move the sensor slowly
through all orientations for about 20–60 seconds: turn every face up, including
upside down, and sweep each axis. Keep it away from magnetic objects and changing
motor currents. Then click **Save calibration**. A figure eight on a flat surface
alone does not provide the required three-dimensional coverage. The monitor
shows sample count and rejection reasons. If coverage is incomplete, keep rotating
and save again; **Cancel** keeps the previous correction.

The backend requires at least ten seconds, 200 samples, coverage of all eight
octants and six axis directions, plausible field ranges and a consistent corrected
field magnitude. Collection is bounded to 12,000 samples and two minutes, cancels
on disconnect, and cannot run alongside gyro calibration. These checks reject
obvious poor data; they do not certify heading accuracy or fully model soft iron.
Apply calibration with the sensor in its intended magnetic environment; changing
mounting, nearby metal or the sensor itself can require recalibration.

Successful calibration is saved atomically to
`~/.ros/saprog/magnetometer_calibration.txt` in the Linux backend's home directory.
The startup parameter `magnetometer_calibration_file` selects another file; use
separate files for different sensors/installations. The profile records `mag_scale`
and is rejected if that startup scale changes. Invalid profiles are reported and
raw magnetic readings are used. Save failures, rejected data and cancellation
preserve the previous correction. Calibration survives reconnects, backend restarts
and closing the monitor. Saving restarts attitude estimation and clears zero while
retaining gyro bias. Both attitude estimation and `/saprog/imu/mag` use the corrected
field; the `serial` topic remains the original input.

Trigger services under `/saprog/imu/` are `start_magnetometer_calibration`,
`finish_magnetometer_calibration`, and `cancel_magnetometer_calibration`.
`ImuStatus` includes magnetometer calibration activity, validity, sample count and
a separate message. Rebuild the ROS package and restart backend and displays after
updating this message interface.

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

The Robot view currently copies the orientation topic to `world -> base` without
applying a sensor-to-base mounting transform. With zero set, the topic is relative
to the saved `imu_zero` reference; after clear-zero it is relative to the filter's
local `imu_world` reference. Neither reference is a measured robot/world alignment.
The frame label on the topic is informational here: the bridge does not compose
that reference with a calibrated mounting rotation. When no orientation arrives
for 0.5 seconds, the view resets `world -> base` to identity rather than holding
the last tilt indefinitely. Floor and height are illustrations, not measured
position. A measured mounting transform and a declared display reference remain
open under [#4](https://github.com/ensapra/saprog/issues/4).
[Filter tests](../src/saprog/test/imu_filter_test.cpp) cover parsing, rotations,
zero and calibration; physical units, mounting, clipping and serial lifecycle still
need validation. Library/transport follow-up: [#5](https://github.com/ensapra/saprog/issues/5).

The project supports ROS 2 Jazzy. The unused ROS 1 `display.launch` and
`gazebo.launch` exports were removed; they used catkin-era package names and
could not run from this ament package. The maintained robot display entry point
is `ros2 launch saprog robot_view.launch.py` (the embedded Home view uses the
same launch file).

## Library evaluation (#5)

No estimator or trajectory dependency is adopted by this refactor. The current
estimator has behavior-specific tests for timestamped gyro integration, gated
gravity correction, optional magnetic heading, zero, and stationary gyro-bias
calibration. Replacing it with a generic Madgwick or complementary filter would
change at least some of those transitions and requires replay comparisons against
the existing tests plus representative recorded sensor traces. The repository
does not yet contain a recorded hardware trace set or mounting/calibration
acceptance data, so an estimator replacement cannot be shown equivalent.

`imu_filter_madgwick` and `imu_complementary_filter` are the relevant ROS
orientation-filter candidates for a later comparison. `robot_localization` is a
multi-sensor EKF/UKF layer rather than a drop-in serial-sample estimator. Any
candidate should first be run against the existing replay cases and recorded
samples, then compared for reset, zero, stale-data, calibration, and magnetic
heading behavior. This pass does not claim an up-to-date maintenance audit of
those external projects.

For timeline execution, ROS 2 `joint_trajectory_controller` is the relevant
maintained controller candidate; it expects the `ros2_control` hardware
interface and trajectory goal lifecycle. saprog currently owns a single shared
DYNAMIXEL bus, per-servo calibration/measured limits, torque services, and its
timeline final-action policy. Moving execution to that controller would require
a hardware plugin and regression coverage for those policies, so it is not a
safe dependency swap within the transport/estimation extraction. MoveIt Servo is
also not a direct replacement: it streams Cartesian/joint commands and brings a
planning-scene/configuration layer that this fixed-joint timeline does not use.

The decision is to retain the tested local estimator and controller until
recorded sensor traces and a `ros2_control` hardware adapter provide evidence for
an equivalent migration. No new runtime dependency is added.
