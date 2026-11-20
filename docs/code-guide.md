# Code guide

This guide explains how the Python desktop layer and the C++ ROS 2 layer fit
together. Read [README.md](README.md) first for project invariants, then use
this page to locate the code to change.

## Mental model

There are three main execution owners:

1. **The C++ servo backend** opens the DYNAMIXEL serial port, publishes
   joint/temperature telemetry and executes ROS commands.
2. **The Python desktop** provides Tk windows, validates files, publishes
   commands, calls services/actions and schedules `rclpy`.
3. **Separate Python or launch processes** provide simulation, robot view,
   standalone monitors and the static animator.

Normal desktop windows do not create ROS executor threads. Home and each
standalone startup function own a `RootRosScheduler`, which calls
`rclpy.spin_once()` periodically while Tk remains in control of the UI thread.
This prevents Tk and ROS callbacks from racing for the same state.

## Top-level process map

```text
ROS/DYNAMIXEL backend
  saprog_nodes
    ReadbackNode  ---- /saprog/joint_states, /saprog/temperatures
    PositionControllerNode
      /saprog/position_command --> synchronized servo writes
      /saprog/servo_<id>/torque --> individual torque changes
      /saprog/refresh_limits --> read and publish measured servo limits
      /saprog/play_timeline --> PlayTimeline action

Python desktop
  Home
    temperature + IMU monitors
    snapshot creator/player/timeline
    controller
    robot/simulation tabs
    launcher/process ownership

  standalone startup functions
    snapshot creator/player/timeline
    temperature monitor
    IMU monitor
```

The process that starts a view is responsible for closing its nodes and backend
processes. Read [operation.md](operation.md) before changing lifecycle behavior.

## C++ ROS implementation

### Package build and interfaces

[CMakeLists.txt](../src/saprog/CMakeLists.txt) is the package build map:

- `saprog_nodes` combines the readback and position-controller executables.
- `imu_node` is a separate C++ executable for serial IMU data.
- `robot_view_embed` is the optional Qt/RViz embedded renderer.
- `rosidl_generate_interfaces` generates code for the action, message and service
  declarations in `action/`, `msg/` and `srv/`.
- The test section installs pure-Python, Tk/ROS, hardware-stub and native C++
  test targets. It is not a runtime dependency list.

[package.xml](../src/saprog/package.xml) declares runtime/build dependencies.
Update it when a new ROS or system package is required.

### Process startup

[main.cpp](../src/saprog/src/main.cpp) creates the readback node and controller
node, gives both nodes the same `DynamixelBus`, and runs a single-threaded
executor. A single executor is intentional: callback serialization prevents
multiple position/torque/telemetry transactions from using the serial port at
the same time.

The timeline executor is separate. It runs in a worker thread, but each serial
transaction takes the bus lock in [dynamixel_bus.cpp](../src/saprog/src/dynamixel_bus.cpp).
That lock is the real owner boundary for hardware writes.

### Servo units and configuration

[servo_units.hpp](../src/saprog/include/saprog/servo_units.hpp) converts:

- AX raw ticks `0..1023` to/from ROS radians.
- AX speed units to the `JointState.velocity` convention.

The zero and direction values are robot calibration; do not replace those with
assumed URDF angles inside a view. [config.hpp](../src/saprog/include/saprog/config.hpp)
stores ordered servo configuration and validates unique IDs, directions and limits.
[saprog.yaml](../src/saprog/config/saprog.yaml) supplies the startup values.

Measured CW/CCW limits belong to the controller, not the YAML file or the
desktop. `refresh_limits()` reads them from hardware and publishes them as ROS
parameters. The desktop decodes those parameters through
[snapshot_config.py](../src/saprog/scripts/saprog_desktop/snapshot_config.py).

### Readback node

[readback_node.cpp](../src/saprog/src/readback_node.cpp) owns no independent
serial hardware; it receives the shared bus from `main.cpp`.

Each timer callback advances to one configured servo. It reads position, torque
and temperature for that servo in one register transaction. After all servos
complete, it publishes one complete `sensor_msgs/JointState` snapshot. A failed
servo is omitted rather than reusing stale position data.

It publishes temperatures once per second per servo; the full joint-state sweep
is published at the configured poll rate.

### Position controller node

[position_controller_node.cpp](../src/saprog/src/position_controller_node.cpp)
is the command and timeline policy layer.

Important interfaces:

- `/saprog/position_command` accepts `sensor_msgs/JointState`.
- `/saprog/servo_<id>/torque` enables/disables one servo.
- `/saprog/refresh_limits` reads measured limits.
- `/saprog/play_timeline` accepts `saprog/action/PlayTimeline.action`.

Topic commands are converted from ROS radians to AX ticks and checked against
measured limits. Optional `velocity` and `effort` fields supply an equal speed
and per-joint torque values. A topic command is the public, low-level interface;
the desktop usually adds file validation and ROS action sequencing around it.

The action is a stricter interface. The goal is fully validated before a worker
starts: joint names must match, arrays must be row-major, motion values must be
finite and within limits, waits must be timed, and looping cannot include an
end torque action.

### Timeline execution

[timeline_executor.hpp](../src/saprog/include/saprog/timeline_executor.hpp)
contains a pure sequencing policy. It receives hook functions for:

- cancellation;
- monotonic waiting;
- sending motion;
- checking arrival;
- applying the final torque action.

This split keeps scheduling rules testable without ROS or USB. ROS-specific
deadline feedback and telemetry waits remain in `position_controller_node.cpp`.
`PlayTimeline.action` is the wire contract in
[PlayTimeline.action](../src/saprog/action/PlayTimeline.action).

Motion steps use one of two duration modes:

- **Until reached**: wait for fresh joint-state telemetry within tolerance.
- **Fixed seconds**: dispatch the command, then advance after its deadline
  regardless of servo arrival.

Wait steps send no commands. A timeline loop repeats motion/wait steps and skips
the end action. Cancellation can stop regular steps but is rejected after the
final torque action has started.

### Dynamixel serial bus

[dynamixel_bus.hpp](../src/saprog/include/saprog/dynamixel_bus.hpp) and
[dynamixel_bus.cpp](../src/saprog/src/dynamixel_bus.cpp) are the only servo
write/read owner. `dynamixel_register_transport.cpp` adapts the DYNAMIXEL SDK
for one-register reads, writes and synchronized writes.

The bus has two safeguards:

- A mutex serializes every serial transaction.
- A setting cache avoids repeated torque/speed/torque-limit writes. The cache
  is updated only after a successful write; readback can invalidate it.

Timeline writes may request bus priority and deadline acquisition. This makes
timeline execution take turns with ROS callbacks, while avoiding a busy spin.
It is bounded scheduling, not hard real-time behavior.

A successful goal-register read proves that the register was accepted, not that
the servo physically reached the position. Full arrival is determined from later
joint-state telemetry.

### IMU node

[imu_node.cpp](../src/saprog/src/imu_node.cpp) owns SparkFun-style IMU serial
reading and filtering. [imu_transport.cpp](../src/saprog/src/imu_transport.cpp)
handles discovery and serial I/O. It publishes raw and filtered sensor messages,
orientation and status.

The node exposes:

- `/saprog/imu/connect` to change the detected device;
- `/saprog/imu/use_magnetometer`;
- `/saprog/imu/zero`, `/calibrate`, `/reset_zero` and `/reset_calibration`
  trigger services.

Calibrated values and mounting/reference behavior are backend-owned. The launch
and monitor are not authoritative for physical mounting transform. Details are
in [imu.md](imu.md).

## Python desktop implementation

### Bootstrap and dependency injection

[runtime.py](../src/saprog/scripts/saprog_desktop/runtime.py) contains
`DesktopDependencies` and `RootRosScheduler`.

`DesktopDependencies` is an explicit import bundle. It makes views testable
without importing ROS/Tk at module import time and allows tests to substitute
fakes. `load_desktop_dependencies()` performs the real imports and, on Linux,
can re-exec through ROS's setup environment if dependencies were unavailable.

`RootRosScheduler` replaces `root.after()` callbacks with tracked callbacks.
Tk callbacks remain cancellable, and `rclpy.spin_once()` is called fairly across
registered nodes with a small budget.

[startup.py](../src/saprog/scripts/saprog_desktop/startup.py) is the thin process
entry layer:

- initialize `rclpy`;
- create one or more nodes;
- construct the Tk root/view;
- register nodes with the scheduler;
- start `root.mainloop()` and clean up.

Each legacy script in `scripts/` imports or wraps the implementation from
`saprog_desktop`. Canonical desktop apps live under `saprog_desktop/apps/`;
standalone ROS/Tk entry points use `saprog_desktop/startup.py` for dependency
loading, node/root/scheduler setup and teardown. Keep legacy executable names
working through forwarding shims for external scripts during the compatibility
cycle.

### Home

[home.py](../src/saprog/scripts/saprog_desktop/apps/home.py) is the desktop
workspace assembly and process owner.

It creates the main tabs, embedded monitors, static animator, controller,
simulation view and robot view. It also:

- starts/stops the shared servo backend;
- starts/restarts the IMU backend;
- launches `.sh` tools from `tabs/`;
- tracks child processes;
- observes the latest torque state;
- owns close and detach/restore behavior for notebook tabs.

Home changes need an entry in `tabs/` only for a launchable external tool.
Embedded tools belong as a tab or a view inside the existing assembly.

### Views and shared modules

Views in [views/](../src/saprog/scripts/saprog_desktop/views/) own Tk layout and
the high-level user workflow. Shared modules under
[saprog_desktop/](../src/saprog/scripts/saprog_desktop/) own reusable logic,
not a Tk root.

Important modules:

| Module | Responsibility |
| --- | --- |
| `snapshot_units.py` | AX/radian conversion, measured bounds and snapshot validation |
| `snapshot_files.py` | snapshot directory discovery and atomic JSON writes |
| `snapshot_config.py` | ROS limits/parameters refresh state machine |
| `snapshot_window.py` | live snapshot creator |
| `snapshot_player.py` | snapshot creator/player UI and playback state |
| `playback.py` | snapshot arrival/dwell coordination |
| `torque.py` | bounded asynchronous per-servo torque requests |
| `timeline_domain.py` | pure timeline validation/control-group rules |
| `timeline_storage.py` | timeline load/save/schema migration and snapshot references |
| `timeline_action.py` | Python data shape for `PlayTimeline.action` |
| `timeline_presenter.py` | timeline UI calculations/presenter state |
| `motion_controls.py` | shared Tk sliders and JointState message construction |
| `desktop_style.py` | common colors, styles and Tk widgets |

Prefer the first existing owner shown in this table or in
[reusability.md](reusability.md). Do not duplicate unit conversion or timeline
serialization in a view.

### Snapshot format and playback

Snapshots are JSON files under `snapshots/` or a `--directory` override.
Schema 1 stores radian positions implicitly. Schema 2 declares `radians` units.
Other units are rejected.

The validation boundary is `validate_snapshot()` in `snapshot_units.py`. It
does not mutate a document and validates radian values against supplied bounds.

Snapshot playback in [snapshot_player.py](../src/saprog/scripts/saprog_desktop/views/snapshot_player.py)
publishes one `JointState` command containing the selected joints/targets.
It does not wait for physical arrival and it does not automatically release
torque. `PlaybackCoordinator` in `playback.py` is optional UI-level monitoring
for windows that need arrival/failure feedback.

### Torque operations

Torque is a service request per joint, not a global state write. The Python
`TorqueBatch` in [torque.py](../src/saprog/scripts/saprog_desktop/torque.py)
tracks one request per joint and reports:

- success;
- response failure;
- timeout;
- unavailable service;
- not sent.

The batch remains bounded and callbacks carry an identity/token so stale
callbacks cannot change a later request. UI code polls `.done` and reads
`.outcomes`.

Closing a window does not disable torque.

### Timeline format and playback

Timelines are schema-2 documents containing snapshot references, not copied
poses. [timeline_storage.py](../src/saprog/scripts/saprog_desktop/timeline_storage.py)
loads and validates those references. [timeline_action.py](../src/saprog/scripts/saprog_desktop/timeline_action.py)
flattens validated timeline steps into the action arrays.

Flat arrays are intentionally row-major:

```text
positions[step * joint_count + joint_index]
speeds[step * joint_count + joint_index]
torque_limits[step * joint_count + joint_index]
```

Do not append an element to `positions`, `speeds` or `torque_limits` without
appending matching metadata and values for every joint.

The UI timeline view may use a legacy compatibility module, but new code should
rely on `timeline_domain`, `timeline_storage` and `timeline_action` as the pure
validation/serialization boundary.

### Robot view

[robot_view.py](../src/saprog/scripts/saprog_desktop/views/robot_view.py) owns
Tk embedding and process state. [robot_view_session.py](../src/saprog/scripts/saprog_desktop/robot_view_session.py)
starts/stops the RViz process and chooses the target window.

[robot_view_bridge.py](../src/saprog/scripts/saprog_desktop/ros/robot_view_bridge.py)
is the live ROS process that receives IMU and joint state, publishes TF and
visualization markers. It is not a desktop-only module and should not depend on
Tk.

The URDF/robot-state boundary is launch configuration:
`robot_view.launch.py` loads the installed model and starts both the bridge and
embedded RViz.

### Simulation

Simulation is hardware-independent:

- [simulation.launch.py](../src/saprog_simulation/launch/simulation.launch.py)
  starts Gazebo and optionally a playback worker.
- [simulation_session.py](../src/saprog/scripts/saprog_desktop/simulation_session.py)
  manages the launch/process from the Tk view.
- [simulation_worker.py](../src/saprog/scripts/saprog_desktop/simulation_worker.py)
  consumes control files and reports state.
- [simulation_playback.py](../src/saprog/scripts/saprog_desktop/simulation_playback.py)
  updates Gazebo joints and maps snapshots/timelines.

When changing physics or URDF behavior, validate the Gazebo tests and update
[simulation.md](simulation.md).

### IMU and temperature views

`views/imu_monitor.py` subscribes to IMU topics and creates the compass/orientation
visualization. `imu_model.py`, `imu_view.py` and `compass_view.py` keep geometry
and drawing outside Tk subscription/update code.

`temperature_monitor.py` and `TemperatureHistory` own rolling temperature
history. Views should poll a model or render callback instead of storing and
transforming all sensor data inside the Tk class.

## ROS interfaces

The principal interfaces are:

| Interface | Type | Purpose |
| --- | --- | --- |
| `/saprog/position_command` | `sensor_msgs/JointState` | low-level position/speed/torque command |
| `/saprog/joint_states` | `sensor_msgs/JointState` | complete latest position/torque sweep |
| `/saprog/temperatures` | `sensor_msgs/Temperature` | one servo temperature reading |
| `/saprog/refresh_limits` | `std_srvs/Trigger` | measure and publish servo limits |
| `/saprog/servo_<id>/torque` | `std_srvs/SetBool` | enable/disable one servo |
| `/saprog/play_timeline` | `saprog/action/PlayTimeline` | execute timeline steps |
| `/saprog/imu/*` | sensor/status/services | IMU streaming, calibration and connection |

Use names and units consistently:

- positions in `JointState.position` are radians;
- `velocity` on topic commands is encoded in AX speed units via
  `saprog::ax::speed_unit_radians`;
- `effort` on topic commands is the raw AX torque limit `0..1023`;
- temperatures are Celsius.

## Launchers and process ownership

Shell launchers in [tabs/](../tabs/) discover external tools for Home.
[_launch_window.bash](../tabs/_launch_window.bash) provides shared setup,
readiness and process cleanup.

Before adding a launcher, decide whether it:

1. needs the shared controller;
2. owns the controller itself;
3. owns only a separate backend (for example IMU); or
4. is hardware-free (for example simulation).

Do not move PID/process cleanup into a view. Ownership belongs in Home or the
startup/launcher layer. A window that closes itself must not silently disable
hardware; the desktop documents and tests this distinction.

## Where to change what

| Change | Start here |
| --- | --- |
| add a ROS topic or service | C++ node declaration, package/interface, Python client |
| change AX conversion | `servo_units.hpp`, `snapshot_units.py`, tests |
| add a servo parameter | `config.hpp`, `config_parameters.hpp`, YAML, tests |
| change limit measurement | `dynamixel_bus.cpp`, controller refresh, `snapshot_config.py` |
| change snapshot schema | `snapshot_units.py`, `snapshot_storage`/player, tests |
| change timeline timing | `timeline_executor.hpp`, controller waits, timeline Python modules |
| change torque UI behavior | `torque.py`, snapshot/creator/player view, tests |
| add a desktop tool | view module, startup assembly, optional `tabs/*.sh` launcher |
| change robot rendering | URDF/launch, bridge and robot-view session |
| change Gazebo behavior | simulation launch, worker/playback and Gazebo tests |
| change IMU calibration/status | `imu_node.cpp`, transport/filter, monitor/model |

## Modification checklist

Before changing a shared behavior:

1. Find the existing owner in [reusability.md](reusability.md).
2. Change the pure/lowest layer first (unit conversion, validation or executor).
3. Update both C++ and Python representations of the same interface.
4. Keep hardware ownership in the backend/launcher and UI state in the view.
5. Add a focused test that fails if the behavior is removed.
6. Run the smallest relevant suite, then a build if C++ or interfaces changed.

See [testing.md](testing.md) for exact commands and build locations.
