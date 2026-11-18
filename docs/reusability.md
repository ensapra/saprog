# Reusable functions and ownership map

Read this map before adding a helper or a new dependency between modules.
Check the linked source for the exact signature and current behavior. This maps
existing APIs; proposed replacements belong in the [refactor review](refactor-review.md)
and its implementation issues until they exist.

After adding, moving, changing or removing shared APIs, update this map in the
same commit: include source, responsibility, input/output and units, one actual
caller or usage example, dependency/lifetime constraints and replaced APIs.
Do not inventory every private method. Keep one owner per shared behavior.

## Pure snapshot and playback rules

| Source / API | Responsibility and input/output | Current usage / constraints |
| --- | --- | --- |
| [snapshot_units.py](../src/saprog/scripts/saprog_desktop/snapshot_units.py): `joint_bounds(ids, zeros, directions, lower, upper)` | Measured encoder/calibration arrays to ordered radian bounds by servo name. | Creator/Player decode controller configuration through this helper. Requires complete equal-length arrays, unique IDs and valid measured limits. |
| Same source: `to_radians(value, bounds, unit)`, `from_radians(value, bounds, unit)` | Radians or legacy normalized values to/from a joint's radian range. | Creator validates typed targets with `to_radians(value, bounds[joint], "radians")`. Conversion is not a hardware command. |
| Same source: `validate_snapshot(document, expected_names=None, bounds=None)` | Schema 1/2 document to `(trimmed_name, radian_positions)`; does not mutate input. | Creator/Player loads and timeline reference resolution. Normalized values require measured bounds. Offline radian validation without bounds checks structure, not reachability. |
| [snapshot_arrival.py](../src/saprog/scripts/saprog_desktop/snapshot_arrival.py): `arrival_progress(playback, positions, joints, tolerance, now)` | Updates progress fields in the playback dictionary; returns within/stalled/detail. Adds half an encoder tick to tolerance. | Creator/Player call only after checking fresh complete post-command telemetry. Caller owns timestamp ordering, stable dwell and arrival timeout. This is not the complete playback state machine. |

Example current snapshot validation:

```python
from saprog_desktop.snapshot_units import validate_snapshot

name, targets = validate_snapshot(document, configured_joints, measured_bounds)
# Validation alone sends no commands.
```

The Python AX range is currently in snapshot_units; snapshot_arrival also has a
tick constant; both AX constants now have their single Python owner in
snapshot_units. C++ uses the equivalent servo_units contract below.

| Source / API | Responsibility and input/output | Current usage / constraints |
| --- | --- | --- |
| [snapshot_config.py](../src/saprog/scripts/saprog_desktop/snapshot_config.py): `decode_configuration(values)` | Six ROS parameter values to a complete `SnapshotConfiguration` containing ordered integer servo IDs, `servo_<id>` names, radian bounds and nonnegative tolerance in radians. | Creator and Player receive only a fully validated result. It reuses `joint_bounds`; invalid/missing arrays and calibration/limit errors reject the whole result. |
| Same source: `SnapshotConfigurationClient(limits_client, parameter_client, trigger_type, on_result, ...)` | Refresh measured limits first, then request the six parameters; reports `(config, error)` through an injected callback. | Creator/Player poll it on their UI scheduler. It owns request deadlines/retry backoff and ignores replaced/closed request identities; callback must commit the full result atomically. `refresh()` forces a new hardware limit read. |
| [playback.py](../src/saprog/scripts/saprog_desktop/playback.py): `PlaybackCoordinator(joints, positions, tolerance, started_at, sent_stamp, ...)` | UI-independent movement-arrival state machine. `observe()` accepts fresh complete telemetry strictly newer than command and last sample; `tick()` returns `waiting`, `arrived`, `release` or `failed`. | Creator and Player share it after publishing their snapshot command. It applies `arrival_progress` and requires three distinct samples spanning at least one second before stable arrival. Caller still owns command publication, ROS timestamp conversion and the torque release action. Timeline sequence dwell remains a separate concern. |
| [snapshot_units.py](../src/saprog/scripts/saprog_desktop/snapshot_units.py): `LEG_JOINTS`, `AX_RANGE`, `AX_TICK_RADIANS` | Ordered leg-to-joint domain membership and AX 300-degree/tick constants. | Creator derives screen placement from `LEG_JOINTS`; timeline can import the same topology without importing Tk. AX tick tolerance in `snapshot_arrival` references these constants. |
| [torque.py](../src/saprog/scripts/saprog_desktop/torque.py): `TorqueBatch(clients, request_type, timeout, clock)` | Per-joint asynchronous SetBool requests with per-joint outcomes: `pending`, `success`, `failure`, `timeout`, or `not_sent`. | Creator uses one adapter per joint for Fix/Release; Player uses a batch for disable-all and post-arrival release. The adapter retains request objects, bounds requests, and retires callbacks after timeout/close so late responses cannot mutate newer state. The backend torque-on service reads fresh present position and commands a hold there; clients must not substitute a stale UI position. |

Typical view-owned use after the injected ROS clients are created:

```python
configuration.refresh()  # poll() performs limits, then parameter read
coordinator = PlaybackCoordinator(joints, targets, tolerance,
                                  started_at=sent_at, sent_stamp=ros_stamp)
outcomes = torque_batch.start(False)  # poll outcomes until no joint is pending
```

## Storage and preferences

| Source / API | Responsibility and input/output | Current usage / constraints |
| --- | --- | --- |
| [snapshot_files.py](../src/saprog/scripts/saprog_desktop/snapshot_files.py): `write_json_atomic(path, document)` | Write/fsync a neighboring temporary JSON file, close it, replace destination, remove temporary file. | Creator saves, timeline saves, Player preferences. Caller creates parent directory, validates content and decides overwrite. Nonfinite JSON is rejected. |
| Same source: `source_workspace()`, `default_snapshot_directory()` | Return source workspace root or `None`; select workspace snapshots in source checkouts and cwd/snapshots for installed execution. | Desktop CLI defaults. Snapshot data remains outside installed package resources. |
| [player_preferences.py](../src/saprog/scripts/saprog_desktop/player_preferences.py): `player_settings_path()`, `load_torque_strength(path)`, `save_torque_strength(path, value)` | OS-specific per-user preference path; invalid/missing strength falls back to 1023; save uses atomic JSON. Strength is an integer 0..1023. | Player and embedded Home Animator share the Player preference. Independent windows retain their existing preference behavior. |
| [snapshot_window.py](../src/saprog/scripts/snapshot_window.py): `safe_filename(name)` | Sanitize snapshot names to a bounded basename with fallback. | Creator save. Currently view-local; move to the storage owner if another caller needs it rather than importing Creator for filename handling. |

Example atomic storage, after validation/overwrite decisions:

```python
from pathlib import Path
from saprog_desktop.snapshot_files import write_json_atomic

path = Path(destination)
path.parent.mkdir(parents=True, exist_ok=True)
write_json_atomic(path, validated_document)
```

## Timeline rules, references and serialization

The compatibility facade is [timeline_model.py](../src/saprog/scripts/timeline_model.py).
Pure rules live in [timeline_domain.py](../src/saprog/scripts/saprog_desktop/timeline_domain.py),
reference I/O in [timeline_storage.py](../src/saprog/scripts/saprog_desktop/timeline_storage.py),
and the ROS-independent action serializer in [timeline_action.py](../src/saprog/scripts/saprog_desktop/timeline_action.py).

| API | Responsibility and input/output | Current usage / constraints |
| --- | --- | --- |
| `timeline_domain.py`: `wait_seconds(step)`, `transition_duration(step)`, `timeline_rows(steps)`, `control_groups(mode, joints)`, `motion_settings(step, joints)`, `regroup_motion(step, mode, joints=None)`, `lifecycle_settings(document)`, `validate_timeline(document, joints=None, bounds=None, snapshot_resolver=None)` | Pure validation and grouping rules; validation returns copied steps. Seconds are finite/nonnegative; speeds/strengths are integer 1..1023. | Timeline form/load/play/save. Backend chrono representability still needs #17. `control_groups` reuses `snapshot_units.LEG_JOINTS`; schema 2 validation receives a snapshot resolver. |
| `timeline_storage.py`: `make_snapshot_reference(path, directory)`, `load_snapshot_reference(reference, directory, joints=None, bounds=None)`, `reference_for_legacy_snapshot(snapshot, directory, joints=None, bounds=None)`, `resolve_timeline_steps(steps, snapshot_resolver)` | Path representation, constrained reference reads, legacy pose matching and deep-copy reference resolution. | Timeline add/playback/load. Relative references cannot escape the snapshot directory; absolute external references are supported. Identical legacy poses select stable filename order. |
| Same source: `load_timeline(path, directory, joints=None, bounds=None)` | Read, migrate schema 1 references, validate all references/steps, return a complete schema-2 document. | Timeline window assigns editor fields only after success, so failed reads/migrations leave editor state unchanged. |
| Same source: `save_timeline(path, steps, on_end="on", loop=False, snapshot_resolver=None)` | Validate schema-2 references then call `snapshot_files.write_json_atomic`. | Timeline save; caller chooses destination and overwrite policy. |
| `timeline_action.py`: `timeline_action_data(steps, joints, loop=False, run_end_action=True, end_torque_enabled=True)` | Resolved validated steps to `PlayTimeline.action` field dictionary in row-major order. | Timeline constructs the generated ROS Goal from result. Wait rows use fixed-width placeholders; loop suppresses end action. No ROS import in this serializer. |

Typical current reference validation:

```python
from saprog_desktop.timeline_storage import load_snapshot_reference
from saprog_desktop.timeline_domain import validate_timeline

def resolve(reference):
    return load_snapshot_reference(reference, snapshot_directory)

steps = validate_timeline(document, snapshot_resolver=resolve)
```

`snapshot_units.LEG_JOINTS` owns ordered servo membership for Creator and
Timeline; view-specific layout placement remains in Creator. Keep this separate
from URDF joint naming.

## Shared desktop presentation

| Source / API | Responsibility | Current usage / constraints |
| --- | --- | --- |
| [desktop_style.py](../src/saprog/scripts/saprog_desktop/desktop_style.py): `apply_style(root)`, palette | Root-owned theme, fonts and shared ttk style definitions. | Home and standalone roots. Embedded panes must not restyle/reconfigure the parent's root. Read [desktop-style.md](desktop-style.md). |
| Same source: `configure_changed(widget, **options)` | Configure only changed options. | Creator/Player/Timeline enablement; useful for frequently refreshed controls. |
| Same source: `status_box(parent, variable, *, tk_module=None, **options)` | Selectable read-only Text synchronized with a StringVar, preserving selection when unchanged. `tk_module` supplies the Tk toolkit; omitted only for legacy standalone calls. | Creator/Player/Timeline pass their injected runtime's `tk`; width/height are Text units, not pixels. Parent ownership stays with caller. |
| [motion_controls.py](../src/saprog/scripts/saprog_desktop/motion_controls.py): `add_slider(parent, text, variable, minimum, command=None, *, ttk_module=None)` | Build a rounded/clamped 0/1..1023 slider row; return the Scale. | Creator/Player/Timeline pass their injected runtime's `ttk`. Optional callback receives the rounded value as a string. Parent ownership stays with caller. |
| Same source: `set_motion(message, speed, strength)` | Fill per-name JointState velocity in rad/s and effort with raw torque strength. | Creator/Player publication; caller provides names/positions and validates eligibility. This is the project's command encoding, not measured effort. |

No shared scroll-frame component exists yet. Repeated canvas/content/scrollbar
construction in the three windows is an extraction candidate under #14; check
each resize contract before creating one.

## ROS and application ownership

| Current owner | API / responsibility | Reuse boundary |
| --- | --- | --- |
| [snapshot_config.py](../src/saprog/scripts/saprog_desktop/snapshot_config.py): `SnapshotConfigurationClient` | Shared config read lifecycle: force measured limit refresh, then bounded parameter read, validated by `decode_configuration`. | Creator/Player use it with ROS's `AsyncParameterClient`; it owns retry and stale request cleanup. Replaces the side-effectful `HardwareParameterClient.services_are_ready()` path in `snapshot_limits_client.py`. |
| [snapshot_player.py](../src/saprog/scripts/snapshot_player.py), [snapshot_window.py](../src/saprog/scripts/snapshot_window.py) | Playback/configuration/torque state currently lives inside each view. | Reuse arrival/unit helpers today. #10/#11/#12 introduce shared adapters/coordinator; do not copy private view methods into another view. |
| [snapshot_window.py](../src/saprog/scripts/snapshot_window.py): `SnapshotWindow(..., show_snapshot_loader, initial_torque_strength, torque_strength_changed, dependencies)` and `load_targets_from_file(path)` | Loader visibility and initial torque strength in raw 0..1023 units are configured before widgets are built; the optional callback receives rounded raw strength. Runtime dependencies supply Tk/ttk, ROS messages/services and dialogs. `load_targets_from_file` validates against configured joints and measured radian bounds, then fills editable targets and the snapshot name; returns true if loaded, false if rejected or queued, and sends no commands. | Home and Static Animator use the hidden loader and shared preference callback. The Creator must stay alive with its Tk root and ROS node. Selection before controller configuration queues the path until configuration completes. This replaces the combined view's duplicate Creator file picker/load controls. |
| [snapshot_player.py](../src/saprog/scripts/snapshot_player.py): `SnapshotPlayer(..., dependencies, on_pose_selected, moving_speed, torque_strength, hold_after_load, can_start_playback)` | Optional embedded selection mode; a selected file path is sent to the callback, while explicit Play starts its validated pose playback. Optional Tk variables share speed (1..1023), strength (0..1023), and hold policy with the Creator. | Home and standalone Static Animator use the Creator variables and an operation gate so playback cannot overlap Creator torque or snapshot-playback requests. The root owner supplies runtime dependencies and owns ROS scheduling/teardown. Callback and Tk variables must share the Player's UI-thread/root lifetime. The embedded mode replaces immediate-play list buttons, duplicate sliders/toggle and duplicate all-joints torque-release control; standalone mode keeps them. |
| [timeline_window.py](../src/saprog/scripts/timeline_window.py): `TimelineWindow(..., communication_timeout_ms=8000)` | Timeline editor with one action-goal session, using a positive communication timeout in milliseconds. | Goal response, cancel acknowledgment and result delivery waits are bounded independently; timeout marks the outcome unknown and keeps motion controls locked until a backend response arrives. The timeout never limits goal execution. Session identities discard stale callbacks. Stop before acceptance is remembered and cancellation is requested only after acceptance; an accepted cancel request is not treated as confirmed motion cancellation. Close waits for a terminal result. #14 still owns the view/presenter extraction. |
| [home.py](../src/saprog/scripts/home.py), [static_animator.py](../src/saprog/scripts/static_animator.py) | Home and standalone root owners provide explicit nodes/scheduler; `AnimatorViews` shares widget assembly and gates host teardown on Timeline's terminal-safe close callback. | #9/#16. The composition owns views only: host owns root, backend processes and scheduler; child views never destroy shared nodes or shut down ROS. Unknown Timeline outcomes retain the root and nodes. Closing the IMU display does not call reset or clear-zero services. |
| [static_animator.py](../src/saprog/scripts/static_animator.py): `build_animator_views(root, parent, creator_node, player_node, timeline_node, directory, *, dependencies, torque_strength_changed=None, on_closed=None)` and `AnimatorViews` | Build the two-tab notebook through the injected `ttk` toolkit with shared Creator/Player controls and separate Timeline; return an owner retaining Creator, Player, Timeline and composition widgets. `request_close()` delegates dirty-edit and action-cancellation decisions to Timeline. Terminal close triggers idempotent `dispose()`, closes Creator/Player adapters, then calls `on_closed`. Torque callback receives rounded raw strength in 0..1023. | Home and standalone Static Animator use the same assembly and pass the same runtime dependency instance to all views. Root and backend ownership remain with the host; `RootRosScheduler` owns callback cancellation, bounded ROS spinning, registered-node destruction and ROS shutdown. Widgets and callbacks run on the root's Tk thread. |

## Desktop startup and root scheduling

| Source / API | Responsibility and input/output | Current usage / constraints |
| --- | --- | --- |
| [runtime.py](../src/saprog/scripts/saprog_desktop/runtime.py): `DesktopDependencies`, `load_desktop_dependencies(args, require_timeline=False, entrypoint=None)` | Import Tk, ROS, message, service and optional action types into an immutable dependency bundle. `args` is the original CLI argument list; returns the bundle or exits with setup/missing-dependency diagnostics. | Standalone entry points call this before importing ROS-dependent views. On Linux it retains the `/opt/ros/$ROS_DISTRO/setup.bash` relaunch fallback and passes through arguments. Embedded views receive the same bundle from their root owner. |
| Same source: `RootRosScheduler(root, dependencies, nodes=(), interval_ms=10, spin_budget=16)`; `add_node(node)`, `start()`, `request_close()`, `shutdown()` | Own one Tk `after` loop for ROS callbacks per root. `interval_ms` is milliseconds; `spin_budget` is maximum nodes spun per scheduler cycle. Tracks root `after` callbacks, rotates through explicitly registered nodes, then cancels callbacks and destroys nodes before ROS shutdown. | Home and each standalone ROS/Tk window use this owner. Embedded views do not initialize/shutdown ROS or spin nodes; owners register additional nodes with `add_node`. Root teardown goes through `request_close()` and `shutdown()`. |
| [startup.py](../src/saprog/scripts/saprog_desktop/startup.py): `snapshot_player_main`, `snapshot_creator_main`, `temperature_monitor_main`, `imu_monitor_main`, `home_main` | Load dependencies, initialize ROS with CLI arguments where supported, create the root/node set, run the selected standalone view, and return status `0`. | Script entry points delegate here before importing ROS-dependent views so missing dependencies can use the sourced-ROS fallback and actionable diagnostics. `snapshot_player_main` also selects Timeline with `--timeline`; Static Animator has its own composition entry point. |
| [imu_model.py](../src/saprog/scripts/saprog_desktop/imu_model.py): `quaternion_to_euler(xyzw)`, `orientation_matrix(angles)` | ROS XYZW quaternion to display roll/pitch/yaw radians; roll/pitch/yaw radians to a 3x3 rotation basis. | `ImuMonitor.receive_orientation` uses it for labels and the IMU canvas. Display conversion only; fusion, zero and calibration remain in the C++ backend. |
| [imu_view.py](../src/saprog/scripts/saprog_desktop/imu_view.py): `draw_orientation(canvas, orientation, zeroed=False, magnetic_heading=False)` | Orientation radians and display state to projected canvas geometry, sized from current canvas width/height in pixels. | `ImuMonitor.render_model` calls it on telemetry refresh and `<Configure>`. It draws sensor-relative orientation and an illustrative ground/shadow plane, not altitude or a mounting transform. |
| [temperature_model.py](../src/saprog/scripts/saprog_desktop/temperature_model.py): `TemperatureHistory` | `add_sample(name, stamp, value)` stores monotonic seconds and °C in per-servo deques capped at 18,000 points; `visible(now)` trims the 300-second history and returns selected points in the current 1..300 second window plus live-servo count. `set_window` validates the range; `zoom(factor)` clamps it. | `TemperatureMonitor` owns one model; selecting all updates the pure selection state and future-servo default. Zoom changes the display window without discarding retained history. |
| [temperature_view.py](../src/saprog/scripts/saprog_desktop/temperature_view.py): `draw_temperature(canvas, history, colors, labels, now)` | Pure history state plus Tk canvas, per-servo colors/labels and monotonic seconds to a graph; returns the selected/live summary. | `TemperatureMonitor.draw` calls it on the root refresh and canvas resize. It owns axes, scaling, gaps for stale samples and line/point drawing; it does not subscribe to ROS. |
| [imu_monitor.py](../src/saprog/scripts/imu_monitor.py), [temperature_monitor.py](../src/saprog/scripts/temperature_monitor.py) | `ImuMonitor(..., dependencies)` and `TemperatureMonitor(..., dependencies)` are concise Tk presenters and ROS telemetry/service adapters. | Backend owns sensor state. The containing root scheduler spins their registered node and owns shutdown; embedded views do not spin or shut down shared nodes. IMU close retires pending service requests; callbacks ignore closed presenters. |
| [robot_view.py](../src/saprog/scripts/robot_view.py) | Tk controls, X11 eligibility checks and viewport for embedded RViz. | Delegates launch/readiness/process cleanup to `RendererSession`; Tk owns polling and widget updates. |
| [robot_view_session.py](../src/saprog/scripts/saprog_desktop/robot_view_session.py): `RendererSession(workspace, ros2="ros2")`, `start(parent_window, width, height)`, `poll()`, `stop()`, `close()` | Own one `ros2 launch saprog robot_view.launch.py` process group and its temporary readiness marker; `start` gets the X11 parent ID and pixel dimensions, `poll` returns the lifecycle state, and status text is available as `message`. | `RobotView` is the current caller. Call `poll` on the Tk thread; signal escalation and close-time wait run on daemon timers. The session owns the marker and cleans it on failure, exit or close. Replaces process state and signal handlers formerly embedded in `RobotView`. |

## C++ units, hardware and sensor boundaries

| Source / API | Responsibility and usage | Constraints |
| --- | --- | --- |
| [servo_units.hpp](../src/saprog/include/saprog/servo_units.hpp) | `saprog::ax::to_radians(raw, zero, direction)`, `to_raw(radians, zero, direction)` and position/tick/speed constants; controller/readback/bus use this contract. | AX 300 degrees, 0..1023 ticks. to_raw returns unrounded value so caller checks limits before rounding. |
| [config.hpp](../src/saprog/include/saprog/config.hpp) | `SaprogConfig`, `read_config_parameters(node)`, `validate_config(config)`: decode ROS startup values, then validate settings independently of ROS. Controller/readback use `read_config`; tests can validate constructed configurations directly. | Cached startup settings; measured limits are backend-owned and populated after bus read. Runtime parameter consistency still needs the controller policy in #18. |
| [servo_commands.hpp](../src/saprog/include/saprog/servo_commands.hpp): `position_tick(radians, zero, direction, lower, upper)` | Finite radian target and calibration/measured raw-tick limits to the nearest checked AX tick. | Timeline validation/execution and topic commands use the same conversion; callers retain their distinct subset/full-joint policies. Throws for invalid calibration/limits or out-of-range positions. No ROS or bus dependency. |
| [timeline_deadline.hpp](../src/saprog/include/saprog/timeline_deadline.hpp): `checked_deadline(seconds, start, deadline)` | Finite nonnegative seconds and a steady-clock start to a representable deadline; returns false on invalid or overflowing input. | Timeline goal validation and execution use the same checked conversion before deadline arithmetic. The caller injects the clock reading; this helper does not wait or access ROS. |
| [dynamixel_registers.hpp](../src/saprog/include/saprog/dynamixel_registers.hpp) | Named AX register addresses and widths for limits, torque, goal/present position, speed and torque limit. | `DynamixelBus` uses the constants for register transactions/cache keys; one bus still owns the serial transaction lock. |
| [dynamixel_bus.hpp](../src/saprog/include/saprog/dynamixel_bus.hpp), [implementation](../src/saprog/src/dynamixel_bus.cpp) | `DynamixelBus`: single serial transaction lock/priority arbitration, settings cache, and single/batch servo operations including before-deadline variants. Before-deadline calls accept an optional `bool()` cancellation predicate checked after lock acquisition and before writes. | Readback creates the bus and controller shares it. Cache mutations and transactions stay under bus ownership. Verification of goal registers does not prove physical arrival. The predicate runs under the bus lock and must be nonblocking. |
| [dynamixel_register_transport.hpp](../src/saprog/include/saprog/dynamixel_register_transport.hpp), [implementation](../src/saprog/src/dynamixel_register_transport.cpp) | `DynamixelRegisterTransport`: owns the one SDK port and performs checked register reads/writes and synchronized register batches. | Owned by `DynamixelBus`; do not construct a second transport for the same physical bus. Status-return-level behavior, timeout verification without duplicate writes, and SDK error translation live here. `DynamixelBus` owns cache and scheduling policy. |
| [imu_filter.hpp](../src/saprog/include/saprog/imu_filter.hpp) | `saprog::imu::Sample`, `parse_sample(line, sample)`, vector/quaternion helpers and `Filter` update/calibration/zero/heading operations. | Strict calculated ten-column CSV, firmware g/degrees/s/microtesla. Quat is wxyz sensor-to-world; ROS uses xyzw. #21 splits protocol/math/filter without replacing numeric behavior. |
| [imu_node.cpp](../src/saprog/src/imu_node.cpp) | Serial/helper lifetime, sample scaling and ROS SI publication/services. | Sensor state remains backend-owned. #20 extracts exclusive transport and helper owners; #21 owns message/unit boundary. |
| [robot_view_model.py](../src/saprog/scripts/saprog_desktop/robot_view_model.py): `SERVO_JOINT_NAMES`, `OrientationReference` | Map servo IDs to URDF joint names and provide the latest valid XYZW orientation while fresh, then identity after 0.5 seconds. | `robot_view_bridge.py` is the current ROS caller. Tests verify all mapped names exist in the URDF; preserve the export's `froint_right_joint_13` spelling. Orientation remains sensor-relative; no calibrated sensor-to-base transform is claimed. Replaces the bridge's unvalidated inline map and indefinitely-held quaternion. |
| [robot_view_bridge.py](../src/saprog/scripts/robot_view_bridge.py): `RobotViewBridge` | ROS adapter from `/saprog/joint_states` to `/joint_states` and `/saprog/imu/orientation` to `world -> base`. | Launched by the installed ROS 2 robot view launch file. It applies no mounting transform; orientation staleness resets the published transform to identity. |

C++ and Python necessarily implement equivalent unit contracts in separate
languages; keep regression fixtures aligned rather than crossing runtimes just
to eliminate a few arithmetic expressions.

Current backend extraction additions:

| Source / API | Responsibility and usage | Constraints |
| --- | --- | --- |
| [config.hpp](../src/saprog/include/saprog/config.hpp): `SaprogConfig`, `ServoCalibration`, `validate_config(config)`, `configured_servo_ids(config)`, `configured_servo_index(config, joint_name)`, `is_startup_only_parameter(name)` | Pure ordered servo configuration validation and joint-name/parameter-policy lookup. Topic commands may select a subset by name; timeline execution uses the full configured order. | No ROS dependency. Limits default to AX full range until controller refresh; validate the fully configured copy after hardware limits are populated. Invalid joint lookup returns `config.servos.size()`. |
| [config_parameters.hpp](../src/saprog/include/saprog/config_parameters.hpp): `read_config_parameters(node)` | Decode ROS startup parameters into `SaprogConfig`, then call pure validation. Controller and readback nodes use it during construction. | ROS adapter only; values are cached at startup. Runtime mutation policy stays with the controller callback. |
| [timeline_executor.hpp](../src/saprog/include/saprog/timeline_executor.hpp): `execute(plan, hooks, allowed_lateness)` | Deterministically sequence waits, scheduled motion dispatch, arrival waits, cancellation and the optional final torque action. Returns success/cancel/completed-step/error state. | Injects monotonic clock, cancellable wait, bus dispatch, telemetry-arrival and final-action hooks. The ROS controller owns feedback and fresh-sample rules; `DynamixelBus` owns its 250 ms bus acquisition deadline. A started final action is non-cancellable. |

## Shell and installed resources

[_launcher_setup.bash](../tabs/_launcher_setup.bash) owns ROS/workspace setup
and installed-package verification for [_launch_window.bash](../tabs/_launch_window.bash).
`saprog_source_ros(distro)` sources `/opt/ros/<distro>/setup.bash` into the
current process. `saprog_verify_workspace(workspace, config, executable?)`
sources that workspace's `install/setup.bash`, validates its package prefix,
and sets `SAPROG_PACKAGE_PREFIX`, `SAPROG_NODES_EXE` and optional
`SAPROG_WINDOW_EXE` to filesystem paths. `saprog_verify_home_backend_pid(pid)`
checks the inherited PID's Linux command line against this workspace's backend
launcher; it never signals or owns that process. The launcher owns build
locking, controller readiness and child-process cleanup:
Home owns its backend child and passes its PID to tool launchers; direct window
launchers start and stop their own backend; IMU launch owns its independent
backend; Home owns the embedded Robot view renderer and state processes. Keep
those lifetimes separate. The `verify` mode checks setup without starting
hardware or a backend. Actual call: `_launch_window.bash` invokes
`saprog_verify_workspace "$workspace_dir" "$config_file" "$window_name"` before
starting or reusing the controller.

Owned-child cleanup ignores repeated INT/TERM signals while it sends TERM, waits
up to two seconds per child, then sends KILL. A reused Home backend is not
signaled or reaped by a window launcher.

[usb_attach.bash](../scripts/usb_attach.bash): `saprog_usb_attach(label,
bus_override, bus_variable_name, hardware_variable_name, target)`
validates the selected BUSID or VID:PID, requires exactly one usbipd match,
attaches a shared device, and sets `SAPROG_USB_BUS_ID`,
`SAPROG_USB_HARDWARE_ID`, and `SAPROG_USB_STATE`; it returns status and writes
diagnostics to stderr. Example: `saprog_usb_attach servo "${SAPROG_USB_BUSID:-}"
SAPROG_USB_BUSID SAPROG_USB_HARDWARE_ID "${SAPROG_USB_HARDWARE_ID:-0403:6014}"`.
Call only from a WSL caller that checked `WSL_DISTRO_NAME`; the helper does not
select a serial tty or own backend lifetime. It is sourced by
[ensure_usb.bash](../scripts/ensure_usb.bash) and
[ensure_imu_usb.bash](../scripts/ensure_imu_usb.bash). It owns usbipd-win
discovery, override validation, uniqueness checks and attach mechanics. Callers
retain device policies: servo preflight validates the configured `device_name`
as a character device (also on native Linux); IMU preflight selects a tty by USB
VID/PID only under WSL and prints only that path to stdout. Diagnostics go to
stderr. The IMU helper is installed beside the shared sourced helper.

Keep installed resource registration in [CMakeLists.txt](../src/saprog/CMakeLists.txt)
and [description installation](../urdf/CMakeLists.txt) aligned with moves.
#8/#25 cover packaging and clean-install validation; the source checkout alone
does not establish that an entry point will work after installation.
