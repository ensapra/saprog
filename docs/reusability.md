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
| [snapshot_units.py](../src/saprog/scripts/snapshot_units.py): `joint_bounds(ids, zeros, directions, lower, upper)` | Measured encoder/calibration arrays to ordered radian bounds by servo name. | Creator/Player decode controller configuration through this helper. Requires complete equal-length arrays, unique IDs and valid measured limits. |
| Same source: `to_radians(value, bounds, unit)`, `from_radians(value, bounds, unit)` | Radians or legacy normalized values to/from a joint's radian range. | Creator validates typed targets with `to_radians(value, bounds[joint], "radians")`. Conversion is not a hardware command. |
| Same source: `validate_snapshot(document, expected_names=None, bounds=None)` | Schema 1/2 document to `(trimmed_name, radian_positions)`; does not mutate input. | Creator/Player loads and timeline reference resolution. Normalized values require measured bounds. Offline radian validation without bounds checks structure, not reachability. |
| [snapshot_arrival.py](../src/saprog/scripts/snapshot_arrival.py): `arrival_progress(playback, positions, joints, tolerance, now)` | Updates progress fields in the playback dictionary; returns within/stalled/detail. Adds half an encoder tick to tolerance. | Creator/Player call only after checking fresh complete post-command telemetry. Caller owns timestamp ordering, stable dwell and arrival timeout. This is not the complete playback state machine. |

Example current snapshot validation:

```python
from snapshot_units import validate_snapshot

name, targets = validate_snapshot(document, configured_joints, measured_bounds)
# Validation alone sends no commands.
```

The Python AX range is currently in snapshot_units; snapshot_arrival also has a
tick constant. Consolidate the Python unit constants during #11 rather than
adding another copy. C++ uses the equivalent servo_units contract below.

## Storage and preferences

| Source / API | Responsibility and input/output | Current usage / constraints |
| --- | --- | --- |
| [snapshot_files.py](../src/saprog/scripts/snapshot_files.py): `write_json_atomic(path, document)` | Write/fsync a neighboring temporary JSON file, close it, replace destination, remove temporary file. | Creator saves, timeline saves, Player preferences. Caller creates parent directory, validates content and decides overwrite. Nonfinite JSON is rejected. |
| Same source: `default_snapshot_directory()` | Search source/installed ancestors for the workspace; otherwise return cwd/snapshots as string. | Desktop CLI defaults. This is current behavior, not a general installed-resource resolver. |
| [snapshot_player.py](../src/saprog/scripts/snapshot_player.py): `player_settings_path()`, `load_torque_strength(path)`, `save_torque_strength(path, value)` | OS-specific preference path; invalid/missing strength falls back to 1023; save uses atomic JSON. | Player stores strength independently from other windows. Currently housed in a view module; extract to storage during #8/#9, not a new window dependency. |
| [snapshot_window.py](../src/saprog/scripts/snapshot_window.py): `safe_filename(name)` | Sanitize snapshot names to a bounded basename with fallback. | Creator save. Currently view-local; move to the storage owner if another caller needs it rather than importing Creator for filename handling. |

Example atomic storage, after validation/overwrite decisions:

```python
from pathlib import Path
from snapshot_files import write_json_atomic

path = Path(destination)
path.parent.mkdir(parents=True, exist_ok=True)
write_json_atomic(path, validated_document)
```

## Timeline rules, references and serialization

Current owner: [timeline_model.py](../src/saprog/scripts/timeline_model.py).
It is importable without ROS/Tk, but includes filesystem operations as well as
pure rules. #13 separates those owners.

| API | Responsibility and input/output | Current usage / constraints |
| --- | --- | --- |
| `wait_seconds(step)`, `transition_duration(step)` | Validate finite nonnegative seconds; normalize default transition mode/seconds. | Timeline form, document validation and action serializer. Backend chrono representability still needs #17; finite alone is not sufficient there. |
| `timeline_rows(steps)` | Visible row to whole-step index/kind mapping; transitions get paired snapshot rows. | Timeline list selection/redraw. Moving entries must preserve the pair. |
| `control_groups(mode, joints)`, `motion_settings(step, joints)`, `regroup_motion(step, mode, joints=None)` | Joint groups, resolved speed/strength, deep-copied regrouped step. | Timeline form and serialization. Coarser groups average current values; finer groups copy them. Speeds/strengths are integers 1..1023. |
| `lifecycle_settings(document)` | Validate/default on_end torque policy. | Timeline load and document validation. |
| `make_snapshot_reference(path, directory)` | Relative path within snapshot directory where possible, otherwise absolute path. | Add snapshot/migration. |
| `load_snapshot_reference(reference, directory, joints=None, bounds=None)` | Read and validate a current snapshot, returning schema-1 radian document. | Timeline resolver. Relative references cannot escape directory; absolute external references are supported. |
| `reference_for_legacy_snapshot(snapshot, directory, joints=None, bounds=None)` | Match embedded legacy pose to a file by name, joint set and radian positions. | Timeline load. Reject changed/missing poses; identical poses select stable filename order. Preserve completed #3 behavior. |
| `resolve_timeline_steps(steps, snapshot_resolver)` | Deep-copy steps and replace path references using injected resolver. | Timeline playback preparation re-reads current referenced poses. |
| `validate_timeline(document, joints=None, bounds=None, snapshot_resolver=None)` | Validate schema, steps, motion and lifecycle; return copied steps. | Timeline load/play/save. Schema 2 needs resolver; schema 1 can embed snapshots. Does not return lifecycle metadata or perform editor-state replacement. |
| `save_timeline(path, steps, on_end="on", loop=False, snapshot_resolver=None)` | Validate schema-2 reference document then write atomically. | Timeline save; caller chooses destination. |
| `timeline_action_data(steps, joints, loop=False, run_end_action=True, end_torque_enabled=True)` | Validated resolved steps to row-major action field dictionary. | Timeline constructs PlayTimeline.Goal from result. Fixed-width placeholder rows for waits; loop suppresses end action. No ROS import in this serializer. |

Typical current reference validation:

```python
from timeline_model import load_snapshot_reference, validate_timeline

def resolve(reference):
    return load_snapshot_reference(reference, snapshot_directory)

steps = validate_timeline(document, snapshot_resolver=resolve)
```

`LEGS` here and `LEG_LAYOUT` in Creator repeat membership. #13/#14 should share
the domain membership owner while leaving placement/labels in views. Do not add
a third leg definition or confuse servo membership with URDF joint naming.

## Shared desktop presentation

| Source / API | Responsibility | Current usage / constraints |
| --- | --- | --- |
| [desktop_style.py](../src/saprog/scripts/desktop_style.py): `apply_style(root)`, palette | Root-owned theme, fonts and shared ttk style definitions. | Home and standalone roots. Embedded panes must not restyle/reconfigure the parent's root. Read [desktop-style.md](desktop-style.md). |
| Same source: `configure_changed(widget, **options)` | Configure only changed options. | Creator/Player/Timeline enablement; useful for frequently refreshed controls. |
| Same source: `status_box(parent, variable, **options)` | Selectable read-only Text synchronized with a StringVar, preserving selection when unchanged. | Creator/Player/Timeline. Width/height are Text units; not pixel sizes. |
| [motion_controls.py](../src/saprog/scripts/motion_controls.py): `add_slider(parent, text, variable, minimum, command=None)` | Build a rounded/clamped 0/1..1023 slider row; return the Scale. | Creator/Player/Timeline. Optional callback receives string of rounded value. Parent ownership stays with caller. |
| Same source: `set_motion(message, speed, strength)` | Fill per-name JointState velocity in rad/s and effort with raw torque strength. | Creator/Player publication; caller provides names/positions and validates eligibility. This is the project's command encoding, not measured effort. |

No shared scroll-frame component exists yet. Repeated canvas/content/scrollbar
construction in the three windows is an extraction candidate under #14; check
each resize contract before creating one.

## ROS and application ownership

| Current owner | API / responsibility | Reuse boundary |
| --- | --- | --- |
| [snapshot_limits_client.py](../src/saprog/scripts/snapshot_limits_client.py) | `HardwareParameterClient(node, remote_node_name)` refreshes hardware limits before parameter readiness. | Creator/Player alias it as AsyncParameterClient. Its services_are_ready has polling side effects; it does not bound the later parameter fetch. #10 owns replacement. |
| [snapshot_player.py](../src/saprog/scripts/snapshot_player.py), [snapshot_window.py](../src/saprog/scripts/snapshot_window.py) | Playback/configuration/torque state currently lives inside each view. | Reuse arrival/unit helpers today. #10/#11/#12 introduce shared adapters/coordinator; do not copy private view methods into another view. |
| [snapshot_window.py](../src/saprog/scripts/snapshot_window.py): `SnapshotWindow(..., show_snapshot_loader, initial_torque_strength, torque_strength_changed)` and `load_targets_from_file(path)` | Loader visibility and initial torque strength in raw 0..1023 units are configured before widgets are built; the optional callback receives rounded raw strength. `load_targets_from_file` validates against configured joints and measured radian bounds, then fills editable targets and the snapshot name; returns true if loaded, false if rejected or queued, and sends no commands. | Home and Static Animator use the hidden loader and shared preference callback. The Creator must stay alive with its Tk root and ROS node. Selection before controller configuration queues the path until configuration completes. This replaces the combined view's duplicate Creator file picker/load controls. |
| [snapshot_player.py](../src/saprog/scripts/snapshot_player.py): `SnapshotPlayer(..., on_pose_selected, moving_speed, torque_strength, hold_after_load, can_start_playback)` | Optional embedded selection mode; a selected file path is sent to the callback, while explicit Play starts its validated pose playback. Optional Tk variables share speed (1..1023), strength (0..1023), and hold policy with the Creator. | Home and standalone Static Animator use the Creator variables and an operation gate so playback cannot overlap Creator torque or snapshot-playback requests. Omit these arguments to retain standalone Player behavior. Callback and Tk variables must share the Player's UI-thread/root lifetime. The embedded mode replaces immediate-play list buttons, duplicate sliders/toggle and duplicate all-joints torque-release control; standalone mode keeps them. |
| [timeline_window.py](../src/saprog/scripts/timeline_window.py) | Editor and timeline action session currently share a Player subclass. | #14/#15 remove inheritance/runtime-global coupling; these are not reusable action-client APIs yet. |
| [home.py](../src/saprog/scripts/home.py), [static_animator.py](../src/saprog/scripts/static_animator.py) | Root/node scheduling, animator assembly and close paths. | #9/#16 provide explicit composition/disposal; no child pane should call global ROS shutdown. |
| [static_animator.py](../src/saprog/scripts/static_animator.py): `build_animator_views(root, parent, creator_node, player_node, timeline_node, directory, *, torque_strength_changed=None, on_timeline_close=None)` | Build the two-tab notebook with shared Creator/Player controls and the separate Timeline; return `(creator, player, timeline)`. The torque callback receives rounded raw strength as decimal text in the 0..1023 range. | Home and the standalone Static Animator call it after ROS is initialized. The caller owns the root, three initialized nodes and their shutdown; widgets and callbacks run on the root's Tk thread. Replaces duplicate notebook and pane assembly in those entry points. |
| [imu_monitor.py](../src/saprog/scripts/imu_monitor.py), [temperature_monitor.py](../src/saprog/scripts/temperature_monitor.py) | Telemetry display and rendering; IMU ROS service requests. | Backend owns sensor state. #22 extracts rendering/history/adapters. |
| [robot_view.py](../src/saprog/scripts/robot_view.py) | Embedded renderer launch/readiness/process cleanup. | #23 extracts session ownership; do not share this view's private process methods directly. |

## C++ units, hardware and sensor boundaries

| Source / API | Responsibility and usage | Constraints |
| --- | --- | --- |
| [servo_units.hpp](../src/saprog/include/saprog/servo_units.hpp) | `saprog::ax::to_radians(raw, zero, direction)`, `to_raw(radians, zero, direction)` and position/tick/speed constants; controller/readback/bus use this contract. | AX 300 degrees, 0..1023 ticks. to_raw returns unrounded value so caller checks limits before rounding. |
| [config.hpp](../src/saprog/include/saprog/config.hpp) | `SaprogConfig`, `read_config(node)`: read ROS startup settings with validation; controller/readback use it. | Cached settings; runtime consistency needs #18. Not yet a ROS-independent validation API. |
| [dynamixel_bus.hpp](../src/saprog/include/saprog/dynamixel_bus.hpp), [implementation](../src/saprog/src/dynamixel_bus.cpp) | `DynamixelBus`: shared serial connection, read/limits/torque, single/batch positions, before-deadline variants, verified settings cache. | Readback creates connection and controller shares it. All transaction/cache access stays under bus ownership. Verification of goal registers does not prove physical arrival. #19 owns split. |
| [imu_filter.hpp](../src/saprog/include/saprog/imu_filter.hpp) | `saprog::imu::Sample`, `parse_sample(line, sample)`, vector/quaternion helpers and `Filter` update/calibration/zero/heading operations. | Strict calculated ten-column CSV, firmware g/degrees/s/microtesla. Quat is wxyz sensor-to-world; ROS uses xyzw. #21 splits protocol/math/filter without replacing numeric behavior. |
| [imu_node.cpp](../src/saprog/src/imu_node.cpp) | Serial/helper lifetime, sample scaling and ROS SI publication/services. | Sensor state remains backend-owned. #20 extracts exclusive transport and helper owners; #21 owns message/unit boundary. |
| [robot_view_bridge.py](../src/saprog/scripts/robot_view_bridge.py), [URDF](../urdf/urdf/URDF.urdf) | Current servo-ID to description-joint map and base orientation publication. | Export includes misspelled joint name; do not silently rename. Mounting/reference/staleness policy belongs to #23 under #4. |

C++ and Python necessarily implement equivalent unit contracts in separate
languages; keep regression fixtures aligned rather than crossing runtimes just
to eliminate a few arithmetic expressions.

## Shell and installed resources

[_launch_window.bash](../tabs/_launch_window.bash) currently owns shared launcher
setup/readiness/build locking and owned-backend cleanup. Existing small launchers
call it; add no competing bootstrap. [ensure_usb.bash](../scripts/ensure_usb.bash)
and [ensure_imu_usb.bash](../scripts/ensure_imu_usb.bash) share attach mechanics
but have distinct selection and output contracts. #24 extracts matching pieces.
IMU helper stdout must remain its selected device path, with diagnostics on stderr.

Keep installed resource registration in [CMakeLists.txt](../src/saprog/CMakeLists.txt)
and [description installation](../urdf/CMakeLists.txt) aligned with moves.
#8/#25 cover packaging and clean-install validation; the source checkout alone
does not establish that an entry point will work after installation.
