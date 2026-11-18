# Maintainer context

Read [operation.md](operation.md) for setup, controls and file formats, and
[imu.md](imu.md) for the sensor contract. Keep durable project knowledge here;
implementation tasks and acceptance criteria belong in the linked GitHub issues.

Read the [reusable API and usage map](reusability.md) before adding shared code.
The [project refactoring review](refactor-review.md) records responsibility
boundaries, source findings, issue dependencies and validation limits. Its
[tracking issue #7](https://github.com/ensapra/saprog/issues/7) links the focused
implementation backlog; existing broad issues below remain open.

## Code map

For any desktop tab, window, or Tk UI change, follow the current
[desktop style guide](desktop-style.md).

| Area | Start here | Responsibility |
| --- | --- | --- |
| Backend | [main.cpp](../src/saprog/src/main.cpp) | Creates readback and controller nodes sharing one serial connection |
| Hardware | [dynamixel_bus.cpp](../src/saprog/src/dynamixel_bus.cpp) | Serial locking, synchronized writes, verified register cache |
| Commands | [position_controller_node.cpp](../src/saprog/src/position_controller_node.cpp) | Limits, torque services, position commands and timeline worker |
| Units/config | [servo_units.hpp](../src/saprog/include/saprog/servo_units.hpp), [saprog.yaml](../src/saprog/config/saprog.yaml) | AX conversions, IDs, zero positions, directions and defaults |
| Desktop | [home.py](../src/saprog/scripts/home.py), [scripts/](../src/saprog/scripts/) | Tk windows and ROS clients |
| Documents | [snapshot_units.py](../src/saprog/scripts/saprog_desktop/snapshot_units.py), [timeline_model.py](../src/saprog/scripts/timeline_model.py), [snapshot_files.py](../src/saprog/scripts/saprog_desktop/snapshot_files.py) | Validation, reference resolution and atomic JSON storage |
| IMU | [imu_node.cpp](../src/saprog/src/imu_node.cpp), [imu_filter.hpp](../src/saprog/include/saprog/imu_filter.hpp) | Serial lifecycle, estimation, calibration and zero |
| Visualization | [robot_view.py](../src/saprog/scripts/robot_view.py), [robot_view_session.py](../src/saprog/scripts/saprog_desktop/robot_view_session.py), [robot_view_bridge.py](../src/saprog/scripts/robot_view_bridge.py), [urdf/](../urdf/) | Tk viewport, owned ROS 2 renderer process, and servo/IMU state to robot transforms |

See [robot-view.md](robot-view.md) for installed launch behavior, display frame
limitations and the X11 smoke-check procedure.

## Behavior to preserve

- ROS node callbacks are serialized. The timeline worker runs separately, with
  bus, goal and telemetry locks protecting shared state. Tk callbacks use `spin_once`.
- Startup leaves torque unchanged; position commands enable it. Stop prevents
  later timeline steps but does not undo motion already transmitted.
- ROS positions are radians. AX encoders cover 300 degrees in 0..1023 ticks;
  commands apply configured zero/direction and measured hardware limits.
- ROS owns timeline timing; Creator/Player still own snapshot arrival/release.
  Never equate publishing a command or reading its goal register with arrival.
- IMU state belongs to its backend. A display closing must not reset calibration
  or zero. Sensor-relative orientation is not a calibrated robot mounting transform.

## Open work

| Priority | Next task | Issue |
| --- | --- | --- |
| High | Extract/test timeline execution; bound durations, final torque actions and runtime parameter changes | [#1](https://github.com/ensapra/saprog/issues/1) |
| High | Compose views/ROS clients/shared playback; bound pending requests | [#2](https://github.com/ensapra/saprog/issues/2) |
| High | Reject ambiguous legacy migration: matching a snapshot name alone can select a different pose | [#3](https://github.com/ensapra/saprog/issues/3) |
| Medium | Validate a measured sensor-to-base mounting transform and display reference | [#4](https://github.com/ensapra/saprog/issues/4) |
| Medium | Separate IMU transport/estimation; evaluate maintained fusion and trajectory libraries with regression evidence | [#5](https://github.com/ensapra/saprog/issues/5) |

Domain/storage helpers have clear responsibilities. The controller and window
classes still need separation; avoid expanding their inheritance or responsibilities.

## Validation and handoff

The last code review passed 34 Python tests under WSL/ROS Jazzy, Ruff checks,
offline validation of 17 snapshots/four sequences, and 12 URDF joint mappings.
It fixed UI lockups after lost torque responses/failed timeline submission,
centralized conversions/status displays, and rejected invalid startup configuration.

C++ build/CTest could not run through the Windows VS Code integration; new C++
configuration/conversion tests remain unverified. Live motion, reconnects, mounting,
sensor accuracy and deadline behavior also need hardware checks.

[Tests](../src/saprog/test/) use Python `unittest` and CTest. Source ROS and the
built workspace before Python discovery; CTest also registers that suite.
Formatting tools are pinned in [requirements-dev.txt](../requirements-dev.txt),
with [Ruff](../pyproject.toml) and [clang-format](../.clang-format) settings.
