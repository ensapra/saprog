# Project context

ROS 2 Jazzy control and desktop tools for a 12-servo quadruped. Use this page for the code map and invariants; consult the linked guide only when relevant.

## Guides

- [Operation](operation.md): launch, USB, controls, ROS interfaces and file formats.
- [IMU](imu.md): sensor contract, calibration and limitations.
- [Robot view](robot-view.md): RViz process, display support and limitations.
- [Desktop style](desktop-style.md): required for desktop UI changes.
- [Testing](testing.md): test suites and environments; [validation history](validation.md).
- [Reusable API map](reusability.md): existing shared owners. Update it when shared APIs change.
- [Refactor review](refactor-review.md): boundaries, backlog and validation limits.

## Code map

| Area | Owner |
| --- | --- |
| Backend and serial bus | [main.cpp](../src/saprog/src/main.cpp), [dynamixel_bus.cpp](../src/saprog/src/dynamixel_bus.cpp) |
| Commands and limits | [position_controller_node.cpp](../src/saprog/src/position_controller_node.cpp), [servo_units.hpp](../src/saprog/include/saprog/servo_units.hpp), [config](../src/saprog/config/saprog.yaml) |
| Desktop | [saprog_desktop](../src/saprog/scripts/saprog_desktop/) |
| Documents | [snapshot and timeline modules](../src/saprog/scripts/saprog_desktop/) |
| IMU | [imu_node.cpp](../src/saprog/src/imu_node.cpp), [imu_filter.hpp](../src/saprog/include/saprog/imu_filter.hpp) |
| Robot view | [robot view modules](../src/saprog/scripts/saprog_desktop/), [URDF](../urdf/) |

## Invariants

- One serial bus owner; ROS callbacks serialize, while timeline work uses locks. Tk callbacks use `spin_once`.
- Startup does not change torque. Position commands enable it. Stop cannot undo transmitted motion.
- ROS positions are radians; AX conversion uses 300 degrees / 1023 ticks plus configured zero, direction and measured limits.
- A sent command or accepted goal register does not prove physical arrival. Snapshot playback and timeline timing have distinct policies.
- IMU calibration/zero belong to its backend. Closing a display does not reset them; sensor-relative attitude is not a calibrated robot mounting transform.

## Open work

| Issue | Focus |
| --- | --- |
| [#1](https://github.com/ensapra/saprog/issues/1) | Timeline execution bounds, final torque action and runtime parameters |
| [#2](https://github.com/ensapra/saprog/issues/2) | Compose views/clients/playback and bound requests |
| [#3](https://github.com/ensapra/saprog/issues/3) | Legacy migration ambiguity (closed; preserve pose matching) |
| [#4](https://github.com/ensapra/saprog/issues/4) | Sensor mounting/reference and robot view |
| [#5](https://github.com/ensapra/saprog/issues/5) | IMU extraction and library evaluation |

## Validation status

Historical: Python tests, Ruff, snapshot validation and URDF mapping checks passed. A fresh ROS Jazzy build and all eight CTest suites passed on 2026-11-18. These results do not cover live motion, reconnects, mounting accuracy or hardware timing. Use WSL for ROS builds/tests; see [testing](testing.md).
