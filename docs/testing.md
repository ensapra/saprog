# Testing

## Desktop

Install [desktop dependencies](../desktop/requirements.txt) into your chosen
Python environment. From the repository root:

```sh
python -m pip install -r desktop/requirements.txt
python -m unittest discover -s desktop/tests
```

The Qt tests select the offscreen platform themselves. They use fake managers,
controllers and ROS endpoints; ROS and physical devices are not required.

| Suite | Coverage |
| --- | --- |
| [test_controller.py](../desktop/tests/test_controller.py) | Input discovery, mappings, sequence ownership, timing, torque release and publication readiness |
| [test_player.py](../desktop/tests/test_player.py) | Snapshot load/save, movement commands, Puppet Mode and limit refresh |
| [test_servo_limits.py](../desktop/tests/test_servo_limits.py) | Shared read-only live reports, refresh deduplication, failure invalidation, consumer updates and bridge routing |
| [test_timeline_editor.py](../desktop/tests/test_timeline_editor.py) | Document migration, references, edits, atomic saves and window close handling |
| [test_ros_launcher.py](../desktop/tests/test_ros_launcher.py) | Launch shell, USB preflight and parameter-file behavior with stub commands |
| [test_ros_worker.py](../desktop/tests/test_ros_worker.py) | Slow WSL startup, bounded timeout, cancellation, process cleanup and path diagnostics |
| [test_wsl_bridge.py](../desktop/tests/test_wsl_bridge.py) | Protocol, subscriptions, command dispatch and owned-process cleanup |
| [test_simulation.py](../desktop/tests/test_simulation.py) | Lazy Gazebo startup, UI controls, isolated command/results, path conversion and world generation |
| [test_leg_urdf.py](../desktop/tests/test_leg_urdf.py) | Single-leg mesh extraction, servo-name mapping, stale feedback, Cartesian reachability, local preview, calibration-only tuning and URDF export |
| [test_leg_joystick.py](../desktop/tests/test_leg_joystick.py) | Analog Cartesian foot control, axis mappings, neutral/enable gating, feedback, command cancellation, timeouts and preview lifecycle |
| [test_controller_selector.py](../desktop/tests/test_controller_selector.py) | Shared controller detection, GUID restoration after reconnect and stable selection on refresh |

See the [single-leg guide](single-leg-test.md) for URDF preview and calibration
limitations.

Launcher tests require Bash and skip when it is unavailable. On Windows they
look for Git Bash at C:/Program Files/Git/bin/bash.exe.

For a focused run, use the matching filename, for example:

```sh
python -m unittest discover -s desktop/tests -p test_controller.py
```

Optional static checks use the pinned tools in
[requirements-dev.txt](../requirements-dev.txt) and the
[Ruff configuration](../pyproject.toml):

```sh
python -m pip install -r requirements-dev.txt
ruff check desktop
```

## ROS packages

The simulation package also has pure offline bounds-profile tests:

```sh
python -m unittest discover -s ros2/saprog_simulation/test -p test_profile.py
```

These check schema/provenance, exact servo coverage, finite ordered bounds,
zero-pose compatibility, input isolation and reset input preservation. Desktop
simulation tests additionally check startup with unavailable hardware limits and
that opening the tab never requests a hardware provider.

The simulation package has an opt-in headless Gazebo test for profile import, gravity,
pause, reset and cleanup. See its [guide](../ros2/saprog_simulation/README.md#validation).

The current C++ tests are registered under BUILD_TESTING in each package's CMake
file. Use a ROS Jazzy environment with the declared package dependencies and
ament_cmake_gtest installed, build with testing enabled, and run the discovered
suites through the CMake Tools test integration.

- [dynamixel_command_test](../ros2/saprog_dynamixel/test/dynamixel_command_test.cpp)
  uses SDK stubs to test position/torque ordering, command rejection, release
  behavior and EEPROM limit reads. Registration is in its
  [CMakeLists.txt](../ros2/saprog_dynamixel/CMakeLists.txt).
- [imu_orientation_estimator_test](../ros2/saprog_imu/test/imu_orientation_estimator_test.cpp)
  checks rotations, hold, gravity correction, reset and magnetic-heading mode.
  Registration is in its [CMakeLists.txt](../ros2/saprog_imu/CMakeLists.txt).

These suites do not establish real USB behavior, physical arrival, EEPROM
configuration, magnetic calibration or robot mounting accuracy. Record actual
hardware validation separately from automated test results. Test counts and old
build outcomes are not guarantees for a later checkout.
