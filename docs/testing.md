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
| [test_servo_limits.py](../desktop/tests/test_servo_limits.py) | Complete hardware reports, metadata preservation and bridge service results |
| [test_timeline_editor.py](../desktop/tests/test_timeline_editor.py) | Document migration, references, edits, atomic saves and window close handling |
| [test_ros_launcher.py](../desktop/tests/test_ros_launcher.py) | Launch shell, USB preflight and parameter-file behavior with stub commands |
| [test_wsl_bridge.py](../desktop/tests/test_wsl_bridge.py) | Protocol, subscriptions, command dispatch and owned-process cleanup |
| [test_simulation.py](../desktop/tests/test_simulation.py) | Lazy Gazebo startup, UI controls, isolated command/results, path conversion and world generation |
| [test_leg_test.py](../desktop/tests/test_leg_test.py) | Four-bar closure, loaded MuJoCo sweeps, lifecycle, Qt controls and isolated node routing |

The single-leg physics tests additionally require `mujoco>=3.2,<4`; without it
those tests skip. See the [single-leg guide](../ros2/saprog_leg_test/README.md)
for the separate ROS runtime environment and calibration limitations.

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

The simulation package has an opt-in headless Gazebo test for import, gravity,
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
