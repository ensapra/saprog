# Saprog Desktop

The PySide6 application starts from the repository root:

```sh
python -m pip install -r desktop/requirements.txt
python desktop
```

The shell and document editing run without ROS. Hardware access through the
shared manager currently requires Windows with WSL, defaulting to Ubuntu-24.04
and ROS Jazzy. Set SAPROG_WSL_DISTRO in the desktop environment to select another
WSL distribution. Existing ROS publishers are reused; missing providers are
built and launched from [ros2](../ros2/).

## Tabs

| Tab | Contents |
| --- | --- |
| Statistics | Temperature, voltage and raw AX speed graph; IMU readings and orientation box |
| Static Animator | Snapshot Creator, saved Keyframes, per-servo movement and Puppet Mode |
| TimeLine | Local snapshot-reference sequence editing, with waits and transition settings |
| Motion Recorder | Record a fixed-length loop, then layer other-leg recordings over it |
| Controller | Native pygame gamepad detection, per-GUID mappings and snapshot/sequence execution |
| Simulation | On-demand Gazebo URDF import, gravity, pause/reset and optional viewer |
| Single Leg Test | Physical crank/rod/knee loop, editable geometry, loaded servo sweeps and live plots |

Drag tabs to reorder them. Double-click a tab to detach it; closing the detached
window returns it to the main window. See [operation](../docs/operation.md) for
control behavior, servo-limit refresh, sequence cancellation and file formats.
See [IMU](../docs/imu.md) and the [servo package](../ros2/saprog_dynamixel/README.md)
for ROS topics and hardware configuration.

See the [simulation guide](../ros2/saprog_simulation/README.md) for the lightweight
defaults, Gazebo dependencies and isolated simulation controls.
See the [single-leg guide](../ros2/saprog_leg_test/README.md) for the MuJoCo
runtime setup, geometry calibration and dedicated test node.

## Adding a tab

Create a package under [tabs](tabs/) with an __init__.py defining a nonempty
TAB_TITLE and a create_tab() factory returning a QWidget. Optionally define
TAB_ORDER; the default is 100, with equal-order tabs sorted by title. A factory
accepting ros_manager receives the shared manager. Ordinary modules and
underscore-prefixed packages are skipped by [discovery](tabs/__init__.py).

Keep page composition inside its tab package, reusable controls under
[widgets](widgets/), and toolkit-independent snapshot/timeline rules under
[documents](documents/). Reuse the [shared style](../docs/desktop-style.md) and
[ROS manager](ros_manager/manager.py). The [code guide](../docs/code-guide.md)
explains ownership, and [testing](../docs/testing.md) lists the current suites.
