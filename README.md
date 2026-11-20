# Saprog

Qt for Python desktop tools and ROS 2 Jazzy backends for a 12-servo quadruped.

## Launch the desktop

Install the desktop dependencies and run from the repository root:

```sh
python -m pip install -r desktop/requirements.txt
python desktop
```

The desktop starts without ROS. Hardware widgets use the shared ROS manager to
build and start the required packages from `ros2/`. On Windows this runs through
WSL, using `Ubuntu-24.04` by default; override it with `SAPROG_WSL_DISTRO`.
ROS 2 Jazzy and the packages' build dependencies must be installed there.

See the [desktop guide](desktop/README.md) for tabs, controller mappings and IMU
setup, and the [DYNAMIXEL guide](ros2/saprog_dynamixel/README.md) for servo topics,
commands and configuration.

## Repository layout

| Folder | Contents |
| --- | --- |
| [desktop/](desktop/) | Current Qt application, shared document models and tests |
| [ros2/](ros2/) | Current DYNAMIXEL and IMU ROS packages |
| [scripts/](scripts/) | USB attachment helpers used by the ROS manager |
| [snapshots/](snapshots/) | Saved poses |
| [sequences/](sequences/) | Saved timelines |
| [urdf/](urdf/) | Robot geometry and measured servo limits |
| [docs/](docs/README.md) | Project documentation and historical design notes |

The legacy ROS/Tk source tree and shell launchers have been retired. Build,
install and log folders are generated when the current ROS workspace is built.

## USB setup

On native Linux, connect the serial adapters and grant the user access to their
device paths. Servo configuration is in
[saprog.yaml](ros2/saprog_dynamixel/config/saprog.yaml).
Set `SAPROG_IMU_DEVICE` to select an explicit IMU serial port.

On Windows, install `usbipd-win` to expose serial devices to WSL. Run the initial
bind from Administrator PowerShell, using the bus ID shown by `usbipd list`:

```powershell
usbipd list
usbipd bind --busid <BUSID>
usbipd attach --wsl Ubuntu-24.04 --busid <BUSID>
```

The ROS manager uses the USB helpers for the servo adapter (`0403:6014`) and IMU
(`1B4F:9D0F`). Override selection with `SAPROG_USB_BUSID` or
`SAPROG_USB_HARDWARE_ID` for servos, and `SAPROG_IMU_USB_BUSID` or
`SAPROG_IMU_USB_HARDWARE_ID` for the IMU. Gamepads are detected by the desktop
through native pygame.

## Desktop tests

With the desktop dependencies installed, run from the repository root:

```sh
python -m unittest discover -s desktop/tests
```
