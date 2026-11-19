# Saprog

ROS 2 Jazzy desktop tools for a 12-servo quadruped: pose editing/playback, timelines, temperature monitoring, IMU and RViz2. Runs on Linux/WSL with ROS and configured USB devices.

## Launching the system

Open Ubuntu/WSL and start the full desktop from the repository root:

```bash
cd /mnt/c/Users/pauso/Documents/GitHub/saprog
bash ./launch_home.sh
```

`launch_home.sh` builds the workspace, starts the shared robot controller,
and opens Home. It can take a USB device into exclusive WSL use. Close Home
to stop its owned backend processes.

Run individual tools from the repository root:

```bash
bash ./tabs/launch_snapshot_creator.sh
bash ./tabs/launch_snapshot_player.sh
bash ./tabs/launch_timeline.sh
bash ./tabs/launch_static_animator.sh
bash ./tabs/launch_imu_monitor.sh
```

The standalone tools start or reuse the ROS controllers required by their
views. Without a hardware device, use them only for editing or inspection
where noted in the operation guide.

## Native Linux

Native Linux requires the configured serial devices to already exist and be
accessible:

```bash
ls -l /dev/ttyUSB* /dev/ttyACM*
```

The servo/U2D2 device is configured by `device_name` in
`src/saprog/config/saprog.yaml`, currently `/dev/ttyUSB0`. The IMU can be
selected with `SAPROG_IMU_DEVICE=/dev/ttyACM0` or a launch argument.

## Windows/WSL USB setup

Saprog uses `usbipd-win` to expose Windows USB devices inside WSL. Install
the required Linux package if needed:

```bash
sudo apt update
sudo apt install -y python3-pygame usbutils
```

Run `usbipd` commands in **Administrator PowerShell**, not inside WSL.
USB bus IDs can change after unplugging or rebooting:

```powershell
usbipd list
```

The normal setup for each component is:

```powershell
usbipd bind --busid <BUSID>
usbipd attach --wsl Ubuntu-24.04 --busid <BUSID>
```

Return a device to Windows with:

```powershell
usbipd detach --busid <BUSID>
```

### U2D2 / servo bus

The default hardware ID is `0403:6014`. It normally appears as
`/dev/ttyUSB0` in WSL. Launching Saprog can bind and attach it automatically
when `usbipd.exe` is available.

```powershell
usbipd list
usbipd bind --busid <U2D2-BUSID>
usbipd attach --wsl Ubuntu-24.04 --busid <U2D2-BUSID>
```

Override automatic selection in WSL with:

```bash
export SAPROG_USB_BUSID="<BUSID>"
# or
export SAPROG_USB_HARDWARE_ID="0403:6014"
```

Keep `device_name` in `src/saprog/config/saprog.yaml` pointed at the character
device created for the adapter.

### IMU

The default IMU hardware ID is `1B4F:9D0F`. It normally appears as
`/dev/ttyACM*` or `/dev/ttyUSB*`.

```powershell
usbipd list
usbipd bind --busid <IMU-BUSID>
usbipd attach --wsl Ubuntu-24.04 --busid <IMU-BUSID>
```

Override selection in WSL with:

```bash
export SAPROG_IMU_USB_BUSID="<BUSID>"
# or
export SAPROG_IMU_USB_HARDWARE_ID="1B4F:9D0F"
export SAPROG_IMU_DEVICE="/dev/ttyACM0"
```

### Game controller

The default controller hardware ID is `2E24:1125`. Close Steam, Xbox
Accessories, controller remapping software, games, and emulator frontends
before attaching it to WSL:

```powershell
usbipd list
usbipd bind --busid <CONTROLLER-BUSID> --force
usbipd attach --wsl Ubuntu-24.04 --busid <CONTROLLER-BUSID>
```

Verify native Windows Pygame from Windows PowerShell:

```powershell
& ".\.venv\Scripts\python.exe" -c "import pygame; pygame.joystick.init(); print(pygame.joystick.get_count())"
```

Pygame must report at least `1`. The Saprog Controller tab uses this native
Windows Pygame process when it runs under WSL, so no WSL `xpad` or
`/dev/input/js*` device is required.

The old WSL-only verification was:

```bash
ls -l /dev/input/js*
python3 -c "import pygame; pygame.joystick.init(); print(pygame.joystick.get_count())"
```

Override automatic controller selection in WSL with:

```bash
export SAPROG_CONTROLLER_USB_BUSID="<BUSID>"
# or
export SAPROG_CONTROLLER_USB_HARDWARE_ID="2E24:1125"
```

See [docs/](docs/README.md) for project context, [operation](docs/operation.md) for setup and controls, and [IMU](docs/imu.md) for sensor behavior and limits.
