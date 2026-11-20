# Saprog Desktop

This is the ROS-independent Qt for Python desktop application. Install its
dependency with:

```sh
python -m pip install -r desktop/requirements.txt
```

Then launch it from the repository root with:

```sh
python desktop
```

The Statistics tab contains an IMU widget and a servo temperature widget. The
IMU widget requests its raw readings and orientation through the shared ROS data
manager. On Windows, the manager connects through WSL (default distro
`Ubuntu-24.04`; override with `SAPROG_WSL_DISTRO`), and builds/starts a requested
ROS package when its topic is unavailable. The desktop can still start without
ROS; the IMU widget reports the connection status.

The desktop uses the `saprog_imu/imu_publisher_node` package in the `ros2/`
workspace. Before starting it in WSL, the bridge runs the project's IMU USB
helper to attach the SparkFun device (`1B4F:9D0F`) and select its serial port.
Set `SAPROG_IMU_DEVICE` in the WSL environment to use an explicit port and skip
automatic attachment. The backend publishes `/saprog/imu/data_raw`,
`/saprog/imu/mag`, and `/saprog/imu/orientation` with sensor-data QoS.
To check it directly from a WSL shell at the repository root:

```sh
source /opt/ros/jazzy/setup.bash
cd ros2
colcon build --packages-up-to saprog_imu
source install/setup.bash
export SAPROG_IMU_DEVICE="$(bash ../scripts/ensure_imu_usb.bash)"
ros2 run saprog_imu imu_publisher_node
```

In another WSL shell with ROS sourced, read a sample with
`ros2 topic echo /saprog/imu/data_raw sensor_msgs/msg/Imu --qos-reliability best_effort --once`.

The tabs use a standard Qt tab bar. Drag a tab within the bar to reorder it;
double-click a tab to open it in a separate window. Closing that window returns
the tab to the main window.

## Controller

The Controller tab detects native Windows USB/Bluetooth gamepads through pygame;
select a device to see its buttons, D-pad directions and axis directions (60%
threshold). Inputs highlight while pressed. Choose an action and, for a snapshot
or sequence, a JSON file. Mappings are saved per controller GUID. Source folders
can be changed in the tab; refresh files after saving new poses or timelines.

Torque on/off and snapshots run once per press through the shared ROS manager.
Snapshot speed and torque use the tab's controls. Choose **Sequence (one shot)**
to run the full sequence once per press, continuing after the input is released.
Choose **Sequence (hold to repeat)** to repeat while held; releasing stops playback.
Both modes use the saved motion settings, waits and transition durations,
regardless of the file's loop/end-torque settings. Until-reached transitions need
fresh joint-state feedback. The latest mapped press takes control; disconnect,
changing devices/mappings/folders or Stop playback cancels later steps and
unpublished controller commands. Torque off also stops playback.
A movement already sent may finish.
Inputs already held when selecting a device must be released before activation.

## Static Animator limits

Entering the Static Animator tab reads the servos' current EEPROM angle limits
through the shared ROS port owner. A complete valid read updates the displayed
radian bounds and target validation, and saves the measured values in
`urdf/config/measured_servo_limits.json`, preserving the URDF metadata. This read
does not move the servos or change torque or EEPROM settings. If the hardware is
unavailable or a limit is invalid, the previous stored values remain in use and
the tab displays the failure; re-enter the tab to retry.

## Adding a tab

Add a folder under `tabs/` with an `__init__.py` that defines `TAB_TITLE` and a
`create_tab()` function returning a `QWidget`. Keep the tab's code and assets
inside its folder. Reusable widgets live under `widgets/`, each in its own
folder with an `__init__.py`; keep supporting code and assets in that widget
folder. Tabs import widgets from this shared package so they can be reused by
other tabs. The app discovers and adds tab packages at startup.
Optionally define `TAB_ORDER` to choose a tab's position; unnumbered tabs follow
the built-in ones alphabetically. Ordinary modules and underscore-prefixed
folders are ignored by tab discovery and can hold shared helpers.

Shared snapshot/timeline document rules and atomic storage live in
[`documents/`](documents/), independently of ROS and the UI toolkit.
