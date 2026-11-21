# Code guide

## Desktop ownership

[app.py](../desktop/app.py) creates one QApplication, applies the shared style,
constructs one RosDataManager and adds the discovered tabs to
[WorkspaceMainWindow](../desktop/tab_workspace.py). Quitting closes the manager.
Tab dragging reorders pages; double-clicking detaches a page, and closing its
window restores it. The timeline page guards unsaved edits when its host closes.

[Tab discovery](../desktop/tabs/__init__.py) imports non-private tab packages.
Each defines TAB_TITLE and create_tab(), optionally TAB_ORDER. The factory gets
ros_manager only if its signature accepts that argument. Shared controls and
visualizations belong in [widgets](../desktop/widgets/).

| Responsibility | Source |
| --- | --- |
| Statistics composition | [home](../desktop/tabs/home/__init__.py) |
| Telemetry history and plots | [dynamixel_graph](../desktop/widgets/dynamixel_graph/) |
| IMU labels and orientation drawing | [imu_visualizer](../desktop/widgets/imu_visualizer/) |
| Snapshot composition and ROS commands | [player](../desktop/widgets/player/__init__.py) |
| Target fields, Puppet Mode and snapshot saving | [creator_view](../desktop/widgets/player/creator_view.py) |
| Shared live hardware limits and EEPROM report validation | [servo_limits](../desktop/servo_limits/) |
| Offline timeline document and editor | [timeline](../desktop/tabs/timeline/) |
| Controller discovery and input polling | [devices](../desktop/tabs/controller/devices.py) |
| Mappings and press/release routing | [Controller tab](../desktop/tabs/controller/__init__.py) |
| Sequence execution and arrival monitoring | [playback](../desktop/tabs/controller/playback.py) |
| Gazebo startup and physics controls | [Simulation tab](../desktop/tabs/simulation/) |

## Shared documents

Each ROS manager owns one [ServoLimits](../desktop/servo_limits/service.py) object.
All motion consumers receive its read-only, live mapping. The service deduplicates
EEPROM reads, validates all configured servos, converts directions/encoder zeros
through the document unit helpers, handles timeouts and invalidates failed reads.
It performs no persistence. The manager enforces these same bounds at the position
publication boundary, and the simulation supervisor receives them in its start
payload to build temporary joint stops. URDFs and tuning files are not limit sources.

[documents](../desktop/documents/) imports neither ROS nor Qt.
[snapshot_units.py](../desktop/documents/snapshot_units.py) validates radian poses
and converts calibrated encoder limits. [timeline_domain.py](../desktop/documents/timeline_domain.py)
owns transition settings, grouping and schema validation.
[timeline_presenter.py](../desktop/documents/timeline_presenter.py) owns editable
steps and dirty state. [timeline_storage.py](../desktop/documents/timeline_storage.py)
resolves snapshot references and migrates embedded poses.
[snapshot_files.py](../desktop/documents/snapshot_files.py) performs atomic JSON
replacement. Reuse these rules across editing and controller playback.

## ROS boundary

[RosDataManager](../desktop/ros_manager/manager.py) exposes topic request tokens,
release, position/torque commands and Qt result signals without importing ROS.
[config.py](../desktop/ros_manager/config.py) describes built-in topic providers.
[RosWorker](../desktop/ros_manager/ros_worker.py) runs a WSL Python process and
exchanges newline-delimited JSON. Its current run path rejects non-Windows hosts.
[node_launcher.py](../desktop/ros_manager/node_launcher.py) contains a local
launcher helper; it does not make the desktop worker support native Linux.

The WSL bridge is split into [runtime](../desktop/ros_manager/wsl_bridge/runtime.py),
[protocol](../desktop/ros_manager/wsl_bridge/protocol.py),
[subscriptions](../desktop/ros_manager/wsl_bridge/subscriptions.py),
[processes](../desktop/ros_manager/wsl_bridge/processes.py) and
[commands](../desktop/ros_manager/wsl_bridge/commands.py). Topic requests share
subscriptions. Existing publishers are reused; missing providers trigger a colcon
build check and launch. Startup runs the USB helpers where appropriate and loads
an installed saprog.yaml when present. Shutdown cancels builds and stops processes
owned by the bridge; it does not stop externally started publishers.

Position-command success means the bridge published the message after matching
a subscriber. It does not confirm backend acceptance or physical arrival.
Controller playback separately checks telemetry for until-reached transitions.
Cancellation can remove unpublished commands, but cannot recall transmitted ones.

## ROS packages

[saprog_dynamixel](../ros2/saprog_dynamixel/) owns the servo serial port, polls
telemetry and handles positions, torque and limit reads in one node. SDK register
access belongs to [ServoBus](../ros2/saprog_dynamixel/src/servo_bus.cpp); topic
encoding belongs to [ServoReadingsPublisher](../ros2/saprog_dynamixel/src/servo_readings_publisher.cpp).

[saprog_imu](../ros2/saprog_imu/) separates serial framing, CSV parsing,
orientation estimation and ROS publication. See the [IMU contract](imu.md).

The [description package](../urdf/) contains geometry and calibrated model data.
It lives outside the desktop's ros2 workspace. The Simulation tab imports it
directly through [saprog_simulation](../ros2/saprog_simulation/), which owns an
isolated Gazebo server using native Transport bindings. Its command/result path
is separate from hardware commands in the shared WSL bridge. See
[simulation](../ros2/saprog_simulation/README.md) and [testing](testing.md).
