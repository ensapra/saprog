# Saprog robot description

[package.xml](package.xml) names this ament_cmake package saprog_description.
[CMakeLists.txt](CMakeLists.txt) installs [meshes](meshes/) and [urdf](urdf/).
This folder is outside the desktop manager's ros2 workspace; the current desktop
does not build or display this package automatically.

The [Simulation tab](../ros2/saprog_simulation/README.md) can import this URDF
directly into Gazebo without building the description package.

## Model and meshes

[URDF.urdf](urdf/URDF.urdf) references eight shared meshes through
package://saprog_description/meshes/: chassis, hip mount, and left/right linkage,
crank and leg parts. All four hip mounts use hip_mount.stl.

The model has twelve servo joints and four leg joints driven by mimic tags.
Desktop and hardware messages use servo_N names; the model uses these names:

| Leg | Servo IDs | Model joint names |
| --- | --- | --- |
| Front right | 11, 12, 13 | front_right_11, front_right_12, front_right_joint_13 |
| Back right | 21, 22, 23 | back_right_21, back_right_22, back_right_joint_23 |
| Front left | 31, 32, 33 | front_left_31, front_left_32, front_left_joint_33 |
| Back left | 41, 42, 43 | back_left_41, back_left_42, back_left_joint_43 |

There is no current desktop joint-state-to-model bridge. The four crank joints
use logical servo radians, -X axes and limits of approximately
0.2661499153 to 0.8803420274 radians. Their origin rotations include the calibrated
reference offset. Each corresponding leg joint mimics its crank with multiplier
2.4 and offset approximately -2.1128208658 radians, with limits about
-1.4740610691 to 0. This is a kinematic approximation; mimic tags do not enforce a
closed mechanical linkage or establish valid contact dynamics.

## Measured servo data

[measured_servo_limits.json](config/measured_servo_limits.json) stores raw encoder
limits, zero positions, directions, converted radians and model metadata. The
Static Animator reads fresh EEPROM limits through the ROS bus owner and updates
this file while preserving the model fields. It does not regenerate the URDF.
The JSON and URDF therefore need separate review after calibration changes.
The source and timestamp in the JSON describe the last stored measurement.

The [launch directory](launch/) contains ROS 1 exporter XML using old package
names and ROS 1 nodes. Those files are not installed by the current CMake package
and are not ROS 2 launch instructions. There is no current RViz or Gazebo launcher
in this package. Old experimental-variant and rollback notes were removed because
their described files are not present in this checkout.
