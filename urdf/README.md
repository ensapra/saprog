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

The desktop **Single Leg Test** maps received `servo_N` radians to one selected
leg, renders its meshes and exposes mapping/foot-path calibration. Its default
[front_right.urdf](urdf/front_right.urdf) contains just the front-right branch
and an empty fixed base. Other legs are extracted from the full description.
See the [single-leg guide](../docs/single-leg-test.md).
The four crank joints use logical servo radians. Right-side cranks (13 and 23)
use +X axes and knee mimic multiplier -2.4 with offset 0.527179134242642 radians:
increasing servo radians folds the leg. Their origin rotations preserve the
existing 0.55 rad preview pose while reversing motion around it. This reference
is a pose calibration, not a travel limit. Left-side cranks (33 and 43) retain
their -X axes and knee mimic multiplier 2.4 with offset -2.112820865757358 radians.
Servo and mimic joints are continuous in the saved geometry: old angle stops
were removed. Gazebo receives current EEPROM bounds in its start payload and
adds motor stops and derived mimic ranges only to its temporary import copy.
This is a kinematic approximation; mimic tags do not enforce a
closed mechanical linkage or establish valid contact dynamics.

## Live servo limits

The old `config/measured_servo_limits.json` cache was removed. Limits are read
from servo EEPROM through the existing serial-port owner and shared in memory by
the desktop [limits service](../desktop/servo_limits/). Static Animator, controller
and timeline playback, Motion Recorder, Single Leg Test and Gazebo all consume
that same report. Failed reads invalidate it. Limits are not written to geometry,
tuning files or another cache. Encoder zeros and directions remain in the hardware
[configuration](../ros2/saprog_dynamixel/config/saprog.yaml).

The [launch directory](launch/) contains ROS 1 exporter XML using old package
names and ROS 1 nodes. Those files are not installed by the current CMake package
and are not ROS 2 launch instructions. There is no current RViz or Gazebo launcher
in this package. Old experimental-variant and rollback notes were removed because
their described files are not present in this checkout.
