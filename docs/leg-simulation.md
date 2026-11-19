# Single-leg simulation preparation

Tracking: [issue #33](https://github.com/ensapra/saprog/issues/33).

The first milestone is a front-right leg geometry inspector, independent of the
desktop and servo controller. It is a kinematic inspection tool, not yet an
accurate linkage or gravity/contact simulation. Working physical stand-up and
rest animations are reference configurations for calibration.

## Launch the inspector

In Home, open **Other panels → Leg Inspector**. Home discovers its
[launcher](../tabs/launch_leg_inspector.sh) automatically, including while Home
is already running. The launcher uses the workspace install when available,
falling back to the dedicated `$HOME/saprog-leg-lab/install` build. Home's existing
controller lifecycle still applies when opening tools from Home; the leg
inspector itself starts no hardware backend.

The same launcher works standalone without starting Home or its controller:

```sh
bash tabs/launch_leg_inspector.sh
```

Use Ubuntu 24.04 WSL with ROS 2 Jazzy sourced and an installed workspace containing
`saprog_description` and `saprog_leg_lab`. The additional runtime dependency is
`ros-jazzy-joint-state-publisher-gui`. Build artifacts belong under the WSL user's
home directory, as described in [testing](testing.md).

Install the slider dependency once in Ubuntu (requires the Ubuntu sudo password):

```sh
sudo apt install ros-jazzy-joint-state-publisher-gui
```

For the dedicated build created for this milestone, start with:

```sh
source /opt/ros/jazzy/setup.bash
source "$HOME/saprog-leg-lab/install/setup.bash"
ros2 launch saprog_leg_lab inspect.launch.py
```

RViz shows the base and front-right leg, with joint-frame axes. A separate
standard ROS joint slider window controls the three driven model joints. The
passive lower-leg hinge follows the source model's `2.4` mimic multiplier,
retained as the working approximation. Sliders start at the known working
`stand` values (`0`, `0.5`, and `0.4` radians), so compare against the full Robot
view while the real robot is in that same pose. The slider tool is independent;
it does not follow live hardware after startup. Closing either window stops the launch.
The inspector preserves coupling from the [source model](../urdf/urdf/URDF.urdf).
All joint states,
description and TF topics are isolated under `/saprog/leg_lab`. No servo or IMU
backend is started, and nothing publishes to `/saprog/position_command`.

Use `description:=/absolute/path/to/model.urdf` to inspect revised geometry with
the same front-right link names. Its mesh package must be installed. Joint
origins, axes, geometry and limits come from the selected model.

## Measured servo limits

All 12 servos were reread through the running controller using
`/saprog/refresh_limits`; it reported success. The read used the existing serial
bus owner and did not change torque, goals, or servo EEPROM. The raw readings,
active zero/direction calibration and timestamp are recorded in
[measured_servo_limits.json](../urdf/config/measured_servo_limits.json).

The calibrated limits calculated from that read are:

| Joint | Lower (rad) | Upper (rad) |
| --- | --- | --- |
| First / sideways | -0.450408 | 1.085073 |
| Second / fore-aft | -0.962234 | 1.085073 |
| Third / crank | 0.266150 | 0.880342 |
| Passive knee (`2.4 * crank`) | 0.638760 | 2.112821 |

These values remain a recorded hardware readback, not the active URDF bounds.
The attempted URDF limit update was reverted after it coincided with a regression
in the full Robot view. The servo-to-model angle convention needs to be verified
before copying hardware limits into the description. Do not treat the report as
a guarantee that current URDF limits match the servos.

## Approximation retained for now

- The source lower-leg hinge uses a `2.4` mimic multiplier from the crank. A
  fixed angular ratio does not establish closure of the physical linkage.
- The existing STEP notes describe approximate bearing locations; the STEP
  component-center CSV is not a set of pivot axes or assembled motion mates.

The three sliders use calibrated servo radians, but send only isolated display
joint states. The knee follows the approximate angular ratio; geometric closure
and agreement with the physical joint axes remain unvalidated. A joint offset
should be considered only if the isolated model remains wrong when compared at
the same three angles as the full Robot view. A `90` degree command offset would
be `1.5708` radians, larger than the measured third-servo range, so it should
not be added to the servo angle without identifying a model-frame correction.

## Geometry needed for the next milestone

Choose one common coordinate frame and record the following from the original
assembled CAD or physical measurements, including units:

1. The three servo shaft axes and their mounting transforms.
2. Every passive hinge axis and the connectivity of each rigid link.
3. The crank radius and connection-pin locations on each participating link.
4. The foot contact point in the lower-leg frame.
5. One reference assembly pose, including which side of the linkage each pin
   occupies; this selects the correct closure branch.
6. The mapping from logical servo radians to mechanical angles (zero and sign),
   plus measured travel limits. Existing hardware calibration must be preserved.

For coplanar parallel-axis hinges, an annotated side view with pivot centers
and center-to-center distances can supply the linkage geometry. Nonparallel
axes require 3D pivot frames. An assembled CAD file with motion mates is useful;
an STL silhouette or component bounding-box center alone is insufficient.

## Calibration sequence

1. Confirm the first joint moves the leg sideways around the correct shaft.
2. Confirm the second joint moves the leg fore/aft around the correct shaft.
3. Confirm the crank rotates about its actual shaft; check the approximated
   knee motion against the physical reference pose and connection-pin locations.
4. Implement the actual geometric closure from the measured topology, rejecting
   unreachable poses and preventing jumps between assembly branches.
5. Compare stand and rest, plus at least one intermediate configuration, against
   CAD or measured foot positions. Two successful animations alone do not
   establish the linkage shape between their endpoints.
6. Only then add inverse kinematics and a physics model with gravity, collision,
   friction, actuator limits and measured masses/inertias.

Do not use inspection joint states as hardware commands or measured feedback.
