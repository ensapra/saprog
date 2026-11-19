# Single-leg simulation preparation

Tracking: [issue #33](https://github.com/ensapra/saprog/issues/33).

The first milestone is a front-right leg geometry inspector, independent of the
desktop and servo controller. It is a kinematic inspection tool, not yet an
accurate linkage or gravity/contact simulation. Working physical stand-up and
rest animations are reference configurations for calibration.

## Launch the inspector

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
standard ROS joint slider window controls the three driven model joints and the
passive lower-leg hinge independently. Closing either window stops the launch.
The inspector removes the guessed mimic coupling only from its in-memory copy;
the [source model](../urdf/urdf/URDF.urdf) remains unchanged. All joint states,
description and TF topics are isolated under `/saprog/leg_lab`. No servo or IMU
backend is started, and nothing publishes to `/saprog/position_command`.

Use `description:=/absolute/path/to/model.urdf` to inspect revised geometry with
the same front-right link names. Its mesh package must be installed. Joint
origins, axes, geometry and limits come from the selected model.

## Current discrepancies

- The source lower-leg hinge uses a `2.4` mimic multiplier from the crank. A
  fixed angular ratio does not establish closure of the physical linkage.
- The front-right crank model range is `[-0.57, 0]` rad, whereas current working
  [stand](../snapshots/stand.json) and [rest](../snapshots/rest.json) servo targets
  for `servo_13` are `0.4` and `0.75` rad. These use different, currently
  unverified angle conventions. Do not widen limits or infer a sign correction
  merely to make those numbers fit.
- The existing STEP notes describe approximate bearing locations; the STEP
  component-center CSV is not a set of pivot axes or assembled motion mates.

The four sliders are **model angles in radians**, not calibrated hardware
commands. Independent slider motion can separate mating parts because the
closure constraint has deliberately not been guessed.

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
3. Confirm the crank rotates about its actual shaft; independently align the
   passive hinge at the reference pose and check all connection pins coincide.
4. Implement the actual geometric closure from the measured topology, rejecting
   unreachable poses and preventing jumps between assembly branches.
5. Compare stand and rest, plus at least one intermediate configuration, against
   CAD or measured foot positions. Two successful animations alone do not
   establish the linkage shape between their endpoints.
6. Only then add inverse kinematics and a physics model with gravity, collision,
   friction, actuator limits and measured masses/inertias.

Do not use inspection joint states as hardware commands or measured feedback.
