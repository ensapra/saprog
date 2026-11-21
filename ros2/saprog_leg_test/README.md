# Single-leg crank test

Open **Single Leg Test** in the desktop and press **Start**. The page starts its
own `saprog_leg_test/leg_test_node` provider through the existing ROS manager.
Opening the tab alone starts nothing. The full-robot Gazebo simulator can run
independently. This node does not use USB or publish hardware commands.

## What is simulated

The upper-leg mount is clamped. Its motor shaft O and knee pivot D are fixed.
The motor drives crank O-A. A passive connecting rod A-B connects to the knee
arm D-C with a physical pin constraint at B=C. The lower leg extends from D to
foot F. Three revolute joints and one loop constraint leave one mechanical
degree of freedom. MuJoCo integrates gravity, link inertia, passive-joint
damping, the crank actuator and an optional downward force at the foot.
There are no mimic joints and no motor on the knee or rod.

The initial pose is calculated by intersecting the rod and knee-arm circles.
Subsequent poses come from physics, not this analytic calculation. The chosen
assembly branch is fixed by the reference pose. Exact distance extrema across
the servo range reject unreachable geometry and toggle positions before start.

**The supplied geometry is an editable example, not a reconstruction validated
against Saprog CAD.** The repository does not define all connecting-rod pins.
The servo travel defaults reflect the existing model, but the mechanical zero,
rod dimensions, masses and motor response still require calibration. The model
assumes a planar four-bar with parallel pin axes; confirm that this matches the
physical topology. It does not model backlash, flexibility, bearing friction,
thermal effects or a measured AX actuator response. The rig hangs freely and
has no ground contact. It is a mechanism bench test, not a walking controller.

## Controls and measurements

- Set X/Z knee-pivot offsets relative to the motor, pin radii and rod length in
  millimetres. Foot angle is measured from the knee pin arm to the foot vector.
- Servo values are radians. Positive servo motion rotates the crank clockwise
  in the side view: mechanical crank angle = crank zero - servo angle.
- Set link masses, motor torque/no-load speed, PD gains and foot load before
  starting. Stop the rig to edit these settings.
- **Move** sets a servo target. **Sweep** cycles between its limits with a cosine
  trajectory. Period is in simulation seconds. Targets are ramped at the motor
  speed setting; a simple torque-speed envelope limits driving torque. External
  forces can backdrive the motor faster than this speed.
- **Hold** stops a sweep at the current servo position. **Pause**, **Reset** and
  **Stop** operate only this rig. Reset preserves the paused state.
- The side view shows actual simulated pin positions and the foot trail. The
  graph shows servo, target and knee angles. Knee angle is the absolute angle
  of the knee pin arm from +X, not a calibrated Saprog knee encoder reading.
- Torque, speed and pin-closure error are shown live. A closure error over 2 mm
  pauses the simulation with an error. Smaller numerical closure errors remain
  visible rather than being hidden by the drawing.
- **Save settings** / **Load settings** preserve a complete configuration as
  versioned JSON. Saving does not certify the geometry as calibrated.

For calibration, obtain the two fixed pivot frames, both connecting-rod pin
locations, reference servo angle and assembly branch from CAD or measurements.
Compare at least three physical poses across the range before trusting loads.

## Runtime setup

ROS 2 Jazzy and the existing Windows/WSL desktop bridge are required for the tab.
The ROS node additionally requires `mujoco>=3.2,<4` in the Python environment
used to build and run this package. MuJoCo runs headlessly; neither OpenGL nor a
separate simulator window is used. The desktop only needs its normal dependencies.

Use your ROS Python environment or a virtual environment with system ROS
packages available. For example, in Ubuntu 24.04 with Python venv support:

```sh
source /opt/ros/jazzy/setup.bash
python3 -m venv --system-site-packages ~/saprog-leg-env
source ~/saprog-leg-env/bin/activate
python -m pip install 'mujoco>=3.2,<4'
cd /path/to/saprog/ros2
colcon build --packages-select saprog_leg_test
```

If MuJoCo is missing from the generated ROS entry point's interpreter, the node
automatically restarts itself with `~/saprog-leg-env/bin/python`. This environment
must include system site packages so ROS remains available. You can select a
different interpreter with `SAPROG_LEG_TEST_PYTHON` in the WSL launch environment.
This fallback applies only to the single-leg node, including after desktop
rebuilds; it does not change the shared bridge or hardware node interpreters.

The desktop builds missing providers on demand. A missing MuJoCo installation
is reported on **Start**, while the node remains available to report status.

Standalone, source the built workspace and run:

```sh
ros2 run saprog_leg_test leg_test_node
```

All ROS messages are `std_msgs/msg/String` containing JSON:

| Topic | Contract |
| --- | --- |
| `/saprog/leg_test/command` | `id`, `action`, and action arguments |
| `/saprog/leg_test/result` | Matching `id`, `success`, `message` after execution |
| `/saprog/leg_test/status` | 20 Hz state, angles, torque, closure error and X/Z pin points |

Actions: `start` (optional `config` in SI units), `stop`, `pause`, `resume`,
`reset`, `target` (`angle`), `sweep` (`lower`, `upper`, `period`), `stop_sweep`.
The schema and bounds are in [geometry.py](saprog_leg_test/geometry.py).

## Validation

From the repository root, in a desktop Python environment:

```sh
python -m pip install 'mujoco>=3.2,<4'
python -m unittest discover -s desktop/tests -p test_leg_test.py
```

Physics tests exercise real MuJoCo dynamics, loaded sweeps, loop closure,
nonlinear knee motion and lifecycle operations. Without MuJoCo those physics
tests skip; routing, geometry and Qt tests still run. These checks validate the
example mechanism, not its agreement with the physical robot.

An opt-in [live ROS test](test/ros_smoke.py) launches the installed executable
in ROS domain 83, exercises start/move/loaded sweep/pause/reset/resume/stop,
checks pin closure, and cleans up its process. In WSL, after sourcing ROS and
the built workspace:

```sh
python3 ros2/saprog_leg_test/test/ros_smoke.py
```
