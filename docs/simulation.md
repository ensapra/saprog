# Whole-robot Gazebo simulation

Tracking: [issue #34](https://github.com/ensapra/saprog/issues/34).
Playback: [issue #35](https://github.com/ensapra/saprog/issues/35).
Native knee coupling: [issue #37](https://github.com/ensapra/saprog/issues/37).

Open Home's **Simulation** tab and choose **Start Gazebo**. The robot appears
in a separate Gazebo Harmonic window, initially paused, above a ground plane.
Use Gazebo's **Play** button to apply gravity and contacts. **Stop simulation**
and start again to reset. Closing the panel's owning desktop stops its Gazebo
process group. Home can detach this tab like its other panels.

## Snapshots and sequences

After **Start Gazebo** reports ready, choose a snapshot or sequence under
**Play in Gazebo** and press its play button. Then press **Play** in the Gazebo
window. A pose can be queued while the world is paused. The selectors initially
list the workspace snapshots and sequences; **Browse** selects another JSON
file, and **Snapshot folder** changes where sequence references are resolved.
**Refresh** lists newly saved files.

Playback reads the existing files and validates every sequence reference and
target before sending motion. Radian snapshot schemas 1 and 2 are supported;
other units are rejected. Targets are checked against the installed URDF limits, including
the source knee mimic relationship. Files outside those limits are rejected
with a message in the panel.

Sequence fixed durations, waits, loops and arrival steps use simulation time.
Pausing Gazebo freezes the sequence. Arrival requires every simulated joint to
remain within 0.04 rad of its target for 0.2 simulated seconds, with a timeout
of 20 simulated seconds. This is simulation feedback, not physical arrival.
Motion speed and torque settings, including per-joint overrides, control the
simulated actuators. End `on` holds the final target; end `off` releases the
simulated joints. Loops continue until stopped.

**Stop playback / hold** cancels the sequence and holds the current measured
simulated pose. **Release simulated joints** cancels and removes actuator
forces. Both work while Gazebo is paused and take effect on its next physics
step. **Stop simulation** closes the world and its playback worker.

**Other panels → Simulation** opens standalone controls. To use simulation
without starting Home or its hardware backends:

```sh
bash tabs/launch_simulation.sh
```

The simulation launcher starts no controller, USB preflight, or IMU backend.
Home itself retains its normal hardware startup behavior. No simulation state
is connected to the hardware command, joint state, IMU, or TF topics. Each
desktop simulation session has its own Gazebo transport partition.

## Setup in Ubuntu 24.04 WSL

Install the ROS Jazzy Gazebo integration once:

```sh
sudo apt update
sudo apt install ros-jazzy-ros-gz-sim
```

From the source workspace, build the two hardware-independent packages. All
generated files stay in the WSL home directory:

```sh
source /opt/ros/jazzy/setup.bash
colcon --log-base "$HOME/saprog-simulation/log" build \
  --packages-select saprog_description saprog_simulation --symlink-install \
  --build-base "$HOME/saprog-simulation/build" \
  --install-base "$HOME/saprog-simulation/install"
```

The desktop launcher prefers this dedicated install, falling back to the
workspace install. Rebuild after changing installed resources if not using
symlink installation. For direct launch or automated inspection:

```sh
source /opt/ros/jazzy/setup.bash
source "$HOME/saprog-simulation/install/setup.bash"
ros2 launch saprog_simulation simulation.launch.py
# Server only (paused):
ros2 launch saprog_simulation simulation.launch.py gui:=false
```

## Model and limitations

The launch imports the installed [URDF](../urdf/urdf/URDF.urdf) directly, with
Gazebo's resource path pointing at the installed mesh package. There is no
simulation-specific robot description or name rewriting. Source link names use
a `_link` suffix so they differ from joint names, as required by SDF's frame
graph. Multiple links can reference the same STL. Joint names and hardware
mappings remain unchanged. The free base starts at 0.5 m in the URDF's native
orientation, with zero roll, pitch, and yaw.

The [world](../src/saprog_simulation/worlds/inspection.sdf) explicitly selects
**Bullet Featherstone** (`gz-physics-bullet-featherstone-plugin`) because this
Harmonic engine supports native mimic constraints. Gazebo's default DART engine
does not support them. There is no fallback to independent knee joints. See
the [Gazebo Harmonic release notes](https://github.com/gazebosim/gz-harmonic/blob/main/release_notes.md)
and [engine selection](https://gazebosim.org/api/sim/8/physics.html).
The world uses a 1 ms physics step, Earth gravity, and a static frictional ground
plane. No external world
assets are needed. The robot is passive until playback begins and can fold or
fall when unpaused. Desktop sessions attach the
[actuator plugin](../src/saprog_simulation/src/actuators.cpp) to the existing
Gazebo model at runtime. No alternate URDF is generated. Its position controller
applies bounded joint forces with a speed-limited reference; it does not
teleport the joints or pin the base. Raw AX speed settings use 0.111 rpm per
unit and are capped by URDF velocity limits. Torque settings scale the URDF
effort limit by `value / 1023`. These are approximate actuators with simulation
PD gains, not calibrated motor dynamics.
The source knee mimic multiplier and offset are enforced by native physics
constraints, including while the robot is passive or its simulated motors are
released. Only the 12 driven joints receive motor forces; the four knee
followers remain passive, with constraint forces transferred through the
solver. Their measured positions still participate in arrival checks, and
derived follower targets are validated against the URDF limits.
The linear mimic relationship itself remains an approximation of the physical
linkage; native enforcement does not establish the real linkage geometry.
Stop and restart Gazebo after updating to reload the engine and actuator plugin.
Collision meshes, friction, exported inertias and joint calibration still need
physical validation. See [single-leg calibration](leg-simulation.md) before
using this model for locomotion or hardware motion planning.

## Verification

From the workspace, with the ROS and dedicated install sourced:

```sh
PYTHONPATH=src/saprog/scripts python3 -m unittest discover \
  -s src/saprog/test -p test_simulation.py
python3 src/saprog_simulation/test/check_physics.py
python3 src/saprog_simulation/test/check_playback.py
```

The first command checks process ownership, forced cleanup, spawn readiness,
timeouts, and the Tk panel when a display is available. The second starts an
isolated headless world, verifies the mesh-bearing robot exists while paused,
steps physics to measure falling under gravity, and checks it settles above the
floor. It always stops its launch group. It does not validate physical accuracy.
The playback check starts the same session used by the desktop, attaches the
actuators, queues a pose while paused, verifies actual joint arrival, executes
timed sequence steps and waits, checks release on completion, and cancels while
paused. It also stops its owned processes.
The same test checks all four source mimic relationships before any motor
command, after reaching a pose, and over multiple samples after motor release.
