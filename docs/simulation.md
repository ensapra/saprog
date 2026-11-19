# Whole-robot Gazebo simulation

Tracking: [issue #34](https://github.com/ensapra/saprog/issues/34).

Open Home's **Simulation** tab and choose **Start Gazebo**. The robot appears
in a separate Gazebo Harmonic window, initially paused, above a ground plane.
Use Gazebo's **Play** button to apply gravity and contacts. **Stop simulation**
and start again to reset. Closing the panel's owning desktop stops its Gazebo
process group. Home can detach this tab like its other panels.

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
mappings remain unchanged. The free base starts at 0.5 m with a
180-degree pitch, matching the current Robot view's model orientation.

The [world](../src/saprog_simulation/worlds/inspection.sdf) uses a 1 ms physics
step, Earth gravity, and a static frictional ground plane. No external world
assets are needed. This is passive physics inspection: there are no motor
controllers to hold a standing pose. The robot can fold or fall when unpaused.
The existing knee mimic ratio is an approximate linkage. The installed default
physics engine reports that it cannot enforce mimic constraints, so knee hinges
move independently; this is not a validated mechanical linkage model.
Collision meshes, friction, exported inertias and joint calibration still need
physical validation. See [single-leg calibration](leg-simulation.md) before
using this model for locomotion or hardware motion planning.

## Verification

From the workspace, with the ROS and dedicated install sourced:

```sh
PYTHONPATH=src/saprog/scripts python3 -m unittest discover \
  -s src/saprog/test -p test_simulation.py
python3 src/saprog_simulation/test/check_physics.py
```

The first command checks process ownership, forced cleanup, spawn readiness,
timeouts, and the Tk panel when a display is available. The second starts an
isolated headless world, verifies the mesh-bearing robot exists while paused,
steps physics to measure falling under gravity, and checks it settles above the
floor. It always stops its launch group. It does not validate physical accuracy.
