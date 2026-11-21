# Saprog Gazebo simulation

The desktop **Simulation** tab starts this ROS 2 Python supervisor on demand in
the existing WSL bridge. Opening the tab does not start Gazebo. The supervisor
imports the selected URDF with the installed SDFormat converter and owns one
headless Gazebo Harmonic server, plus an optional separate viewer.

## Requirements

Use ROS 2 Jazzy on Ubuntu 24.04 with the matching Gazebo vendor packages:

```sh
sudo apt install ros-jazzy-gz-sim-vendor ros-jazzy-gz-transport-vendor \
  ros-jazzy-gz-msgs-vendor ros-jazzy-sdformat-vendor
```

The standard ROS desktop / ros_gz_sim installation commonly supplies these
already. Source the ROS environment before starting the node. No ros_gz_bridge,
ros2_control, RViz, camera, IMU simulator or GPU is needed for headless physics.
The optional viewer requires WSLg or another working Linux display.

The desktop's existing package launcher builds and starts `saprog_simulation`
when **Start** first requests its status topic. Its Python entry point is
`ros2 run saprog_simulation simulation_node`; launching it alone does not start
Gazebo. The selected URDF path is sent by the desktop and translated with
`wslpath` before reaching the node. No description package build is necessary.

## Controls

- **Start:** import the URDF, load a ground plane and enable `0 0 -9.81` gravity.
- **Pause / Resume:** control the physics clock. Gravity stays configured.
- **Reset:** restart the owned server with the same URDF and settings, clearing
  body motion. The current pause/viewer choices persist.
- **Stop:** terminate the owned server and viewer and remove temporary files.
- **Open / Close viewer:** attach or close a small Gazebo view while physics
  continues. Closing the viewer window itself also leaves physics running.

Joints move freely under gravity and mimic joints retain their native
constraints. This first version provides passive URDF physics; gait generation,
actuation and timeline playback can be added on top of this baseline.

The status shows simulation time, measured real-time factor, body height and
tilt. A factor of 0.5 means one simulated second per two wall-clock seconds.
Height is the model origin's world Z coordinate. Tilt is its angle from upright.

## Performance and isolation

Defaults are **no viewer**, **0.5× speed**, **4 ms physics steps**, 2 Hz desktop
status and at most 2 Hz transport feedback. The viewer uses a minimal scene with
no shadows and a 15 Hz scene-state broadcaster. No sensor, recording or joint
controller systems are loaded. Bullet Featherstone is selected because Harmonic supports native mimic
constraints with that engine. Raising step size reduces work but can destabilize
small, stiff mechanisms; lower it if contact or joint motion becomes unstable.
Lowering simulation speed reduces CPU demand without changing the physics step.

The supplied meshes, collision geometry, masses, inertia, joint axes, limits and
mimic relationships are retained. Only a temporary copy has mesh URIs resolved
and gravity enabled. The source URDF is never saved.
The checked-in `saprog_description` package can be resolved directly from the
URDF's parent package directory. Other package meshes use the ament index.

Each supervisor owns a unique Gazebo Transport partition. Its ROS topics live
under `/saprog/simulation`; it never subscribes to hardware position commands,
publishes hardware joint states, opens USB devices or changes physical torque.
Other desktop tabs retain their existing hardware startup behavior.

Gazebo processes are terminated when the supervisor receives normal ROS shutdown.
The desktop bridge stops its owned supervisor when the application exits.

## ROS protocol

All three topics use `std_msgs/msg/String` containing JSON:

| Topic | Content |
| --- | --- |
| `/saprog/simulation/command` | `id`, `action`, and action options |
| `/saprog/simulation/result` | matching `id`, `success`, `message` after completion |
| `/saprog/simulation/status` | `running`, `busy`, `paused`, `viewer`, timing, pose, error and joint mapping |

Start options: `urdf_path` (required Linux path), `step_size` (seconds, default
0.004), `real_time_factor` (default 0.5), `spawn_height` (metres, default 0.3),
`paused` and `viewer` (both false). Other actions are `stop`, `pause`, `resume`,
`reset`, `show_viewer`, and `hide_viewer`.
Long actions run on one worker thread; concurrent requests get a busy response.
Gazebo startup has a 25-second readiness deadline. Desktop command delivery and
completion have a 90-second deadline. Errors are surfaced in the tab.

## Validation

Offline desktop tests cover lazy startup, defaults, command isolation, result
acknowledgements, Windows path conversion and preserved mimic constraints.
Run the repository's desktop unittest suite as described in [testing](../../docs/testing.md).

The opt-in [live test](test/test_gazebo_smoke.py) imports the actual URDF into
headless Gazebo, verifies a gravity drop, pauses, resets
and checks process cleanup. Enable `SAPROG_RUN_GAZEBO_TESTS=1` in a sourced Jazzy
Linux environment and run unittest discovery in this package's `test` directory.
It starts no physical robot nodes. It does not establish hardware gait quality.

References: [Gazebo physics engines](https://gazebosim.org/api/sim/8/physics.html),
[Harmonic mimic support](https://github.com/ros-controls/gz_ros2_control/issues/340).
