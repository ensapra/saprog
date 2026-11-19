# Robot view

The desktop Robot tab launches the same ROS 2 Jazzy entry point as the
standalone command:

```sh
ros2 launch saprog robot_view.launch.py
```

The desktop passes its X11 parent window ID and current viewport size. The
launch file resolves `saprog/config/robot_view.rviz` and
`saprog_description/urdf/URDF.urdf` from the installed ament package shares;
it does not depend on source-tree-relative paths. The robot description package
installs the URDF and meshes. ROS 1 `display.launch` and `gazebo.launch` were
removed because they use catkin-era packages and paths and are not supported
entry points for this ament/ROS 2 workspace.

## Display contract

Servo IDs are mapped to existing URDF joint names in
[robot_view_model.py](../src/saprog/scripts/saprog_desktop/robot_view_model.py).
Tests compare every mapped name to the URDF. The replacement description uses
`front_right_joint_13`, replacing the old export typo `froint_right_joint_13`.

The bridge publishes a complete display joint state at 20 Hz, initially zero.
Partial telemetry replaces only the supplied finite positions; other joints keep
their previous display value. The URDF axes use logical servo radians directly, so the bridge forwards
angles unchanged. Axes for 11, 31, 12, 22, 13, 23, 33 and 43 were reversed
from the CAD convention, with their bounds mirrored and the four knee mimic
multipliers negated to preserve the corrected motion. This does not change
hardware calibration, commands or stored poses. URDF bounds are model travel
bounds, not verified measured servo limits (see [leg simulation](leg-simulation.md)).
These are preview values, not measured feedback
or commands. This keeps all TF branches (including URDF mimic joints) available
without hardware. Missing TF makes RViz use its white, unlit error material and
leaves meshes without their correct placement.

The startup `base_height` bridge parameter defaults to 0.183 metres: the supplied
zero-angle meshes reach -0.18281 metres below the base. The orbit camera looks
at z=0.08 metres. Height is a fixed visual reference; changing joint angles or
IMU orientation does not solve ground contact or closed-loop linkage geometry.

The installed bridge entry point must retain its Python shebang and LF line
endings: ROS launches it directly, not through an explicit Python interpreter.
Without that header, the bridge cannot start and the renderer has no base or
joint transforms even though it can load the URDF. Import-only tests do not
cover this failure. The executable regression test launches the script directly
and waits for its complete joint-state message, then verifies clean SIGINT
shutdown. The bridge tolerates ROS having already shut down its context.

Validation for issue #30: nine robot-view tests passed under WSL/Jazzy, including
the executable entry point. Both ROS packages built successfully. The installed
`ros2 launch` path was exercised with a mapped Tk viewport and its screenshot
confirmed assembled, shaded gray meshes and red cranks. All 17 mesh links also
passed TF checks. Physical mounting and linkage accuracy remain unvalidated.
The supplied replacement URDF/meshes are pre-existing workspace changes and
are not included in the bridge fix commits.

The bridge publishes the IMU orientation as `world -> base` without a measured
sensor-to-base mounting transform. The quaternion is sensor-relative: zero uses
the saved `imu_zero` reference, while clear-zero returns to the estimator's
local `imu_world` reference. Neither is a calibrated map/world alignment. When
orientation updates stop for more than 0.5 seconds, the bridge returns the base
to identity rather than holding an old orientation. The floor and base height
are visual references, not measured robot position. See the [IMU contract](imu.md)
and open mounting work in [issue #4](https://github.com/ensapra/saprog/issues/4).

## X11 smoke check

Run after building and sourcing the installed Jazzy workspace, with the Home
window on an X11 display:

1. Select Robot, start the 3D view, and confirm RViz appears inside the Tk
   viewport with no separate top-level RViz window.
2. Resize the window and verify the render panel tracks the viewport.
3. Publish a known joint state and orientation; confirm the mapped leg and base
   respond. Stop orientation publication for over 0.5 seconds and confirm the
   base returns to identity.
4. Stop and restart the view, then close Home while it is running. Confirm no
   renderer or launch child remains and the UI does not wait for process exit.
5. Repeat with an invalid `DISPLAY` and with a launch executable that exits
   during startup; confirm the UI reports the failure and removes its readiness
   marker.

These checks require a Linux/WSL X11 session and an installed ROS 2 Jazzy
workspace. The hardware-free installed-launch rendering check above ran on WSLg;
it does not establish live servo or IMU alignment.
