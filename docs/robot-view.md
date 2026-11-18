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
Tests compare every mapped name to the URDF. Preserve the export spelling
`froint_right_joint_13` until the URDF and any consumers can be migrated
together.

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

This smoke check requires a Linux/WSL X11 session and an installed ROS 2 Jazzy
workspace. It was not run in the Windows editing environment.
