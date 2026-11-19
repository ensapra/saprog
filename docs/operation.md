# Operation and file contracts

## Start and connect

Runtime requires Linux/WSL, ROS 2 Jazzy, `colcon`, `flock`, Tk and the package's
[ROS dependencies](../src/saprog/package.xml). Run from the workspace:

```sh
bash ./launch_home.sh
```

Home builds all packages before opening; a failed build prevents launching stale
applications. It starts the shared servo controller and independent IMU backend.
Tool launchers reuse installed executables; restart Home after source changes.
Home closes its owned backends on exit; the servo controller also stops after the
last launched tool closes. Tools launched from Home reuse the controller PID and
leave its lifetime with Home. Direct window launchers own their controller and
stop it when their window exits. Standalone IMU launch owns its separate backend;
closing the monitor alone does not stop it. Home owns the Robot view renderer and
its state processes, which stop with Home or when **Stop view** is used. The
launcher checks that Home's controller PID is still the controller launcher
from this workspace; this prevents a recycled PID from being reused. It sends
TERM and allows owned child processes up to two seconds to exit before KILL.
Repeated termination signals are ignored during cleanup so owned processes get
the same bounded shutdown path.
The `verify` mode checks ROS/workspace setup without starting hardware.

```sh
bash ./tabs/launch_snapshot_creator.sh
bash ./tabs/launch_snapshot_player.sh
bash ./tabs/launch_timeline.sh
bash ./tabs/launch_static_animator.sh
```

Home includes a **Static Animator** tab with **Creator / Player** and **Timeline**
tabs. Creator and Player are visible together in an equal, adjustable side-by-side
split.
They share movement speed and torque strength settings. Selecting a pose
loads its values into the Creator without sending commands; use **Play selected
pose** to move the robot. The Timeline has its own tab. The standalone Static
Animator launcher uses the same layout, and the individual launchers above
remain available. Static Animator accepts the same `--directory PATH` option as
the snapshot tools.

Home and standalone Static Animator share one view assembly. Closing either asks
Timeline to resolve unsaved edits and any active action first. If cancellation or
the terminal action result is unknown, the window and its registered ROS nodes
remain alive until the backend reports a terminal result. Home stops its owned
backend processes only after that safe close path completes. Closing the IMU
display does not call reset or clear-zero; calibration and zero are backend-owned
while that backend remains running.

Under WSL USB devices are shared with usbipd-win. The IMU helper requests the
one-time share through Windows UAC when needed; approve the administrator prompt.
If that prompt is unavailable or denied, run `usbipd bind --busid 1-2` in
elevated Windows PowerShell, using its current BUSID.
Servo launchers attach hardware ID `0403:6014` by default; override with
`SAPROG_USB_BUSID` or `SAPROG_USB_HARDWARE_ID`. The configured `device_name` must
appear as a character device before ROS starts. Closing the app does not detach USB.
The IMU launch attaches hardware ID `1B4F:9D0F` under WSL; override with
`SAPROG_IMU_USB_BUSID` or `SAPROG_IMU_USB_HARDWARE_ID`. Its helper selects the
matching serial tty and emits its path on stdout. On native Linux no USB attach
is attempted; pass `device:=/dev/tty...` when automatic detection is unsuitable.

New `*.sh` files in [tabs/](../tabs/) appear in Home; use
[_launch_window.bash](../tabs/_launch_window.bash) with `window EXECUTABLE` for
shared setup/readiness/cleanup. The IMU display is embedded in Home. The Robot
tab embeds the accelerated RViz2 renderer in its dark viewport; it requires
Linux/WSL, Tk on X11, a working OpenGL display, and the RViz/Qt/X11 dependencies
declared by the package. Select the Robot tab before starting the view. Drag in
the viewport to orbit; resizing Home resizes the render surface, and hiding the
tab hides it with the parent window. Use **Stop view** to stop its renderer and
live robot-state processes. Closing Home stops them as well. Unsupported display
backends and renderer startup errors appear in the tab instead of opening a
separate desktop window.

## Snapshots and torque

The Home window's **Controller** tab can bind gamepad buttons to snapshot and
sequence files. The tab shows every available physical button and D-pad
direction, with a dropdown for each action. Controller input uses SDL through
`python3-pygame`. Under WSL,
When running under WSL, Home polls the controller through native Windows Pygame
instead of requiring a WSL `/dev/input` or `xpad` device. Leave the controller
shared but unattached in Windows. Approve a Windows USB sharing prompt if one
is shown. WSL takes exclusive access only to devices explicitly attached with
`usbipd`. The default hardware ID is
`2E24:1125`; override it with `SAPROG_CONTROLLER_USB_HARDWARE_ID` or set
`SAPROG_CONTROLLER_USB_BUSID` to the intended BUSID when multiple devices match.
Mappings are saved in the user's Saprog configuration directory. The Home /
middle button mapping toggles all-joint torque using the latest fresh torque
state reported by `/saprog/joint_states`; it waits for every configured joint
to report a known state before acting.

Creator edits radian targets; **Save targets** saves all typed values without
moving or needing fresh telemetry. Each servo shows its live position and torque
state. **Fix** sends its typed target and enables torque in the same position
command; torque stays on. **Release** disables torque. The button follows
reported torque state, not command publication. Layout: front-left 31..33,
front-right 11..13, back-left 41..43, back-right 21..23.

Player automatically discovers JSON snapshots; **Refresh** reloads names and
**Browse** selects external files. Files are re-read/validated at playback.
Both tools default to [snapshots/](../snapshots/); `--directory PATH` overrides it.

- Speed 1..1023 and torque strength 0..1023 apply to subsequent moves; zero strength
  is rejected before GUI movement. Fix uses these slider settings. Player persists
  strength in `saprog/snapshot_player.json` under `$XDG_CONFIG_HOME`/`~/.config`
  (Windows: `%APPDATA%`). Windows keep independent slider settings.
- Pose playback sends one synchronized command and immediately unlocks controls for further commands. It does not wait for arrival telemetry or automatically release torque. Use the manual torque-release controls when needed; command publication does not confirm physical arrival.
- **Release all joints** stays available during playback and pending torque operations. It discards queued pose commands and retries torque-off requests, including when only some joint services are available. Unavailable services are reported; already transmitted motion cannot be undone.
- Torque operations require acknowledgements; missing responses report unknown
  state. Closing a snapshot window does not change torque.
- Schema 1 saves `schema_version: 1`, `name` and `positions` keyed by joint
  names such as `servo_11`, with radian values.
  Schema 2 remains readable with explicit `units` of `radians` or `normalized`.
  Normalized targets need measured limits; all targets must be finite and match
  the configured joint set. Overwrites ask first; writes replace JSON atomically.

## Timelines

Timeline edits transition/snapshot pairs and independent wait rows. Reorder,
duplicate or remove whole entries. Global, Leg and Custom controls set speed and
strength (1..1023); expanding groups copies values, combining them averages values.
Legacy torque-off transitions are rejected.

**Until reached** advances on fresh complete arrival telemetry, without the
snapshot player's stable dwell. **X seconds** advances after a finite non-negative
duration, including zero, regardless of arrival; duration starts after command
transmission. Wait rows send no commands. Speed controls servo travel separately.

The [ROS action](../src/saprog/action/PlayTimeline.action) validates the whole
sequence, applies per-joint settings and sends synchronized position packets.
Monotonic worker scheduling aborts on wakeup lateness over 20 ms or bus acquisition
over 250 ms; this is not hard real-time. An in-flight transaction or preparation
can still delay execution. **Stop** requests cancellation; transmitted commands
can finish. Cancellation is rejected once the final torque operation begins.

**Loop timeline** repeats until stopped and skips the end action. Successful full
non-looping playback applies **After timeline: Torque on/off** and verifies it;
failure/cancellation skips it. Torque on holds freshly read positions using
controller defaults. **Test selected step** runs once without an end action.

Saved schema-2 timelines contain snapshot paths, not copies. Paths are relative
to the snapshot directory; external choices retain absolute paths. Changing a
snapshot changes future playback. Keep referenced files available and save
timelines outside the snapshot directory. Offline editing works without hardware.
Legacy embedded-pose migration currently has the ambiguity tracked in
[#3](https://github.com/ensapra/saprog/issues/3).

## Hardware diagnostics and monitoring

Limits are read from servo CW/CCW registers at startup and when Creator/Player
opens, via `/saprog/refresh_limits`. They stay in memory; incomplete reads prevent
position commands. With the controller stopped, [read_servo_limits.py](../scripts/read_servo_limits.py)
inspects limits without changing hardware/configuration.

Set `status_return_level` to match EEPROM (1 or 2); the application does not change
it. At level 1, writes are transmit-only and are not followed by register
readbacks; the bus caches requested settings after successful transmission.
At level 2, writes use status acknowledgements, with a register-read fallback
when an acknowledgement times out. `verify_goal_registers` optionally
reads synchronized goals for diagnostics. Reading a goal confirms acceptance,
not arrival. Check power, cabling, IDs, baud rate and competing port owners on
failure.

`/saprog/position_command` uses `JointState`: positions in radians, optional equal
speeds in rad/s (0.111 rpm per AX unit), and effort as raw torque limits.
Timeline actions support different per-joint speeds. `/saprog/joint_states`
publishes positions and observed torque state; `/saprog/temperatures` identifies
each servo through `header.frame_id` and reports Celsius.

Home's temperature graph retains five minutes, flags stale readings after 2.5 s,
and leaves gaps for missing data. Select traces and zoom from 1..300 seconds.
The Robot tab embeds RViz2 using live servo and IMU data. Select the tab, start
the view, then drag in the viewport to orbit; resizing Home resizes the render
surface. It requires Linux/WSL with Tk on X11 or XWayland and a working OpenGL
display. There is no separate-window fallback; unsupported display backends and
startup errors appear in the tab. Stop the view before restarting it; closing
Home also stops its renderer and state processes. IMU mounting/reference limits
are explained in [imu.md](imu.md). Experimental gait notes remain beside the
[walk-cycle snapshots](../snapshots/walk_cycle/README.md).
