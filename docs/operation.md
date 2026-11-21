# Operation and file formats

## Start and connect

From the repository root, install [desktop dependencies](../desktop/requirements.txt)
and launch:

```sh
python -m pip install -r desktop/requirements.txt
python desktop
```

The Qt shell and document editor do not require ROS. The desktop's hardware
worker currently requires Windows and WSL. It starts a separate Python process
inside Ubuntu-24.04 by default, sources ROS Jazzy, and uses the
[ros2 workspace](../ros2/). Set SAPROG_WSL_DISTRO in the desktop environment to
choose another distribution; ROS_DISTRO selects another installed ROS setup.

ROS and the dependencies declared by the [servo](../ros2/saprog_dynamixel/package.xml)
and [IMU](../ros2/saprog_imu/package.xml) packages must be installed in WSL.
Requests reuse existing publishers. If a provider is absent, the bridge checks
its package with colcon build, then launches it. Build/install/log outputs are
created under ros2. Closing the desktop stops backend processes it owns.

The Statistics and Static Animator pages request telemetry when constructed, so
hardware startup may begin while another tab is selected. Opening the desktop
is not a promise that the serial ports stay unused until a movement button is
pressed. Startup itself does not enable servo torque.

For WSL serial devices, install usbipd-win on Windows, run usbipd list, and bind
the intended device from Administrator PowerShell. See [USB setup](../README.md#usb-setup).
The retained [servo helper](../scripts/ensure_usb.bash) uses device_name from the
installed servo config and attaches only when that character device is missing.
The [IMU helper](../scripts/ensure_imu_usb.bash) selects the attached sensor's
serial port. Both use [usb_attach.bash](../scripts/usb_attach.bash).

| Selection | Override | Default hardware ID |
| --- | --- | --- |
| Servo adapter | SAPROG_USB_BUSID or SAPROG_USB_HARDWARE_ID | 0403:6014 |
| IMU | SAPROG_IMU_USB_BUSID or SAPROG_IMU_USB_HARDWARE_ID | 1B4F:9D0F |

An explicit SAPROG_IMU_DEVICE in the WSL environment skips automatic IMU
attachment. The helpers still require suitable Linux device permissions.
Gamepads stay on the desktop host and are read by pygame.

## Statistics

The telemetry graph selects temperature, voltage or speed for individual servos
or all servos. It retains up to five minutes of samples and supports a 1-to-300
second display window. Temperature is in degrees Celsius, voltage in volts and
speed is the signed raw AX register magnitude. The IMU widget shows acceleration,
angular velocity, magnetic field and estimated orientation; see [IMU](imu.md).

## Static Animator

Creator fields and the Keyframes browser share speed and torque-strength
controls. Load copies a saved pose into target fields; Play sends the pose.
A servo's move button sends its target using the shared settings. Targets are
radians, and available measured bounds are applied when targets are loaded,
saved or sent. Motion Recorder captures and imports are clamped before entering
the timeline.

Entering the page requests current EEPROM angle limits through the existing
serial-port owner. A complete valid report updates the page's bounds and saves
[measured_servo_limits.json](../urdf/config/measured_servo_limits.json), preserving
existing model metadata. The read does not move servos or write EEPROM. On a
failed or timed-out read, cached bounds remain; leave and re-enter to retry.
If persistence fails after a valid read, fresh bounds remain active in this page.

Puppet Mode disables a servo's torque, cancels its queued movement and tracks
its measured position into the target field. That servo cannot be commanded by
Static Animator while Puppet Mode is active. Leaving Puppet Mode keeps torque
off; a later movement command enables it after writing the new goal. This is a
Static Animator control, not a global lock against other command senders.

Save targets writes all target fields as a named snapshot into the Keyframes
folder. It checks finite numeric values; loading, saving or sending a pose
clamps targets to available servo limits. Existing files require overwrite
confirmation. Release all torque cancels queued commands and requests torque-off
for all configured servos. A transmitted movement cannot be recalled.

## TimeLine

The TimeLine tab edits documents locally. Add snapshots and wait steps, reorder,
duplicate or remove steps, and choose Global Control, Leg Control or Custom
Control for transition speed and torque. Fixed-duration transitions use seconds;
until-reached transitions need joint-state feedback during execution.

The editor saves loop, on_end and smooth playback settings and can play the current timeline.
Smooth timed playback is saved with the timeline and applies in the Timeline,
Motion Recorder and Controller. For consecutive poses with a fixed duration, it
estimates each joint's speed from the distance between commanded poses and the
current step's duration, capped by that step's saved speed. The first pose,
steps after a wait, newly introduced joints and arrival-based steps use their
saved speeds. Timing is approximate: command latency, servo load and speed
limits can prevent the robot from reaching a pose within its allotted time.
New, load and closing the host window guard unsaved edits. Source browsers can point to different folders;
the opened document retains its own snapshot reference base.

## Motion Recorder

Select a leg and use **Copy selected leg** to copy its targets from every pose.
Select the destination leg, enter a pose offset, and use **Paste to selected leg**.
A positive offset shifts the copied poses later and wraps around the pose list;
wait steps are left unchanged. Every source pose must contain all three leg targets.
During playback, the active pose or wait is marked in the table.

New starts a timeline; Load opens an existing timeline from the sequence folder.
Pose and wait steps appear in order, with each leg's servo values in its own
column. **Play selected step** plays only that step once, using its motion,
mirroring and duration settings. Double-click a pose or use **Edit selected pose**
to open its targets in the existing Static Animator tab or focus its detached
window. Use **Apply pose to Motion Recorder** to update
that step within measured bounds, then save the recorder timeline to keep the
changes. **Finish editing** or loading another keyframe ends recorder editing.
Remove selected leg data clears only the selected leg from a pose; Delete entire
selected step removes that pose or wait. The first recording fixes the loop
length, and deleting a pose shortens that loop. Play the shortened loop to
record another leg over it. Save updates the loaded timeline, while Save as
writes a separate file; waits, mirroring, per-step motion settings, durations,
loop and on_end settings are preserved.

## Controller

The [Controller tab](../desktop/tabs/controller/__init__.py) detects USB/Bluetooth
gamepads through pygame on the desktop host. Select a device, choose snapshot
and sequence folders, then assign an action and optional JSON file to each input.
Buttons, D-pad directions and positive/negative axis directions are separate
inputs. An axis activates at an absolute value of at least 0.6. Active inputs
highlight. Refresh controllers rescans devices; Refresh files reloads available
JSON files recursively. Source folders and mappings are stored with QSettings;
mappings are keyed by controller GUID, so devices of the same type can share one.

| Action | Behavior |
| --- | --- |
| Torque on/off | Runs on a new press; torque-off also stops controller playback |
| Snapshot | Sends once per press, using the tab's snapshot speed and torque |
| Sequence (one shot) | Runs once per press and continues after release |
| Sequence (hold to repeat) | Repeats while held and stops when its input is released |

Both sequence modes use saved speed/torque, waits and transition durations;
they ignore the file's loop and on_end fields. The latest mapped press takes
control. A simultaneous torque-off press takes priority over other mapped
presses. Inputs already held at selection need release before they can trigger.

Disconnect, device selection, mapping or source-folder changes, Stop playback,
and any shared release-all request stop controller execution and cancel its
unpublished commands. Refreshing unchanged folders does not stop a sequence.
Stopping does not disable torque by itself or undo an already transmitted move.

[Playback](../desktop/tabs/controller/playback.py) waits for publication results
before sending another motion group. Until-reached transitions require three
fresh complete samples within 0.1 radians for every target; sample timestamps
must be newer than the final motion group's publication and no more than 2.5
seconds old when received by the bridge. Publication and telemetry timestamps
are compared using the WSL ROS clock, so Windows clock drift does not delay
arrival detection. Publication or
arrival waits time out after 30 seconds. A publication result confirms sending,
not physical arrival or backend acceptance. The controller loads stored measured
bounds when its playback object is created; restart the desktop to reload a
limit file changed later.

The bridge drains telemetry bursts within a bounded processing window, checking
desktop commands between windows. Stop clears playback state immediately,
cancels unpublished commands, and ignores late results; manual commands can be
sent immediately with their own speed and torque settings. An already published
movement may finish unless a new manual target or torque request overrides it.

## Simulation

The Simulation tab loads a selected URDF into Gazebo with gravity and a ground
plane. Start is explicit; the tab initially requests no ROS provider. Defaults
are a headless server, 0.5× simulation speed and a 4 ms physics step. An optional
viewer can be opened and closed while physics continues. Pause, Resume, Reset
and Stop affect only this simulation. Joints move freely under gravity with the
URDF's native mimic constraints. See the [simulation guide](../ros2/saprog_simulation/README.md)
for requirements, timing, physics settings and command isolation.

## Snapshot format

[validate_snapshot](../desktop/documents/snapshot_units.py) accepts schema 1 with
implicit radians or schema 2 with units explicitly set to radians. A pose can
contain a subset of the known servo names. Values must be finite numbers;
booleans and other units are rejected.

```json
{
  "schema_version": 1,
  "name": "Example pose",
  "positions": {"servo_11": 0.1}
}
```

Creator saves schema 1. This example describes the format, not a validated pose
for a connected robot. Servo conversion uses direction * (raw - zero) *
(300 degrees in radians / 1023). Hardware configuration is in
[saprog.yaml](../ros2/saprog_dynamixel/config/saprog.yaml).

## Timeline format

[timeline_storage.py](../desktop/documents/timeline_storage.py) saves schema 2
with references to snapshot files:

```json
{
  "kind": "saprog_timeline",
  "schema_version": 2,
  "steps": [
    {
      "snapshot": "example.json",
      "control_mode": "Global Control",
      "motion": {"speed": 512, "torque": 512},
      "overrides": {},
      "duration": {"mode": "seconds", "seconds": 1.0}
    },
    {"kind": "wait", "seconds": 0.5}
  ],
  "on_end": "on",
  "loop": false,
  "smooth_playback": false
}
```

Relative references resolve against the selected snapshot directory, not the
sequence file's folder. Relative paths escaping that directory are rejected;
absolute references are accepted. Keep referenced files available when moving a
sequence. If the browser's snapshot folder differs from the open document's
reference base at save time, the editor writes absolute references to preserve
the original targets.

Speed and torque are integers from 1 through 1023 in sequence transitions.
Per-servo overrides use the same pair. Wait durations are finite and nonnegative.
Transition duration mode is seconds or until_reached; omission defaults to
until_reached. Documents must contain at least one step.
The optional smooth_playback field is a boolean and defaults to false for older timelines.

Schema-1 timelines with embedded snapshots are migrated in memory by matching
snapshot name, joint set and positions against files in the selected directory.
Migration fails if there is no matching pose. Loading does not rewrite the source
file; saving writes schema 2 through atomic replacement.
