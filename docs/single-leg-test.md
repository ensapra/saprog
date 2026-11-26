# Single-leg URDF and joystick foot control

The desktop **Single Leg Test** loads
[front_right.urdf](../urdf/urdf/front_right.urdf), a single-leg extraction of
the repository robot with its CAD meshes. Selecting another leg extracts that
branch from the full robot description. **Load URDF** accepts a matching
single-leg file or the full robot. The URDF preview needs only the desktop
dependencies. ROS and a successful EEPROM limit read are required for motion
preview; manual model inspection is available offline.
Meshes are simplified for display; the foot estimate and kinematic calculations
use the original geometry. The view updates at up to 20 Hz during motion.
Live telemetry bursts are combined into the latest complete fresh pose on each
display tick. Static geometry, joint frames and mesh images are cached; exposure
or foot-trail repaints do not redraw the mesh unless the pose, camera or size changes.
The foot-path solver uses an analytic Jacobian including the knee coupling.

## Match received servo data to the model

Press **Connect live feedback** to request `/saprog/joint_states` through the
shared ROS manager. Positions are matched by `name`, independent of array order;
all three mapped servos must have reported within two seconds before the pose
updates. Missing or stale feedback holds the last pose and is labelled on screen.
Out-of-range readings remain visible instead of being clamped. Stop/disconnect
releases this feedback subscription. Entering the page refreshes the shared
hardware limits through the same process as Static Animator.

| Leg | Servo → URDF joint mappings |
| --- | --- |
| Front right | `servo_11` → `front_right_11`, `servo_12` → `front_right_12`, `servo_13` → `front_right_joint_13` |
| Back right | `servo_21` → `back_right_21`, `servo_22` → `back_right_22`, `servo_23` → `back_right_joint_23` |
| Front left | `servo_31` → `front_left_31`, `servo_32` → `front_left_32`, `servo_33` → `front_left_joint_33` |
| Back left | `servo_41` → `back_left_41`, `servo_42` → `back_left_42`, `servo_43` → `back_left_joint_43` |

Feedback is already in logical servo radians, including encoder zero and
direction calibration. Start with `q_joint = 1 × servo_radians + 0`. The URDF's
joint axes and origin rotations are applied separately; do not apply the raw
encoder direction or crank zero again. Edit **Received servo name**, **Gain**,
**Offset**, then press **Apply calibration**. Gain -1 reverses a model joint;
offset is in radians. Hardware bounds are read-only labels supplied by the shared
[EEPROM limits service](../desktop/servo_limits/). The solver uses exactly those
servo bounds, without intersecting them with old URDF stops. A complete fresh
report updates every consumer; failed reads clear the bounds and prevent path
generation. No stored limit fallback or editable per-leg limits exist.

The live metrics show both the received servo value and resulting joint angle.
Manual **Servo pose / received** fields let you check the mapping without ROS.
The preview requests telemetry and sends no position or torque commands.

## Control the foot with a joystick

The **Joystick foot control** panel, to the right of the leg view, maps two analog
stick axes to foot velocity.
Left/right changes base X; up/down changes base Z. Base Y stays fixed.
These directions follow the robot base, independent of the camera.
The controlled point is the calibrated foot dot.

1. Click **Detect controllers** and select your controller. The first two axes
   are used by default; foot mappings are saved separately from the **Controller** tab.
2. Set **Foot speed** (default 10 mm/s). If needed, open **Controller setup and
   motor settings** to change axis assignments, reverse a direction, adjust the
   15% dead zone, or set the live servo speed ceiling and torque. Move the stick
   and use the axis readings there to identify its axes; the two axes must differ.
   These settings are saved separately for each controller type.
3. Click **Preview stick**, then center the stick once to begin. Small deflections
   move slowly; full deflection reaches the selected speed. Diagonal speed is
   capped to that same maximum. Holding the stick continues moving, even without
   new axis-change events. Centering holds the target. Current EEPROM
   limits are required even for preview.
4. For hardware, choose a physical controller button under **Hold to move (live)**
   and click **Control real leg**. The live **Controller buttons**
   readings identify which generic button number is pressed on your controller.
   The controller waits for fresh feedback from all three mapped servos and
   captures the actual foot position, rather than moving toward the manual
   preview pose. The status names the physical controller button to hold. Hold it
   with the stick centered. Live control first moves the selected leg's x1 servo
   (11/21/31/41) to **0 rad**, waits for feedback, then keeps its target fixed at zero
   while the remaining two servos produce X/Z movement. Zero must be inside its
   current EEPROM limits. Once x1 reaches zero, move the stick.
   Releasing that button stops new targets; center the stick while holding it
   again before resuming. Servo speed ceiling and torque apply only to live output.
5. **Stop joystick**, changing the mapping or leg, hiding/closing the tab, controller
   loss, stale feedback, a command failure or timeout, changed limits, another
   control publishing a position, and **Release all torque** stop joystick output.
   Unpublished owned commands are cancelled; a target already sent may finish.

Each tick requests an incremental horizontal/vertical
movement with the calibrated knee coupling. The current EEPROM servo limits and
linkage reach determine how far the foot can move. A step approaching a limit is
shortened along the requested direction; further outward motion is blocked while
reversing the stick moves back immediately. Rejected motion is not accumulated.
Live output starts at the measured pose, then integrates a continuous target so
small stick movements accumulate across the servos' encoder resolution and
position deadband. Only one command awaits publication at a time. Motion time
while awaiting publication is retained, with catch-up capped at a quarter second.
Live output gives each servo the speed needed for its joint increment, capped by
the selected speed ceiling, instead of driving every small waypoint at the ceiling.
The Windows bridge checks outgoing commands at least every 10 ms while idle to
avoid adding a 100 ms wait between jog updates.
The servo speed ceiling also scales each increment. If the servos fall behind
the target, new increments pause rather than accumulating indefinitely; reversing
the stick rebases to the measured pose and cancels unpublished movement in the old
direction. Controller state is read directly on each motion tick, so a held stick or
a delayed display update does not cause a stale-input stop. Disconnects and invalid
readings still stop output. Feedback is rendered in live mode; preview renders
the solved target pose. Physical straightness
still depends on calibration and servo tracking. Existing button/sequence mappings
in the **Controller** tab remain independent; avoid assigning those actions to the
same stick directions or enable button while using live joystick control.

## Calibrate the tracked foot

1. Check at least three measured poses against the CAD view. Drag the view to
   orbit. Tune knee/crank gain and knee offset if the lower leg does not align.
2. Set **Foot X/Y/Z in lower-leg link** to the point to track. Bundled leg meshes
   default to the center of the circular foot, including its thickness; the dot,
   trail and solver all use this point. Other meshes default to an estimated
   distal tip. Adjust these coordinates for the physical contact point.
3. Press **Apply calibration**, then preview joystick movement before live use.

The vertical-cycle generator, centre/stroke/period fields and sequence export
have been removed from this tab. Use the joystick for Cartesian motion and the
TimeLine tab for saved movement sequences.

## Preview sequences and save calibration

**Preview sequence** defaults to
[BackSteps_working.json](../sequences/BackSteps_working.json). It resolves the
referenced snapshots from the repository snapshot directory, filters the selected
leg, applies mirror settings, and interpolates saved fixed-second transitions.
Preview disconnects live feedback so the sources cannot overwrite one another.
Waits and loop settings are respected. Until-reached transitions require hardware
feedback and are rejected for local preview. Speed/torque dynamics are not
simulated; the display represents saved targets, not measured motion.

**Save tuning / Load tuning** preserves the URDF path, leg, mappings,
knee coupling and foot point. Legacy centre/stroke/period and export settings
are ignored when loading older tuning files. **Export single leg URDF** saves
the selected geometry and knee mimic calibration, with absolute mesh paths for
local reuse. Servo mappings and the foot point are stored in tuning separately.
Version-2 tuning files never save limits. When loading old version-1 tuning,
saved lower/upper fields are ignored and current shared limits remain authoritative.
URDF exports contain geometry and coupling, without saved angle stops.

This preview uses the URDF's approximate knee mimic relation. It does not
establish that a physical crank linkage follows the same law. Calibrate knee
coupling and the contact point before judging physical linearity from the model
preview.

## Validation

From the repository root, in a desktop Python environment:

```sh
python -m unittest discover -s desktop/tests -p test_leg_urdf.py
```

These tests cover URDF transforms, servo mappings, Cartesian motion, sequence
preview, tuning, URDF export and feedback cleanup. They validate the model and UI;
physical calibration still requires measurements from the robot.
