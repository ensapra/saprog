# Single-leg URDF and vertical motion

The desktop **Single Leg Test** loads
[front_right.urdf](../urdf/urdf/front_right.urdf), a single-leg extraction of
the repository robot with its CAD meshes. Selecting another leg extracts that
branch from the full robot description. **Load URDF** accepts a matching
single-leg file or the full robot. The URDF preview needs only the desktop
dependencies. ROS and a successful EEPROM limit read are required for motion
preview/export; manual model inspection is available offline.
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

## Tune linear up/down motion

1. Check at least three measured poses against the CAD view. Drag the view to
   orbit. Tune knee/crank gain and knee offset if the lower leg does not align.
2. Set **Foot X/Y/Z in lower-leg link** to the physical contact point. Defaults
   estimate the distal mesh tip; they are not measured contact coordinates.
3. Pick a comfortable servo pose and press **Use current foot as centre**.
4. Start with **Total up/down stroke = 10 mm**, **Cycle period = 4 s**, then
   press **Preview up / down**. The solver coordinates all three servos, holds
   base-frame X/Y constant and varies Z sinusoidally. Centre Z is the midpoint;
   the two extremes are centre Z ± half the stroke. This is spatially vertical
   motion with smooth reversals, not constant vertical speed.
5. Move **Centre X/Y/Z**, reduce stroke, or change the seed pose if the entire
   cycle cannot be reached. All 81 waypoints are checked before preview begins,
   with a 0.1 mm Cartesian tolerance and current hardware limits. Dashed white
   shows the requested path; blue shows the model foot trail. Servo interpolation
   can introduce a small deviation between the checked waypoints.

| Parameter | Starting value / purpose |
| --- | --- |
| Per-joint gain / offset | 1 / 0 rad; align the received angles to the model |
| Per-servo lower / upper | Read-only current EEPROM limits, shared with Static Animator |
| Knee / crank gain | -2.4 for right legs (13/23); 2.4 for left legs (33/43), from the URDF |
| Knee offset | 0.527179134242642 rad for right legs; -2.112820865757358 rad for left legs |
| Foot local X/Y/Z | Estimated mesh tip; replace with measured contact point |
| Centre X/Y/Z | Capture from the current foot; coordinates are in the fixed base frame |
| Stroke / period | 10 mm total / 4 s per cycle |
| Export speed / torque | AX 100 / 300; hardware playback settings, not preview dynamics |

**Preview sequence** defaults to
[BackSteps_working.json](../sequences/BackSteps_working.json). It resolves the
referenced snapshots from the repository snapshot directory, filters the selected
leg, applies mirror settings, and interpolates saved fixed-second transitions.
Preview disconnects live feedback so the sources cannot overwrite one another.
Waits and loop settings are respected. Until-reached transitions require hardware
feedback and are rejected for local preview. Speed/torque dynamics are not
simulated; the display represents saved targets, not measured motion.

**Export up/down sequence** saves 80 selected-leg snapshots and a version-2
timeline compatible with TimeLine/Controller playback. The loop duration equals
the requested period; the duplicate closing waypoint is omitted. Review the
sequence and approach the centre pose before playback on hardware. The first
transition begins from the hardware's current pose, which need not be on the
vertical path. Motor speed limits, load and calibration affect actual motion.
The selected AX speed is a ceiling; it must allow the servos to reach each
waypoint in its duration. Other legs are omitted from the exported targets.

**Save tuning / Load tuning** preserves the URDF path, leg, mappings,
knee coupling, foot point and motion parameters. **Export single leg URDF** saves
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

These tests cover URDF transforms, servo mappings, vertical paths, sequence
preview, tuning, export and feedback cleanup. They validate the model and UI;
physical calibration still requires measurements from the robot.
