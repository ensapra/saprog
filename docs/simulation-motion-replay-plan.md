# Simulation motion replay implementation plan

Status: Stage 1 started. Explicit versioned offline bounds profiles, visible
provenance, supervisor validation and per-session resolved input capture are
implemented. The Simulation tab no longer requests live EEPROM limits. Full
calibration profiles, configurable initial joint poses, body support and the
remaining Stage 1 exit checks are pending. The existing simulation still provides
passive physics; actuation and replay below remain intended implementation.

The first objective is to replay existing Saprog snapshots and timelines in
Gazebo through simulated AX-18A servos. Replay must preserve the saved motion
settings while allowing gravity, actuator limits and ground contact to determine
the actual movement. The first release succeeds when an existing timeline runs
without editing its JSON and either completes or reports a specific tracking
failure.

## Agreed scope and physical assumptions

| Item | Starting point |
| --- | --- |
| Robot | Four legs, twelve driven servo joints and four coupled knee joints |
| Geometry and mass properties | Use the full [robot URDF](../urdf/urdf/URDF.urdf), including link masses, centres of mass and inertia tensors; total model mass is approximately 1.39 kg |
| Actuators | DYNAMIXEL AX-18A |
| Power | Cable supply providing nominal 12 V with a combined 12 A current rating for the servo bus |
| Structure | Plastic robot components, using the existing rigid link model |
| Foot contact | Assume rubber feet against a high-friction rubber floor |
| Knee transmission | Retain the calibrated URDF mimic relationship for the first release |
| Physics environment | Extend the existing Gazebo Harmonic integration through ROS 2 Jazzy and WSL |
| Motion input | Existing snapshots and timelines, including recorded motions saved in these formats |

There is no battery. The first release uses constant 12 V and does not simulate
discharge. The 12 A rating belongs to the whole supply, not each motor. Supply
loading is initially an approximate diagnostic; a realistic current limit or
voltage-drop model needs measurements of the supply and wiring. Forces from the
power cable are outside the initial model.

The supplied mass properties are the baseline. Do not add twelve servo masses
on top without first establishing whether they are already included in the link
masses.

Autonomous gait generation, balance control, reinforcement learning, complex
terrain and detailed thermal or electrical simulation are later extensions.
Existing motions may fall over in a physically simulated environment; successful
replay means executing their intended commands and reporting the result, not
guaranteeing balance.

## Existing foundation and missing capabilities

| Responsibility | Existing implementation | Required extension |
| --- | --- | --- |
| Leg geometry and kinematics | [SingleLegModel](../desktop/tabs/leg_test/urdf_model.py) supplies transforms, foot position, Jacobian and inverse kinematics | Share the calibrated robot definition with simulation and verify all four branches |
| Robot description | [Robot assets](../urdf/README.md) provide meshes, inertial data, axes and knee coupling | Validate imported collision geometry and coupling under contact load |
| World creation | [World preparation](../ros2/saprog_simulation/saprog_simulation/world.py) imports URDF and adds gravity and ground | Add actuator configuration, initial joint pose, contact settings and a body support option |
| Simulator lifecycle | [GazeboSession](../ros2/saprog_simulation/saprog_simulation/session.py) owns server, viewer, pause and reset | Expose joint feedback and replay state; reset all controller and replay state together |
| ROS supervision | [SimulationNode](../ros2/saprog_simulation/saprog_simulation/node.py) accepts lifecycle commands | Add isolated motion loading, playback and cancellation operations |
| Motion documents | [Timeline domain](../desktop/documents/timeline_domain.py) and [storage](../desktop/documents/timeline_storage.py) validate settings and resolve snapshot references | Reuse these rules without creating another motion file format |
| Playback behavior | [ControllerPlayback](../desktop/tabs/controller/playback.py) sends hardware commands and monitors arrival | Extract reusable scheduling rules; use a simulation backend and simulation clock |
| Desktop interface | [Simulation tab](../desktop/tabs/simulation/__init__.py) supplies lifecycle controls and body metrics | Add motion selection, replay controls and tracking diagnostics |
| Joint bounds | [Servo limits service](../desktop/servo_limits/) requires fresh hardware EEPROM data | Add an explicit simulation-only bounds source for offline operation |

The current simulation retains mimic constraints and supplies passive joints.
It has no servo controller or timeline execution. Its desktop status is designed
for low-frequency display, not feedback control.

The single-leg UI can change servo gains, offsets, knee calibration and the foot
reference. These values must be represented consistently in the simulation
configuration. Loading a URDF alone does not transfer all UI tuning. In
particular, the current single-leg export does not encode the whole servo
mapping calibration or tracked foot point.

## Actuator and contact model

### AX-18A parameters

At 12 V, the [ROBOTIS AX-18A manual](https://emanual.robotis.com/docs/en/dxl/ax/ax-18a/)
specifies 1.8 N m stall torque at 2.2 A and a no-load speed of 97 rpm, approximately
10.16 rad/s. These are endpoint specifications, not simultaneous operating
capabilities. Stall torque is not a continuous torque rating.

Use these values as parameters of an approximate position servo. Apply motor
forces at physics updates, with position feedback, damping and output saturation.
Rate-limit the controller reference according to the saved moving-speed setting.
Do not teleport joints to targets during playback. Direct state initialization
is reserved for creating or resetting an experiment.

Preserve the project's AX speed conversion:

```text
requested_speed_rad_s = raw_speed * 0.111 * 2*pi / 60
effective_speed_limit = min(requested_speed_rad_s, no_load_speed_rad_s)
torque_fraction = raw_torque_strength / 1023
```

The register units and their interpretation are described in the same
[AX-18A control table](https://emanual.robotis.com/docs/en/dxl/ax/ax-18a/#moving-speed-32).
Existing timeline validation accepts speeds from 1 to 1023. If a separate
command API later accepts raw speed zero, implement its maximum-speed meaning
explicitly rather than interpreting it as stopped.

Start with a configurable torque-speed envelope and a separate command torque
cap. A linear envelope between the published endpoints is an engineering
approximation, not a measured AX-18A performance curve. Treat assisting loads,
braking and backdriving separately when validating it. Controller gains and
damping must be tuned against tracking and timestep stability.

Simulated torque is expressed in N m. Hardware joint-state effort currently
represents torque-enabled state, and command effort represents raw torque
strength. Keep these meanings explicit at backend boundaries.

### Knee coupling

Actuate only the twelve motor joints. Let the simulator enforce the four passive
knee constraints with the calibrated multiplier and offset. Verify that contact
forces are transmitted to the driven joints and that passive-joint settings do
not introduce unintended effort limits.

The mimic relationship is a kinematic approximation of the mechanism. It is
acceptable for the initial replay model if its motion and constraint response
remain consistent during the tests below. If it produces systematic errors
under load, implement the actual linkage constraints before using simulated
torque demand to predict hardware performance.

### Ground and collisions

Keep the detailed meshes for appearance. Check the collision shapes produced by
URDF conversion, especially the small foot contact surface and concave parts.
Use simple or decomposed collision shapes where necessary, preserving dimensions
and placement. The tracked foot centre is a kinematic reference, not itself a
collision surface.

Start with a tunable effective friction coefficient of 1.0 and low restitution.
This is an initial high-friction assumption, not a material measurement. Verify
the selected physics backend actually applies the configured contact parameters.
Do not emulate high friction by locking feet to the floor. Specify which
self-collision pairs are active, excluding intended adjacent-part overlaps.

### Supply loading diagnostic

Keep the nominal 12 V assumption independent from the 12 A supply rating. If an
estimated current signal is added, label it as an estimate and report aggregate
demand against 12 A. Do not divide 12 A equally between motors or silently scale
all joint torques to force the estimate below that rating.

The initial replay milestone does not depend on this diagnostic. Actual supply
current limiting, wiring losses and servo shutdown behavior require a later
validated electrical model.

## Proposed runtime architecture

The desktop resolves motion documents into a validated run description and
sends it to the simulation supervisor. Resolve snapshot references on Windows
before sending the run so the WSL process does not need to interpret Windows
snapshot paths. Keep names and original references in run metadata for errors.

The simulation side owns the run clock, current step, controller targets and
arrival decisions. The desktop displays state and sends user actions. A delayed
UI update must not delay actuator control or advance playback incorrectly.

Use three distinct update responsibilities:

1. Physics updates apply servo control and enforce constraints.
2. A scheduler advances replay against simulation time and fresh joint feedback.
3. Throttled status updates drive desktop labels and plots.

Choose a Gazebo controller integration after a small compatibility test with the
existing Bullet Featherstone backend and mimic constraints. A native Gazebo
system or a compatible control integration must support bounded effort and joint
feedback. Keep the existing engine initially; change it only if that test exposes
a concrete limitation. The current 2 Hz status stream cannot close a servo loop.

Continue using the simulation namespace and isolated Gazebo Transport partition.
Simulation playback must never publish to hardware position/torque endpoints or
start the hardware node to obtain feedback. Reuse document rules and scheduling
logic through an explicit backend boundary rather than replacing the manager's
hardware publisher globally.

## Motion replay contract

| Input or action | Required simulation behavior |
| --- | --- |
| Snapshot | Send the named servo targets with explicit speed and torque-strength settings |
| Partial snapshot | Update named joints; preserve other joints' existing target and enable state |
| Mirroring | Apply the existing leg exchange rules before validating destination joint bounds |
| Per-servo overrides | Resolve saved speed and torque settings using the existing control-group rules |
| Fixed-duration transition | Apply targets and advance after the saved number of simulated seconds; do not require arrival |
| Until reached | Advance only after fresh feedback confirms the commanded joints remain within tolerance for the settling interval |
| Wait | Retain controller targets while simulated time elapses |
| Smooth playback | Preserve the current saved-speed ceiling and duration-based speed calculation |
| Loop | Repeat the resolved steps until stopped; apply end behavior only when playback actually finishes |
| End torque on | Hold the last commanded targets after normal completion |
| End torque off | Disable simulated actuators after normal completion |
| Pause | Freeze physics and the replay clock; resume from the same state |
| Stop replay | Cancel future steps and replace active motion targets with measured positions to request a hold; physics and momentum continue |
| Release simulated torque | Cancel replay and remove active motor effort; retain passive mechanical constraints |
| Reset | Cancel replay and restore the configured body pose, joint pose, velocities, controller state and run clock |
| Stop simulation | Terminate owned processes and clear the active run |

The existing controller player intentionally ignores file loop/end-torque flags
and receives repetition from its caller. The simulation's file replay should
honor the saved document flags explicitly; avoid changing hardware controller
behavior as a side effect of sharing code.

Use configurable arrival tolerance, settling duration and timeout, measured in
simulation time. Start with the existing 0.1 rad arrival tolerance as a baseline
and review it against the replay results. A fresh publication acknowledgement
means a command was accepted, not that a joint reached its target.

Use wall-clock deadlines for process startup and transport failures. Keep them
separate from replay deadlines so pausing does not generate an arrival timeout.
Stale feedback or a disconnected simulator must prevent step advancement. Reject
results from cancelled runs with a run identifier or equivalent generation guard.

## Implementation stages

### Stage 1 Define configuration and reproducible starting state

Create a versioned simulation profile containing the robot description,
servo-to-joint calibration, explicit simulation bounds, actuator parameters,
contact parameters, initial body/joint pose and physics settings. Record the
resolved profile and motion inputs with each run.

Simulation bounds must be visibly distinct from live EEPROM limits. Support an
explicitly imported hardware report as one possible source, with provenance;
do not silently reuse an old hardware cache. Keep the live limits service and
hardware publication validation intact. Review existing documentation when this
new offline simulation behavior is implemented.

Provide fixed-body and free-body startup modes. Validate all supplied values,
initial poses, mappings and referenced assets before starting a run. Reject
out-of-bounds motion targets with the servo name and limits; do not silently
rewrite saved motions. Allow initial poses to be set explicitly so the robot
does not have to fall from an arbitrary zero-joint configuration.

Exit condition: the configured model can start, pause and reset offline to the
same initial state, and its calibrated transforms match the single-leg model.

### Stage 2 Prove actuation on one supported leg

Add the controller integration and sufficiently frequent joint feedback. Test
each driven joint independently, then drive all three while checking knee
coupling. Exercise position holds, speed changes, reduced torque and torque
release. Press the foot against a surface to check constrained response.

Use the test to choose control gains and a physics timestep. Start by comparing
the existing 4 ms step with 2 ms and 1 ms; retain the largest step that gives
adequately consistent contact and tracking results for the chosen controller.

Exit condition: loaded and unloaded motion respects the configured actuator
limits, constraint errors stay bounded and smaller timesteps produce consistent
results.

### Stage 3 Execute existing motion documents

Add the resolved run representation and a simulation-time replay scheduler.
Share parsing, mirroring and motion-setting rules with the desktop document
modules. Factor reusable scheduling rules out of the hardware player where
practical, while keeping command transport, feedback and clock dependencies
explicit.

Test snapshots before multi-step timelines. Include partial poses, waits,
overrides, both transition modes, looping, smooth playback and end torque.
Define cancellation and late-result handling before connecting UI controls.

Exit condition: existing motion files execute without conversion or edits, and
pause, timeout, stop and reset have the behavior in the replay contract.

### Stage 4 Add full robot replay controls

Extend the existing Simulation tab with snapshot/timeline selection, initial
pose selection, supported/free-body mode and replay controls. Keep Stop replay,
Release simulated torque and Stop simulation distinguishable.

Show the active file and step, simulated elapsed time, target and measured joint
positions, tracking error, saturation state, body height and tilt. Include foot
contact state where supported. Keep full-rate data inside the simulator and
publish an appropriate subset to the UI.

Exit condition: a user can select an existing timeline, run it, inspect a
failure, reset and repeat without connecting the physical robot.

### Stage 5 Validate free body motion and document operation

Begin with the body supported to isolate motion mapping and timing. Then run
the free robot on the rubber floor. Use existing [stand-up](../sequences/doStandUp.json),
[rest](../sequences/doRest.json) and [recorded stepping](../sequences/FrontSteps_working.json)
timelines after validating their snapshot references and bounds.

Record whether each run completed, timed out, saturated motors, slipped or
fell. Compare representative runs at two physics timesteps and repeat resets
with identical inputs. Expect comparable trajectories within documented
tolerances rather than exact equality across simulator installations.

Update the [simulation operating guide](../ros2/saprog_simulation/README.md) and
[testing guide](testing.md) with the implemented controls, profile format,
commands and measured limitations.

Exit condition: the first release meets the acceptance checks below and provides
enough diagnostics to explain a motion that does not complete.

## Acceptance checks

| Check | Required result |
| --- | --- |
| Existing files | A snapshot and representative saved timelines replay with unchanged source JSON |
| Offline startup | Simulation can run without USB devices or a live hardware limit read |
| Calibration | All four leg transforms agree with the calibrated model at representative joint poses |
| Dynamics | Gravity and contacts remain active; motors obey configured effort and speed limits |
| Timing | Fixed durations and waits track simulated seconds at different real-time factors |
| Arrival | Until-reached steps require fresh feedback and fail clearly when targets cannot be reached |
| Pause | Neither the physical state nor replay deadlines advance while paused |
| Lifecycle | Stop, release torque, end torque and reset each follow the replay contract |
| Reset | Body, joints, controller state and scheduler return to the recorded initial configuration |
| Isolation | Simulation actions produce no hardware publications or hardware node startup |
| Diagnostics | A failure identifies the run, step, affected joints and relevant tracking or transport condition |
| Numerical stability | Representative loaded runs remain consistent when reducing the timestep |

Use pure scheduler and document tests for timing, cancellation and file
semantics. Use desktop tests with fake managers for controls and routing. Extend
the existing [Gazebo smoke tests](../ros2/saprog_simulation/test/test_gazebo_smoke.py)
with live actuation, coupling, feedback and replay scenarios. Keep physics
validation in live tests; mocked successful command responses cannot establish
that the imported mechanism moves correctly.

## Remaining measurements and later improvements

No additional hardware information is required to start the baseline. The
implementation still needs to validate controller compatibility with the
selected physics engine and choose the initial simulation bounds/profile.

Later fidelity improvements can use measured single-leg step responses, loaded
holds, foot friction tests and supply voltage/current recordings. These would
refine servo compliance, torque-speed behavior, contact parameters and shared
supply effects. A closed-linkage model, cable forces and plastic flexibility
should be added only when observed replay errors justify them.
