# Project refactoring review

Tracking issue: [#7](https://github.com/ensapra/saprog/issues/7).
This is a source review and implementation backlog, not an implemented refactor.
The working tree was clean at the start of review.

## Findings and priorities

The strongest opportunity is to separate state machines and resource ownership
from desktop layout. File counts alone do not identify a good boundary.
Prefer small cohesive modules that expose a useful operation; keep related state
together and avoid one-line wrappers or a generic utilities directory.

| Priority | Observed source | Finding and consequence | Work |
| --- | --- | --- | --- |
| High | [Player](../src/saprog/scripts/saprog_desktop/views/snapshot_player.py), [Creator](../src/saprog/scripts/saprog_desktop/views/snapshot_window.py) | 785/797 lines combine views, configuration, telemetry, commands, file selection and playback. Sending/waiting/releasing state, freshness checks, stability counters and torque callbacks are duplicated. Fixing one path can leave the other inconsistent. | #10, #11, #12 |
| High | [Timeline](../src/saprog/scripts/saprog_desktop/views/timeline_window.py) | 669 lines; inherits Player, overrides its build/readiness/refresh/tick behavior, and obtains action types from another module's runtime globals. Construction brings in Player resources/settings the action editor does not need. | #9, #14, #15 |
| High | [Timeline callbacks](../src/saprog/scripts/saprog_desktop/views/timeline_window.py), [limits client](../src/saprog/scripts/saprog_desktop/ros/snapshot_limits_client.py) | Goal/result/cancel waits and post-refresh parameter fetch lack complete bounds. A timeout must not be mistaken for confirmed cancellation or permission to send conflicting motion. | #10, #15 |
| High | [Home](../src/saprog/scripts/saprog_desktop/apps/home.py), [Animator](../src/saprog/scripts/saprog_desktop/apps/static_animator.py) | Baseline finding addressed by #16: both entry points now use `AnimatorViews`, retain all three child views and wait for Timeline's safe close callback before host teardown. | Completed in #16; root/node/backend ownership remains with each host. |
| High | [Controller](../src/saprog/src/position_controller_node.cpp) | 721 lines mix ROS arbitration, command validation, configuration, telemetry and threaded timeline scheduling. Locking, cancellation and final hold semantics need explicit test seams. Existing #1 identifies duration overflow, final-action deadline/limit checks and runtime-parameter consistency. | #17, #18 |
| Medium | [Bus](../src/saprog/src/dynamixel_bus.cpp) | 434 lines combine SDK/register details, cache verification, batching and priority/deadline arbitration. Share transport/error machinery while preserving transaction ordering and the single serial owner. | #19 |
| Medium | [IMU backend](../src/saprog/src/imu_node.cpp), [filter](../src/saprog/include/saprog/imu_filter.hpp) | 576-line backend owns descriptors, USB discovery/helper process, framing, ROS publication and connection state. Filter header also owns CSV parsing/math. These have distinct test and dependency needs. | #20, #21 |
| Medium | [IMU monitor](../src/saprog/scripts/saprog_desktop/views/imu_monitor.py), [Temperature](../src/saprog/scripts/saprog_desktop/views/temperature_monitor.py) | Rendering/projection, telemetry histories, ROS services and widgets share classes. IMU rendering alone is a large method; extracting pure geometry/history gives useful boundaries. | #22 |
| Medium | [Robot view](../src/saprog/scripts/saprog_desktop/views/robot_view.py), [bridge](../src/saprog/scripts/saprog_desktop/ros/robot_view_bridge.py), [description](../urdf/) | Process-group cleanup/readiness lives in the view; joint mapping and sensor reference/mounting need authoritative contracts. Existing #4 also covers installed ROS 1 export leftovers. | #23 |
| Medium | [CMake](../src/saprog/CMakeLists.txt), [tests](../src/saprog/test/) | Flat helper installation, duplicate program/module copies, test path injection and runtime-global bootstrap complicate moves. RViz/Qt/X11 are unconditional build dependencies. Pure tests are mixed with view/ROS tests. | #8, #9, #25 |
| Medium | [USB helpers](../scripts/), [launchers](../tabs/) | Repeated WSL/usbipd selection/attach logic; shared launch script combines build, verification, readiness and cleanup. Device selection and stdout contracts differ and must stay explicit. | #24 |
| Medium | [Operation](operation.md), [style guide](desktop-style.md) | Baseline layout and ownership documentation was updated with the shared animator composition and deferred shutdown behavior. | Completed in #16. |

Line counts describe this review's baseline, not acceptance limits.
Known behavior defects inherited from #1/#4 are tracked explicitly; extraction
must not silently change their contracts or claim them fixed.

## Target responsibility boundaries

Python imports should flow from app composition into views/presenters and ROS or
storage adapters, with adapters depending on pure domain rules. Domain code must
not import views, ROS, Tk or launchers. Views may depend on shared UI primitives,
but must not borrow another window's private state or initializer.

A distinct Python namespace (for example `saprog_desktop`) should avoid the
generated `saprog.action/msg/srv` namespace. Proposed responsibilities are:

- Domain: snapshot units/validation, motion grouping, timeline rules/editor commands
  and snapshot playback state.
- Storage: atomic JSON, snapshot catalog/references, timeline migration/load/save,
  and Player preferences.
- ROS: configuration, telemetry, torque, command publication and timeline actions.
- UI: shared style/scroll primitives, small forms/lists/charts and presenters.
- App: entry points, root scheduler, component assembly and process ownership.

These are proposed boundaries, not existing import paths. Extract preferences
from Player when its package/runtime boundary moves (#8/#9); retain its current
independent per-window settings. Reuse one authoritative servo-leg grouping
between Creator and Timeline during #13/#14 rather than preserving separate
`LEG_LAYOUT` and `LEGS` membership definitions. Keep placement labels in UI.

C++ should separate pure configuration/command/plan rules, execution and telemetry
contracts, transport/register ownership, and thin ROS adapters. IMU transport,
protocol and estimation are distinct responsibilities; robot embedding remains
a platform boundary. Keep the existing short main/readback/conversion files
cohesive unless a real shared responsibility emerges.

Smaller files should reduce the number of concepts a reader needs at once.
Do not replace straight-line readable code with compressed expressions, layers
that only forward arguments, per-method files or a universal callback framework.
Some repeated syntax is appropriate where lifetime or policy differs.

## Focused implementation issues

Each issue contains source evidence, ordered implementation steps, acceptance
criteria and the same-commit API-map requirement. Existing broad issues remain
open and linked; these are concrete implementation slices, not competing plans.

| Issue | Responsibility | Dependencies / relationship |
| --- | --- | --- |
| [#8](https://github.com/ensapra/saprog/issues/8) | Package desktop code by responsibility and simplify Python installation | Foundation; complete before moving modules in the other issues. |
| [#9](https://github.com/ensapra/saprog/issues/9) | Move ROS/Tk bootstrap and event-loop ownership out of SnapshotPlayer | After Python package foundation; part of #2. |
| [#10](https://github.com/ensapra/saprog/issues/10) | Extract a bounded controller-configuration ROS client shared by desktop tools | After explicit runtime adapters; part of #2. Coordinate backend parameter policy with #1. |
| [#11](https://github.com/ensapra/saprog/issues/11) | Share snapshot playback as a UI-independent coordinator | After configuration and torque adapters; part of #2. |
| [#12](https://github.com/ensapra/saprog/issues/12) | Centralize acknowledged torque operations and pending-request cleanup | After configuration adapter; prerequisite for shared playback; part of #2. |
| [#13](https://github.com/ensapra/saprog/issues/13) | Separate timeline documents, reference storage and action serialization | After Python package foundation; preserve completed #3. |
| [#14](https://github.com/ensapra/saprog/issues/14) | Replace TimelineWindow inheritance with a focused editor presenter and small views | After runtime/configuration, timeline document and action client extractions; part of #2. |
| [#15](https://github.com/ensapra/saprog/issues/15) | Extract a bounded timeline action client with explicit cancellation states | After runtime and pure action serializer; part of #2. |
| [#16](https://github.com/ensapra/saprog/issues/16) | Reuse animator composition and coordinate desktop shutdown | After runtime and view/client lifecycle APIs; coordinate with active #6. |
| [#17](https://github.com/ensapra/saprog/issues/17) | Extract a deterministic timeline executor from the controller node | Focused implementation of #1; after reusable command/config boundaries are agreed. |
| [#18](https://github.com/ensapra/saprog/issues/18) | Separate validated servo commands and configuration from ROS callbacks | Focused implementation of #1; coordinate with timeline executor and desktop configuration client. |
| [#19](https://github.com/ensapra/saprog/issues/19) | Separate DYNAMIXEL register operations from bus arbitration and command batches | Coordinate executor bus interface; no second serial-port owner. |
| [#20](https://github.com/ensapra/saprog/issues/20) | Extract IMU serial transport, USB discovery and helper-process lifetime | Focused implementation of #5. |
| [#21](https://github.com/ensapra/saprog/issues/21) | Split IMU sample decoding, quaternion math and estimation contracts | After IMU transport boundaries; focused implementation of #5, separate from optional library adoption. |
| [#22](https://github.com/ensapra/saprog/issues/22) | Split monitor rendering from telemetry state and ROS requests | After runtime lifecycle extraction; use existing style helpers. |
| [#23](https://github.com/ensapra/saprog/issues/23) | Organize robot-view process ownership and authoritative robot mappings | After process/runtime ownership APIs; concrete implementation of #4. |
| [#24](https://github.com/ensapra/saprog/issues/24) | Deduplicate USB setup and clarify launcher/backend process ownership | After application/process lifetime policies; preserve launcher CLI behavior. |
| [#25](https://github.com/ensapra/saprog/issues/25) | Separate validation layers and make refactor checks work without full GUI dependencies | Begin characterization before extraction; finalize after packaging and module migrations. |

## Suggested execution order

1. Start #25's behavior characterization and #8's package boundary. Move existing
   useful helpers intact and preserve source/installed execution.
2. Establish #9 runtime ownership, #10 configuration and #12 torque; then #11
   shared playback. #13 document/storage separation can proceed independently.
3. Complete #15 action lifecycle and #14 editor composition, then #16 animator
   assembly/shutdown. Keep legacy migration regression tests from completed #3.
4. Agree #18 command/config and #19 bus contracts before extracting #17 executor;
   finish #1's safety regressions in those implementations.
5. Extract #20 IMU transport, then #21 protocol/estimation. #22 monitor rendering
   can follow runtime ownership independently of backend extraction.
6. Complete #23 robot boundaries under existing #4 and #24 launcher ownership.
   Finish #25's clean-install/build/integration checks across migrated modules.

Every issue is independently reviewable after its prerequisites. Update the
[reuse map](reusability.md) with actual names as each slice lands; do not document
proposed symbols as already available.

## Existing work and responsibilities to retain

[#3](https://github.com/ensapra/saprog/issues/3) is closed: current legacy migration
compares normalized poses, rejects mismatched/missing matches, and chooses stable
order among identical-pose files. Preserve it; do not propose the old name-only
bug as still present. [#6](https://github.com/ensapra/saprog/issues/6) remains
open for the embedded Static Animator composition and Creator / Player workflow.

[#1](https://github.com/ensapra/saprog/issues/1),
[#2](https://github.com/ensapra/saprog/issues/2),
[#4](https://github.com/ensapra/saprog/issues/4) and
[#5](https://github.com/ensapra/saprog/issues/5) remain broader existing work.
Library replacement evaluation in #5 is separate from structural extraction;
shorter code alone does not justify a fusion or trajectory-stack change.

Already useful boundaries include
[snapshot_units](../src/saprog/scripts/saprog_desktop/snapshot_units.py),
[snapshot_files](../src/saprog/scripts/saprog_desktop/snapshot_files.py),
[snapshot_arrival](../src/saprog/scripts/saprog_desktop/snapshot_arrival.py),
[desktop_style](../src/saprog/scripts/saprog_desktop/desktop_style.py),
[motion_controls](../src/saprog/scripts/saprog_desktop/motion_controls.py) and
[servo_units](../src/saprog/include/saprog/servo_units.hpp).
Reuse and relocate these rather than creating replacements.

## Preservation and validation limits

Preserve the behavior documented in [README](README.md) and
[operation](operation.md): one servo bus, serialized ROS callbacks plus worker
locks, radians/measured limits/calibration, startup torque unchanged, command
publication distinct from arrival, snapshot stable dwell distinct from timeline
arrival, cancellation unable to undo transmitted commands, atomic JSON, offline
editing and backend-owned IMU state. UI work follows
[desktop-style](desktop-style.md).

This pass inspected source, manifests, test structure, launchers and existing
GitHub issues. It did not execute Python tests, a C++ build, CTest, a GUI or robot
hardware. The earlier results in [README](README.md) are historical, not new
evidence. Each implementation issue specifies meaningful validation for its
boundary, including explicit Linux/WSL/Jazzy, X11 and hardware checks where
needed. Documentation changes in this pass require link/content/diff checks,
not a runtime regression claim.
