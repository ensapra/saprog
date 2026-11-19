# Validation suites

## Fast development loop

Keep one persistent build tree in the WSL home directory. Reusing it preserves
the CMake cache and avoids placing generated files in the Windows-mounted
checkout. Run these commands from Ubuntu/WSL at the workspace root:

```sh
cd /mnt/c/Users/pauso/Documents/GitHub/saprog
source /opt/ros/jazzy/setup.bash
colcon build --packages-select saprog --symlink-install \
  --build-base "$HOME/saprog-build/build" \
  --install-base "$HOME/saprog-build/install" \
  --log-base "$HOME/saprog-build/log" \
  --cmake-args -DSAPROG_BUILD_ROBOT_VIEW=OFF
source "$HOME/saprog-build/install/setup.bash"
```

Repeat the build command after C++ or interface changes. Colcon reuses the
existing build tree and rebuilds affected targets; `--symlink-install` also
exposes Python and resource edits without copying them into the install tree.
Use the full default build in `launch_home.sh` when validating the embedded
RViz view or preparing to run the full desktop.

For Python-only changes, skip colcon build and run the narrow suite directly
from `src/saprog/test`:

```sh
cd /mnt/c/Users/pauso/Documents/GitHub/saprog/src/saprog/test
PYTHONPATH=../scripts python3 -m unittest test_monitor_models
```

Replace the test module with the affected module. To run the complete pure
Python layer, use the command below. Once the core build is sourced, the CTest
layers can be run independently as documented here.

```sh
PYTHONPATH=../scripts python3 -m unittest \
  test_player_settings test_snapshot_units test_monitor_models \
  test_playback_adapters test_project_data test_controller
```

The first build in a fresh `$HOME/saprog-build` tree is still a full compile.
Keep that tree between sessions; remove it only when intentionally starting
from a clean build. Add `-DCMAKE_CXX_COMPILER_LAUNCHER=ccache` to the CMake
arguments if `ccache` is installed and you want compiler-result caching across
separate build trees.

The Python tests are split by the dependencies and behavior they exercise.
Run commands from `src/saprog/test` after activating the Python environment
used by the ROS Jazzy workspace.

## Pure Python

These tests do not need sourced ROS, a display server, or robot hardware:

```sh
PYTHONPATH=../scripts python3 -m unittest \
  test_player_settings test_snapshot_units test_monitor_models test_playback_adapters \
  test_project_data
```

The `saprog_pure_python_tests` CTest entry runs the same modules from the test
directory. It covers storage preferences, snapshot units and validation,
display-independent monitor models, playback coordination, configuration
decoding, and asynchronous adapter behavior through small fakes.
It also validates all checked-in snapshots and resolves each checked-in
timeline sequence against the snapshot directory.

## Process and platform checks

`test.test_robot_view` checks URDF mappings and orientation freshness on any
platform. Renderer process-group lifecycle tests run on POSIX; they are skipped
on platforms without `os.killpg`. Run them with:

```sh
PYTHONPATH=../scripts python3 -m unittest test_robot_view
```

## ROS, Tk, and native checks

Source the ROS Jazzy and built workspace setup files before running the
`saprog_ros_tk_python_tests` CTest entry. It covers ROS client integration,
Tk presenters and timeline behavior. Tk construction tests need an available
display. The `saprog_process_python_tests` entry exercises renderer process
lifecycle with fake children and requires POSIX process-group support.

The remaining CTest entries build C++ tests against ROS and message packages.
Configure `SAPROG_BUILD_ROBOT_VIEW=OFF` to build the core package without the
RViz, Qt, and X11 embedding target; the default remains ON for the full desktop
build. A successful pure Python run does not establish a Jazzy build, display
embedding, serial behavior, or live-hardware timing.

## Servo response latency

The virtual AX serial bus test runs the real controller on a pseudo-terminal,
in an isolated ROS domain, without accessing robot hardware. After the core
build above:

```sh
python3 src/saprog/test/test_readback_latency.py \
  "$HOME/saprog-build/install/saprog/lib/saprog/saprog_nodes" \
  src/saprog/config/saprog.yaml
```

It checks full-sweep frequency, position-command torque enable and torque-off
service/telemetry latency with delayed serial replies. Its timings describe the
virtual bus; USB/IP, disconnected servos and physical motion need separate live
measurement.
