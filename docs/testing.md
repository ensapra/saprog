# Validation suites

The Python tests are split by the dependencies and behavior they exercise.
Run commands from `src/saprog/test` after activating the Python environment
used by the ROS Jazzy workspace.

## Pure Python

These tests do not need sourced ROS, a display server, or robot hardware:

```sh
PYTHONPATH=../scripts python3 -m unittest \
  test_player_settings test_snapshot_units test_monitor_models test_playback_adapters
```

The `saprog_pure_python_tests` CTest entry runs the same modules from the test
directory. It covers storage preferences, snapshot units and validation,
display-independent monitor models, playback coordination, configuration
decoding, and asynchronous adapter behavior through small fakes.

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
