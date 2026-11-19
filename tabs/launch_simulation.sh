#!/usr/bin/env bash
set -Eeuo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
if [[ "${1:-}" != --launch ]]; then
  export PYTHONPATH="${workspace_dir}/src/saprog/scripts${PYTHONPATH:+:${PYTHONPATH}}"
  exec python3 -m saprog_desktop.apps.simulation "$@"
fi
shift
source "${workspace_dir}/tabs/_launcher_setup.bash"
saprog_source_ros "${ROS_DISTRO:-jazzy}"
set +u
if [[ -r "${HOME}/saprog-simulation/install/setup.bash" ]]; then
  source "${HOME}/saprog-simulation/install/setup.bash"
elif [[ -r "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
fi
set -u
if ! ros2 pkg prefix saprog_simulation >/dev/null 2>&1; then
  printf 'Build saprog_description and saprog_simulation first; see docs/simulation.md.\n' >&2
  exit 1
fi
if ! ros2 pkg prefix ros_gz_sim >/dev/null 2>&1 || ! command -v gz >/dev/null; then
  printf 'Install Gazebo: sudo apt install ros-jazzy-ros-gz-sim\n' >&2
  exit 1
fi
# No USB preflight, controller, IMU backend, or ROS-to-hardware bridge.
exec ros2 launch saprog_simulation simulation.launch.py "$@"
