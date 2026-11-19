#!/usr/bin/env bash
set -Eeuo pipefail

fail() { printf 'saprog leg inspector: %s\n' "$*" >&2; exit 1; }
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source "${workspace_dir}/tabs/_launcher_setup.bash"
saprog_source_ros "${ROS_DISTRO:-jazzy}"

# Prefer Home's workspace install. The isolated leg-lab build is also supported.
set +u
if [[ -r "${workspace_dir}/install/setup.bash" ]]; then
  source "${workspace_dir}/install/setup.bash"
fi
set -u
if ! ros2 pkg prefix saprog_leg_lab >/dev/null 2>&1; then
  leg_setup="${HOME}/saprog-leg-lab/install/setup.bash"
  [[ -r "$leg_setup" ]] || fail "Leg inspector is not built. Build saprog_description and saprog_leg_lab, then try again."
  set +u
  source "$leg_setup"
  set -u
fi
ros2 pkg prefix saprog_leg_lab >/dev/null 2>&1 || fail "ROS package saprog_leg_lab is unavailable after sourcing the workspace."

# This tool needs no serial port, torque service, or additional hardware backend.
exec ros2 launch saprog_leg_lab inspect.launch.py "$@"
