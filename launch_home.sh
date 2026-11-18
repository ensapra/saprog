#!/usr/bin/env bash
set -Eeuo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
ros_distro="${ROS_DISTRO:-jazzy}"
set +u
source "/opt/ros/${ros_distro}/setup.bash"
set -u
bash "${workspace_dir}/tabs/_launch_window.bash" build
set +u
source "${workspace_dir}/install/setup.bash"
set -u
exec python3 "${workspace_dir}/src/saprog/scripts/home.py"
