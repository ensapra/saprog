#!/usr/bin/env bash
set -Eeuo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
set +u
source "/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
source "${workspace_dir}/install/setup.bash"
set -u
if [[ "${SAPROG_IMU_EXTERNAL_BACKEND:-}" == 1 ]]; then
  exec ros2 run saprog imu_monitor "$@"
fi
exec ros2 launch saprog imu.launch.py "$@"
