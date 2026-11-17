#!/usr/bin/env bash
set -Eeuo pipefail

fail() { printf 'saprog launcher: %s\n' "$*" >&2; exit 1; }
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
mode="${1:?usage: _launch_window.bash build|backend|window [executable]}"
shift
window_name=""
if [[ "$mode" == window ]]; then
  window_name="${1:-}"
  [[ -n "$window_name" ]] || fail "Window executable is required."
  shift
fi
ros_distro="${ROS_DISTRO:-jazzy}"
ros_setup="/opt/ros/${ros_distro}/setup.bash"
workspace_setup="${workspace_dir}/install/setup.bash"
config_file="${workspace_dir}/src/saprog/config/saprog.yaml"

[[ "$mode" == build || "$mode" == backend || "$mode" == window ]] || fail "Unknown mode: ${mode}"
[[ -r "$ros_setup" ]] || fail "ROS setup not found: ${ros_setup}. Install ROS 2 ${ros_distro} or set ROS_DISTRO."
set +u
source "$ros_setup" || fail "Could not source ${ros_setup}."
set -u
if [[ "$mode" == build ]]; then
  command -v colcon >/dev/null 2>&1 || fail "colcon is required to build the workspace. Install python3-colcon-common-extensions."
  command -v flock >/dev/null 2>&1 || fail "flock is required to serialize workspace builds. Install util-linux."
  printf 'saprog launcher: Building all workspace packages for Home.\n' >&2
  (cd -- "$workspace_dir" && flock -x 9 && colcon build) 9>"${workspace_dir}/.saprog-build.lock" || fail "Workspace build failed; Home will not start with stale installed windows."
  exit 0
fi
if [[ -z "${SAPROG_HOME_BACKEND_PID:-}" ]]; then
  bash "${workspace_dir}/scripts/ensure_usb.bash" "$config_file"
fi
set +u
[[ -r "$workspace_setup" ]] || fail "Workspace is not built: ${workspace_setup} is missing. Build this workspace with colcon first."
source "$workspace_setup" || fail "Could not source ${workspace_setup}. Rebuild the workspace if its install setup is incomplete."
set -u
[[ -r "$config_file" ]] || fail "Robot config not found: ${config_file}."

if ! package_prefix="$(ros2 pkg prefix saprog 2>/dev/null)"; then
  fail "ROS package 'saprog' is unavailable. Build this workspace and source its install/setup.bash."
fi
workspace_install="$(realpath -m -- "${workspace_dir}/install")"
resolved_prefix="$(realpath -m -- "$package_prefix")"
case "${resolved_prefix}/" in
  "${workspace_install}"/*) ;;
  *) fail "ROS resolved saprog outside this workspace (${package_prefix}). Build and source this workspace." ;;
esac
nodes_exe="${package_prefix}/lib/saprog/saprog_nodes"
[[ -x "$nodes_exe" ]] || fail "Installed backend is missing or not executable: ${nodes_exe}. Build with colcon."

if [[ "$mode" == window ]]; then
  [[ "$window_name" =~ ^[a-zA-Z0-9_]+$ ]] || fail "Invalid window executable name: ${window_name}"
  window_exe="${package_prefix}/lib/saprog/${window_name}"
  [[ -x "$window_exe" ]] || fail "Installed window is missing or not executable: ${window_exe}. Build with colcon."
fi

backend_pid=""
readiness_pid=""
window_pid=""
home_pid=""
owns_backend=0
cleanup() {
  local result=$?
  trap - EXIT INT TERM
  if [[ -n "$readiness_pid" ]] && kill -0 "$readiness_pid" 2>/dev/null; then
    kill "$readiness_pid" 2>/dev/null || true
    wait "$readiness_pid" 2>/dev/null || true
  fi
  if [[ -n "$window_pid" ]] && kill -0 "$window_pid" 2>/dev/null; then
    kill "$window_pid" 2>/dev/null || true
    wait "$window_pid" 2>/dev/null || true
  fi
  if [[ -n "$home_pid" ]] && kill -0 "$home_pid" 2>/dev/null; then
    kill "$home_pid" 2>/dev/null || true
    wait "$home_pid" 2>/dev/null || true
  fi
  if [[ "$owns_backend" -eq 1 && -n "$backend_pid" ]] && kill -0 "$backend_pid" 2>/dev/null; then
    kill "$backend_pid" 2>/dev/null || true
    wait "$backend_pid" 2>/dev/null || true
  fi
  exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ -n "${SAPROG_HOME_BACKEND_PID:-}" ]]; then
  backend_pid="$SAPROG_HOME_BACKEND_PID"
  kill -0 "$backend_pid" 2>/dev/null || fail "Home controller process ${backend_pid} is no longer running."
else
  "$nodes_exe" --ros-args --params-file "$config_file" &
  backend_pid=$!
  owns_backend=1
fi

# A successful parameter response follows controller initialization and executor startup.
controller_ready=0
for _ in {1..150}; do
  if ! kill -0 "$backend_pid" 2>/dev/null; then
    if [[ "$owns_backend" -eq 1 ]]; then
      if wait "$backend_pid"; then
        fail "saprog_nodes exited during startup. Check ROS output and robot serial configuration."
      else
        result=$?
        fail "saprog_nodes failed during startup (exit ${result}). Check ROS output and robot serial configuration."
      fi
    else
      fail "Home controller process ${backend_pid} exited during startup. Check ROS output and robot serial configuration."
    fi
  fi
  if [[ -z "$readiness_pid" ]]; then
    ros2 param get /saprog_position_controller servo_ids >/dev/null 2>&1 &
    readiness_pid=$!
  elif ! kill -0 "$readiness_pid" 2>/dev/null; then
    if wait "$readiness_pid"; then
      readiness_pid=""
      controller_ready=1
      break
    fi
    readiness_pid=""
  fi
  sleep 0.2
done
[[ "$controller_ready" -eq 1 ]] || fail "Controller did not become ready within 30 seconds. Check saprog_nodes output and ROS communication."

if [[ "$mode" == backend ]]; then
  if wait "$backend_pid"; then backend_result=0; else backend_result=$?; fi
  backend_pid=""
  exit "$backend_result"
else
  case "$window_name" in
    snapshot_window|snapshot_player)
      mkdir -p -- "${workspace_dir}/snapshots"
      set -- --directory "${workspace_dir}/snapshots" "$@"
      ;;
  esac
  "$window_exe" "$@" &
  window_pid=$!
  if wait "$window_pid"; then window_result=0; else window_result=$?; fi
  window_pid=""
  exit "$window_result"
fi
