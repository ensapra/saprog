#!/usr/bin/env bash
set -Eeuo pipefail

fail() { printf 'saprog launcher: %s\n' "$*" >&2; exit 1; }
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
mode="${1:?usage: _launch_window.bash build|verify|backend|window [executable]}"
shift
window_name=""
if [[ "$mode" == window ]]; then
  window_name="${1:-}"
  [[ -n "$window_name" ]] || fail "Window executable is required."
  shift
fi
ros_distro="${ROS_DISTRO:-jazzy}"
config_file="${workspace_dir}/src/saprog/config/saprog.yaml"
source "${workspace_dir}/tabs/_launcher_setup.bash"

[[ "$mode" == build || "$mode" == verify || "$mode" == backend || "$mode" == window ]] || fail "Unknown mode: ${mode}"
saprog_source_ros "$ros_distro"
if [[ "$mode" == build ]]; then
  command -v colcon >/dev/null 2>&1 || fail "colcon is required to build the workspace. Install python3-colcon-common-extensions."
  command -v flock >/dev/null 2>&1 || fail "flock is required to serialize workspace builds. Install util-linux."
  printf 'saprog launcher: Building all workspace packages for Home.\n' >&2
  (cd -- "$workspace_dir" && flock -x 9 && colcon build) 9>"${workspace_dir}/.saprog-build.lock" || fail "Workspace build failed; Home will not start with stale installed windows."
  exit 0
fi
if [[ "$mode" == verify ]]; then
  saprog_verify_workspace "$workspace_dir" "$config_file"
  exit 0
fi
if [[ -z "${SAPROG_HOME_BACKEND_PID:-}" ]]; then
  bash "${workspace_dir}/scripts/ensure_usb.bash" "$config_file"
fi
saprog_verify_workspace "$workspace_dir" "$config_file" "$window_name"

backend_pid=""
readiness_pid=""
window_pid=""
owns_backend=0
stop_child() {
  local child_pid="$1"
  [[ -n "$child_pid" ]] || return 0
  kill -0 "$child_pid" 2>/dev/null || { wait "$child_pid" 2>/dev/null || true; return 0; }
  kill "$child_pid" 2>/dev/null || true
  for _ in {1..20}; do
    kill -0 "$child_pid" 2>/dev/null || break
    sleep 0.1
  done
  if kill -0 "$child_pid" 2>/dev/null; then kill -KILL "$child_pid" 2>/dev/null || true; fi
  wait "$child_pid" 2>/dev/null || true
}
cleanup() {
  local result=$?
  trap - EXIT
  trap '' INT TERM
  stop_child "$readiness_pid"
  stop_child "$window_pid"
  if [[ "$owns_backend" -eq 1 ]]; then stop_child "$backend_pid"; fi
  exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [[ -n "${SAPROG_HOME_BACKEND_PID:-}" ]]; then
  backend_pid="$SAPROG_HOME_BACKEND_PID"
  saprog_verify_home_backend_pid "$backend_pid"
else
  "$SAPROG_NODES_EXE" --ros-args --params-file "$config_file" &
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
    snapshot_window|snapshot_player|static_animator)
      mkdir -p -- "${workspace_dir}/snapshots"
      set -- --directory "${workspace_dir}/snapshots" "$@"
      ;;
  esac
  "$SAPROG_WINDOW_EXE" "$@" &
  window_pid=$!
  if wait "$window_pid"; then window_result=0; else window_result=$?; fi
  window_pid=""
  exit "$window_result"
fi
