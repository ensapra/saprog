#!/usr/bin/env bash

# Workspace discovery and setup checks shared by the build and runtime modes.
saprog_source_ros() {
  local ros_setup="/opt/ros/${1}/setup.bash"
  [[ -r "$ros_setup" ]] || fail "ROS setup not found: ${ros_setup}. Install ROS 2 $1 or set ROS_DISTRO."
  set +u
  source "$ros_setup" || fail "Could not source ${ros_setup}."
  set -u
}

saprog_verify_workspace() {
  local workspace_dir="$1" config_file="$2" executable_name="${3:-}"
  local workspace_setup="${workspace_dir}/install/setup.bash" package_prefix workspace_install resolved_prefix
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
  SAPROG_PACKAGE_PREFIX="$package_prefix"
  SAPROG_NODES_EXE="${package_prefix}/lib/saprog/saprog_nodes"
  [[ -x "$SAPROG_NODES_EXE" ]] || fail "Installed backend is missing or not executable: ${SAPROG_NODES_EXE}. Build with colcon."
  if [[ -n "$executable_name" ]]; then
    [[ "$executable_name" =~ ^[a-zA-Z0-9_]+$ ]] || fail "Invalid window executable name: ${executable_name}"
    SAPROG_WINDOW_EXE="${package_prefix}/lib/saprog/${executable_name}"
    [[ -x "$SAPROG_WINDOW_EXE" ]] || fail "Installed window is missing or not executable: ${SAPROG_WINDOW_EXE}. Build with colcon."
  fi
}

saprog_verify_home_backend_pid() {
  local pid="$1"
  local expected_launcher="${workspace_dir}/tabs/_launch_window.bash"
  local actual_launcher
  local -a process_args=()
  [[ "$pid" =~ ^[0-9]+$ ]] || fail "Home controller process ID is invalid: ${pid}."
  kill -0 "$pid" 2>/dev/null || fail "Home controller process ${pid} is no longer running."
  [[ -r "/proc/${pid}/cmdline" ]] || fail "Cannot verify Home controller process ${pid}; launch through Home on Linux/WSL."
  mapfile -d '' -t process_args < "/proc/${pid}/cmdline"
  [[ "${#process_args[@]}" -ge 3 && "${process_args[2]}" == backend ]] || fail "Process ${pid} is not this workspace's Home controller launcher. Restart Home before opening this tool."
  actual_launcher="$(realpath -m -- "${process_args[1]}")"
  expected_launcher="$(realpath -m -- "$expected_launcher")"
  [[ "$actual_launcher" == "$expected_launcher" ]] || fail "Process ${pid} is not this workspace's Home controller launcher. Restart Home before opening this tool."
}
