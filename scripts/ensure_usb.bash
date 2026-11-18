#!/usr/bin/env bash
set -Eeuo pipefail

fail() { printf 'saprog USB preflight: %s\n' "$*" >&2; exit 1; }
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "${script_dir}/usb_attach.bash"
config_file="${1:?Usage: ensure_usb.bash PATH_TO_SAPROG_YAML}"
[[ -r "$config_file" ]] || fail "Robot config is not readable: ${config_file}."
serial_device="$(awk '
  /^[[:space:]]*device_name:[[:space:]]*/ {
    sub(/^[[:space:]]*device_name:[[:space:]]*/, "")
    sub(/[[:space:]]+#.*/, "")
    sub(/[[:space:]]+$/, "")
    if ($0 ~ /^".*"$/ || $0 ~ /^\047.*\047$/) $0 = substr($0, 2, length($0) - 2)
    sub(/[[:space:]]+$/, "")
    print
    exit
  }
' "$config_file")"
[[ -n "$serial_device" ]] || fail "No device_name is configured in ${config_file}."
if [[ -c "$serial_device" ]]; then exit 0; fi

saprog_usb_is_wsl || fail "Configured serial device ${serial_device} is missing. Connect the robot adapter and check its configured device path and permissions."
[[ -n "${WSL_DISTRO_NAME:-}" ]] || fail "WSL_DISTRO_NAME is empty. Launch this from the target WSL distribution."
saprog_usb_attach servo "${SAPROG_USB_BUSID:-}" SAPROG_USB_BUSID SAPROG_USB_HARDWARE_ID "${SAPROG_USB_HARDWARE_ID:-0403:6014}" || exit 1
for _ in {1..40}; do
  [[ -c "$serial_device" ]] && exit 0
  sleep 0.5
done
fail "USB ${SAPROG_USB_HARDWARE_ID} (${SAPROG_USB_BUS_ID}) was attached, but ${serial_device} did not appear as a character device within 20 seconds. Check the serial adapter, Linux USB driver, and device_name in ${config_file}."
