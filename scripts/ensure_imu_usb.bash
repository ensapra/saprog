#!/usr/bin/env bash
set -Eeuo pipefail

fail() { printf 'saprog IMU preflight: %s\n' "$*" >&2; exit 1; }
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "${script_dir}/usb_attach.bash"
saprog_usb_is_wsl || exit 0
[[ -n "${WSL_DISTRO_NAME:-}" ]] || fail "WSL_DISTRO_NAME is empty; launch from the target WSL distribution."
saprog_usb_attach IMU "${SAPROG_IMU_USB_BUSID:-}" SAPROG_IMU_USB_BUSID SAPROG_IMU_USB_HARDWARE_ID "${SAPROG_IMU_USB_HARDWARE_ID:-1B4F:9D0F}" || exit 1

shopt -s nullglob
for _ in {1..40}; do
  for device in /dev/ttyACM* /dev/ttyUSB*; do
    device_sys="$(readlink -f "/sys/class/tty/${device##*/}/device")"
    matched=0
    while [[ "$device_sys" == /sys/* ]]; do
      if [[ -r "$device_sys/idVendor" && -r "$device_sys/idProduct" ]]; then
        actual_id="$(cat "$device_sys/idVendor"):$(cat "$device_sys/idProduct")"
        [[ "${actual_id^^}" != "${SAPROG_USB_HARDWARE_ID^^}" ]] || matched=1
        break
      fi
      device_sys="${device_sys%/*}"
    done
    if [[ "$matched" -eq 1 && -c "$device" ]]; then
      printf '%s\n' "$device"
      exit 0
    fi
  done
  sleep 0.5
done
fail "USB device ${SAPROG_USB_HARDWARE_ID} was attached, but its serial port did not appear."
