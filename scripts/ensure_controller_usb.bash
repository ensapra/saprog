#!/usr/bin/env bash
set -Eeuo pipefail

fail() { printf 'saprog controller USB preflight: %s\n' "$*" >&2; exit 1; }
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "${script_dir}/usb_attach.bash"
saprog_usb_is_wsl || exit 0
[[ -n "${WSL_DISTRO_NAME:-}" ]] || fail "WSL_DISTRO_NAME is empty; launch from the target WSL distribution."

saprog_usb_attach CONTROLLER "${SAPROG_CONTROLLER_USB_BUSID:-}" SAPROG_CONTROLLER_USB_BUSID SAPROG_CONTROLLER_USB_HARDWARE_ID "${SAPROG_CONTROLLER_USB_HARDWARE_ID:-2E24:1125}" || exit 1

shopt -s nullglob
for _ in {1..40}; do
  for device in /dev/input/event* /dev/input/js*; do
    if [[ -c "$device" ]]; then
      device_sys="$(readlink -f "/sys/class/input/${device##*/}/device")"
      while [[ "$device_sys" == /sys/* ]]; do
        if [[ -r "$device_sys/idVendor" && -r "$device_sys/idProduct" ]]; then
          actual_id="$(cat "$device_sys/idVendor"):$(cat "$device_sys/idProduct")"
          if [[ "${actual_id^^}" == "${SAPROG_USB_HARDWARE_ID^^}" ]]; then
            [[ -r "$device" ]] || fail "Input device ${device} exists but is not readable by this user. Add the user to the input group, then restart the WSL session."
            printf '%s\n' "$device"
            exit 0
          fi
          break
        fi
        device_sys="${device_sys%/*}"
      done
    fi
  done
  sleep 0.5
done
fail "USB device ${SAPROG_USB_HARDWARE_ID} was attached, but no matching Linux gamepad input device appeared. Check the xpad/HID driver and input-device permissions."
