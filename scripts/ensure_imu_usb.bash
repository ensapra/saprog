#!/usr/bin/env bash
set -Eeuo pipefail

fail() { printf 'saprog IMU preflight: %s\n' "$*" >&2; exit 1; }
is_wsl=0
if [[ -n "${WSL_DISTRO_NAME:-}" ]] || { [[ -r /proc/sys/kernel/osrelease ]] && grep -qi microsoft /proc/sys/kernel/osrelease; }; then
  is_wsl=1
fi
if [[ "$is_wsl" -ne 1 ]]; then
  exit 0
fi
[[ -n "${WSL_DISTRO_NAME:-}" ]] || fail "WSL_DISTRO_NAME is empty; launch from the target WSL distribution."

usbipd=""
if command -v usbipd.exe >/dev/null 2>&1; then
  usbipd="$(command -v usbipd.exe)"
elif [[ -x "/mnt/c/Program Files/usbipd-win/usbipd.exe" ]]; then
  usbipd="/mnt/c/Program Files/usbipd-win/usbipd.exe"
else
  fail "Could not find usbipd.exe. Install usbipd-win and enable WSL interop."
fi

attach_option="--hardware-id"
target="${SAPROG_IMU_USB_HARDWARE_ID:-1B4F:9D0F}"
if [[ -n "${SAPROG_IMU_USB_BUSID:-}" ]]; then
  attach_option="--busid"
  target="$SAPROG_IMU_USB_BUSID"
  [[ "$target" =~ ^[0-9]+-[0-9]+$ ]] || fail "SAPROG_IMU_USB_BUSID must look like 1-2; got '${target}'."
else
  [[ "$target" =~ ^[[:xdigit:]]{4}:[[:xdigit:]]{4}$ ]] || fail "SAPROG_IMU_USB_HARDWARE_ID must look like 1B4F:9D0F; got '${target}'."
  target="${target^^}"
fi

if ! listing="$("$usbipd" list 2>&1)"; then
  fail "usbipd.exe list failed: ${listing}"
fi
listing="${listing//$'\r'/}"
if [[ "$attach_option" == "--busid" ]]; then
  matches="$(awk -v id="$target" '$1 == id && $1 ~ /^[0-9]+-[0-9]+$/ { print $1 " " $2 " " $NF }' <<<"$listing")"
else
  matches="$(awk -v id="$target" 'toupper($2) == id && $1 ~ /^[0-9]+-[0-9]+$/ { print $1 " " $2 " " $NF }' <<<"$listing")"
fi
match_count="$(awk 'NF { count++ } END { print count + 0 }' <<<"$matches")"
[[ "$match_count" -eq 1 ]] || {
  if [[ "$match_count" -eq 0 ]]; then
    fail "No connected IMU matches ${target}. Check its USB connection and device ID."
  else
    fail "More than one device matches ${target}. Set SAPROG_IMU_USB_BUSID to the intended device's BUSID."
  fi
}
read -r bus_id hardware_id state <<<"$matches"
if [[ "$state" != "Shared" && "$state" != "Attached" ]]; then
  fail "IMU USB device ${bus_id} (${hardware_id}) is not shared with WSL (state: ${state}). In an elevated Windows PowerShell, run: usbipd bind --busid ${bus_id}"
fi
if [[ "$state" == "Shared" ]]; then
  if ! attach_output="$("$usbipd" attach --wsl "$WSL_DISTRO_NAME" "$attach_option" "$target" 2>&1)"; then
    fail "usbipd could not attach ${hardware_id} (${bus_id}) to WSL ${WSL_DISTRO_NAME}: ${attach_output}"
  fi
fi

shopt -s nullglob
for _ in {1..40}; do
  for device in /dev/ttyACM* /dev/ttyUSB*; do
    device_sys="$(readlink -f "/sys/class/tty/${device##*/}/device")"
    matched=0
    while [[ "$device_sys" == /sys/* ]]; do
      if [[ -r "$device_sys/idVendor" && -r "$device_sys/idProduct" ]]; then
        actual_id="$(cat "$device_sys/idVendor"):$(cat "$device_sys/idProduct")"
        [[ "${actual_id^^}" != "${hardware_id^^}" ]] || matched=1
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
fail "USB device ${hardware_id} was attached, but its serial port did not appear."
