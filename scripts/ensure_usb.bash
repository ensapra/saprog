#!/usr/bin/env bash
set -Eeuo pipefail

fail() { printf 'saprog USB preflight: %s\n' "$*" >&2; exit 1; }
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
if [[ -c "$serial_device" ]]; then
  exit 0
fi

is_wsl=0
if [[ -n "${WSL_DISTRO_NAME:-}" ]] || { [[ -r /proc/sys/kernel/osrelease ]] && grep -qi microsoft /proc/sys/kernel/osrelease; }; then
  is_wsl=1
fi
if [[ "$is_wsl" -ne 1 ]]; then
  fail "Configured serial device ${serial_device} is missing. Connect the robot adapter and check its configured device path and permissions."
fi
[[ -n "${WSL_DISTRO_NAME:-}" ]] || fail "WSL_DISTRO_NAME is empty. Launch this from the target WSL distribution."

usbipd=""
if command -v usbipd.exe >/dev/null 2>&1; then
  usbipd="$(command -v usbipd.exe)"
elif [[ -x "/mnt/c/Program Files/usbipd-win/usbipd.exe" ]]; then
  usbipd="/mnt/c/Program Files/usbipd-win/usbipd.exe"
else
  fail "Could not find usbipd.exe. Install usbipd-win on Windows and enable WSL interop."
fi

attach_option="--hardware-id"
target="${SAPROG_USB_HARDWARE_ID:-0403:6014}"
if [[ -n "${SAPROG_USB_BUSID:-}" ]]; then
  attach_option="--busid"
  target="$SAPROG_USB_BUSID"
  [[ "$target" =~ ^[0-9]+-[0-9]+$ ]] || fail "SAPROG_USB_BUSID must look like 1-2; got '${target}'."
else
  [[ "$target" =~ ^[[:xdigit:]]{4}:[[:xdigit:]]{4}$ ]] || fail "SAPROG_USB_HARDWARE_ID must look like 0403:6014; got '${target}'."
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
    fail "No connected USB device matches ${target}. Check the adapter and USB override."
  else
    fail "More than one connected USB device matches ${target}. Set SAPROG_USB_BUSID to the intended device's BUSID."
  fi
}
read -r bus_id hardware_id state <<<"$matches"
if [[ "$state" != "Shared" && "$state" != "Attached" ]]; then
  fail "USB device ${bus_id} (${hardware_id}) is not shared with WSL (state: ${state}). In an elevated Windows PowerShell, run: usbipd bind --busid ${bus_id}"
fi

if [[ "$state" == "Shared" ]]; then
  if ! attach_output="$("$usbipd" attach --wsl "$WSL_DISTRO_NAME" "$attach_option" "$target" 2>&1)"; then
    fail "usbipd could not attach ${hardware_id} (${bus_id}) to WSL ${WSL_DISTRO_NAME}: ${attach_output}. Ensure it is shared; if not, run 'usbipd bind --busid ${bus_id}' in an elevated Windows PowerShell."
  fi
fi

for _ in {1..40}; do
  [[ -c "$serial_device" ]] && exit 0
  sleep 0.5
done
fail "USB ${hardware_id} (${bus_id}) was attached, but ${serial_device} did not appear as a character device within 20 seconds. Check the serial adapter, Linux USB driver, and device_name in ${config_file}."
