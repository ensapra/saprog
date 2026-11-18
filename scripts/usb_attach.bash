#!/usr/bin/env bash
# Shared usbipd-win discovery and attach policy. Callers own device selection.

saprog_usb_fail() {
  printf 'saprog %s USB preflight: %s\n' "$1" "$2" >&2
  return 1
}

saprog_usb_is_wsl() {
  [[ -n "${WSL_DISTRO_NAME:-}" ]] || {
    [[ -r /proc/sys/kernel/osrelease ]] && grep -qi microsoft /proc/sys/kernel/osrelease
  }
}

# Arguments: profile label, bus override, bus variable name, hardware variable name, target ID.
# Sets SAPROG_USB_BUS_ID / SAPROG_USB_HARDWARE_ID / SAPROG_USB_STATE.
saprog_usb_attach() {
  local label="$1" bus_var="$2" bus_name="$3" hardware_var="$4" target="$5"
  local usbipd attach_option listing matches count attach_output
  attach_option=--hardware-id
  if [[ -n "$bus_var" ]]; then
    attach_option=--busid
    target="$bus_var"
    [[ "$target" =~ ^[0-9]+-[0-9]+$ ]] || { saprog_usb_fail "$label" "${bus_name} must look like 1-2; got '${target}'."; return 1; }
  else
    [[ "$target" =~ ^[[:xdigit:]]{4}:[[:xdigit:]]{4}$ ]] || { saprog_usb_fail "$label" "${hardware_var} must look like 0403:6014; got '${target}'."; return 1; }
    target="${target^^}"
  fi

  if command -v usbipd.exe >/dev/null 2>&1; then
    usbipd="$(command -v usbipd.exe)"
  elif [[ -x "/mnt/c/Program Files/usbipd-win/usbipd.exe" ]]; then
    usbipd="/mnt/c/Program Files/usbipd-win/usbipd.exe"
  else
    saprog_usb_fail "$label" "Could not find usbipd.exe. Install usbipd-win and enable WSL interop."; return 1
  fi
  if ! listing="$("$usbipd" list 2>&1)"; then
    saprog_usb_fail "$label" "usbipd.exe list failed: ${listing}"; return 1
  fi
  listing="${listing//$'\r'/}"
  if [[ "$attach_option" == --busid ]]; then
    matches="$(awk -v id="$target" '$1 == id && $1 ~ /^[0-9]+-[0-9]+$/ { print $1 " " $2 " " $NF }' <<<"$listing")"
  else
    matches="$(awk -v id="$target" 'toupper($2) == id && $1 ~ /^[0-9]+-[0-9]+$/ { print $1 " " $2 " " $NF }' <<<"$listing")"
  fi
  count="$(awk 'NF { n++ } END { print n + 0 }' <<<"$matches")"
  if [[ "$count" -ne 1 ]]; then
    if [[ "$count" -eq 0 ]]; then
      saprog_usb_fail "$label" "No connected USB device matches ${target}. Check the adapter and USB override."
    else
      saprog_usb_fail "$label" "More than one connected USB device matches ${target}. Set ${bus_name} to the intended device's BUSID."
    fi
    return 1
  fi
  read -r SAPROG_USB_BUS_ID SAPROG_USB_HARDWARE_ID SAPROG_USB_STATE <<<"$matches"
  if [[ "$SAPROG_USB_STATE" != Shared && "$SAPROG_USB_STATE" != Attached ]]; then
    saprog_usb_fail "$label" "USB device ${SAPROG_USB_BUS_ID} (${SAPROG_USB_HARDWARE_ID}) is not shared with WSL (state: ${SAPROG_USB_STATE}). In elevated Windows PowerShell, run: usbipd bind --busid ${SAPROG_USB_BUS_ID}"
    return 1
  fi
  if [[ "$SAPROG_USB_STATE" == Shared ]]; then
    if ! attach_output="$("$usbipd" attach --wsl "$WSL_DISTRO_NAME" "$attach_option" "$target" 2>&1)"; then
      saprog_usb_fail "$label" "usbipd could not attach ${SAPROG_USB_HARDWARE_ID} (${SAPROG_USB_BUS_ID}) to WSL ${WSL_DISTRO_NAME}: ${attach_output}. Ensure it is shared; if not, run 'usbipd bind --busid ${SAPROG_USB_BUS_ID}' in elevated Windows PowerShell."
      return 1
    fi
  fi
}
