#!/usr/bin/env python3
"""Read and display AX angle limits without modifying hardware or configuration."""

import argparse
import subprocess
from pathlib import Path

import yaml
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "src/saprog/config/saprog.yaml",
    )
    args = parser.parse_args()
    original = args.config.read_text()
    config = yaml.safe_load(original)["/**"]["ros__parameters"]
    device = config["device_name"]
    if subprocess.run(["fuser", device], capture_output=True).returncode == 0:
        raise SystemExit(f"{device} is in use. Close the controller and snapshot windows first.")
    port = PortHandler(device)
    packet = PacketHandler(config["protocol_version"])
    lower, upper = [], []
    try:
        if not port.openPort() or not port.setBaudRate(config["baud_rate"]):
            raise RuntimeError(f"Cannot open {device} at the configured baud rate")
        for servo in config["servo_ids"]:
            data, result, error = packet.readTxRx(port, servo, 6, 4)
            if result != COMM_SUCCESS or error:
                raise RuntimeError(
                    f"servo_{servo}: {packet.getTxRxResult(result) if result != COMM_SUCCESS else packet.getRxPacketError(error)}"
                )
            low, high = data[0] | data[1] << 8, data[2] | data[3] << 8
            if not 0 <= low < high <= 1023:
                raise RuntimeError(
                    f"servo_{servo}: invalid joint limits {low}, {high} (wheel mode or locked range)"
                )
            lower.append(low)
            upper.append(high)
            print(f"servo_{servo}: {low} .. {high}")
    finally:
        port.closePort()
    print("Limits read from hardware; configuration unchanged")


if __name__ == "__main__":
    main()
