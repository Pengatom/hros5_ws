#!/usr/bin/env python3
"""
dxl_scan_bus.py

Sweep a Dynamixel bus for responding IDs across baudrates and protocol versions.

Example:
  python3 dxl_scan_bus.py --port /dev/dxl --start-id 1 --end-id 25 --baud 57600 1000000 --protocols 1.0 2.0
"""

import argparse
from typing import Iterable, List, Tuple

from dynamixel_sdk import (
    COMM_RX_TIMEOUT,
    COMM_SUCCESS,
    PacketHandler,
    PortHandler,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan a Dynamixel bus for active IDs.")
    parser.add_argument("--port", "--device", dest="port", default="/dev/dxl", help="Serial device path (default: /dev/dxl)")
    parser.add_argument("--baud", nargs="+", type=int, default=[57600, 1000000], help="Baudrates to try (default: 57600 1000000)")
    parser.add_argument("--protocols", nargs="+", type=float, choices=[1.0, 2.0], default=[1.0, 2.0], help="Protocol versions to try (default: 1.0 2.0)")
    parser.add_argument("--start-id", type=int, default=1, help="Starting ID to scan (inclusive, default: 1)")
    parser.add_argument("--end-id", type=int, default=24, help="Ending ID to scan (inclusive, default: 24)")
    parser.add_argument("--exhaustive", action="store_true", help="Try all baud/protocol combos even after devices are found.")
    return parser.parse_args()


def scan_bus(port: PortHandler, ids: Iterable[int], protocol_version: float, baudrate: int) -> List[Tuple[int, int, float, int]]:
    packet = PacketHandler(protocol_version)
    found = []

    if not port.setBaudRate(baudrate):
        print(f"⚠️ Failed to set baudrate: {baudrate}")
        return found

    print(f"\n🔍 Scanning with Protocol {protocol_version} at {baudrate} bps...")
    for dxl_id in ids:
        dxl_model_number, dxl_comm_result, dxl_error = packet.ping(port, dxl_id)
        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
            print(f"✅ Found ID {dxl_id} | Model: {dxl_model_number}")
            found.append((dxl_id, dxl_model_number, protocol_version, baudrate))
        elif dxl_comm_result != COMM_RX_TIMEOUT:
            print(f"⚠️ ID {dxl_id} error: {packet.getTxRxResult(dxl_comm_result)}")

    return found


def main() -> int:
    args = parse_args()
    scan_ids = range(args.start_id, args.end_id + 1)

    port = PortHandler(args.port)
    if not port.openPort():
        print(f"❌ Failed to open port: {args.port}")
        return 1

    all_found: List[Tuple[int, int, float, int]] = []

    for protocol in args.protocols:
        for baud in args.baud:
            found = scan_bus(port, scan_ids, protocol, baud)
            if found:
                all_found.extend(found)
                if not args.exhaustive:
                    break
        if all_found and not args.exhaustive:
            break

    if not all_found:
        print("\n🚫 No devices found at any protocol or baudrate.")
    else:
        print(f"\n✅ Total devices found: {len(all_found)}")
        for entry in all_found:
            print(f" - ID {entry[0]} | Model {entry[1]} | Protocol {entry[2]} | Baudrate {entry[3]}")

    port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
