#!/usr/bin/env python3
"""
Continuously read and print the present position for a single Dynamixel.
"""

from __future__ import annotations

import argparse
import time

from dxl_common import DEFAULT_BAUD, DEFAULT_PORT, DEFAULT_PROTOCOL, import_sdk

ADDR_PRESENT_POSITION = 132


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Poll present position for one Dynamixel ID.")
    parser.add_argument("--port", "-p", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD, help=f"Baudrate (default: {DEFAULT_BAUD})")
    parser.add_argument("--protocol", type=float, default=DEFAULT_PROTOCOL, help=f"Protocol version (default: {DEFAULT_PROTOCOL})")
    parser.add_argument("--id", "-i", type=int, required=True, help="Servo ID to read")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between reads (default: 1.0)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sdk = import_sdk()
    port = sdk.PortHandler(args.port)
    packet = sdk.PacketHandler(args.protocol)

    if not port.openPort():
        print(f"Failed to open port {args.port}")
        return 1
    if not port.setBaudRate(args.baud):
        print(f"Failed to set baudrate {args.baud}")
        return 1

    print("Port opened and baudrate set")

    try:
        while True:
            position, comm, err = packet.read4ByteTxRx(port, args.id, ADDR_PRESENT_POSITION)
            if comm != sdk.COMM_SUCCESS:
                print("Read failed:", packet.getTxRxResult(comm))
            elif err != 0:
                print("Read error:", packet.getRxPacketError(err))
            else:
                print(f"Present position: {position}")
            time.sleep(max(0.01, args.interval))
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
