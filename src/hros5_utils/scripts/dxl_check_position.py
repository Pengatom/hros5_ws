#!/usr/bin/env python3
"""
dxl_check_position.py

Continuously read the Present Position register from a single Dynamixel.

Example:
  python3 dxl_check_position.py --id 24 --port /dev/dxl --baud 1000000 --protocol 2.0 --interval 0.5
"""

import argparse
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

ADDR_PRESENT_POSITION = 132


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Poll the Present Position register for a single servo.")
    parser.add_argument("--id", type=int, required=True, help="Servo ID to read")
    parser.add_argument("--port", "--device", dest="port", default="/dev/dxl", help="Serial device (default: /dev/dxl)")
    parser.add_argument("--baud", type=int, default=1000000, help="Baudrate (default: 1000000)")
    parser.add_argument("--protocol", type=float, default=2.0, help="Dynamixel protocol version (default: 2.0)")
    parser.add_argument("--interval", type=float, default=1.0, help="Polling interval in seconds (default: 1.0)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    port = PortHandler(args.port)
    packet = PacketHandler(args.protocol)

    if not port.openPort():
        print(f"Failed to open port {args.port}", file=sys.stderr)
        return 1
    if not port.setBaudRate(args.baud):
        print(f"Failed to set baudrate {args.baud}", file=sys.stderr)
        port.closePort()
        return 1

    print(f"Reading Present Position for ID {args.id} every {args.interval}s (protocol {args.protocol})")
    try:
        while True:
            position, result, error = packet.read4ByteTxRx(port, args.id, ADDR_PRESENT_POSITION)
            if result != COMM_SUCCESS:
                print("Read failed:", packet.getTxRxResult(result))
            elif error != 0:
                print("Read error:", packet.getRxPacketError(error))
            else:
                print(f"ID {args.id}: {position}")
            time.sleep(max(0.01, args.interval))
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
