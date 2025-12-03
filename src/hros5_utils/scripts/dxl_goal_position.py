#!/usr/bin/env python3
"""
dxl_goal_position.py

Enable torque, command a goal position, read back present position, and disable torque.

Example:
  python3 dxl_goal_position.py --id 22 --goal 2048 --port /dev/dxl --baud 1000000 --protocol 2.0
"""

import argparse
import sys

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

# MX-28AT (protocol 2.0) control table addresses
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Set a goal position on a Dynamixel and read it back.")
    parser.add_argument("--id", type=int, required=True, help="Servo ID to command")
    parser.add_argument("--goal", type=int, default=2048, help="Goal position (ticks, default: 2048)")
    parser.add_argument("--port", "--device", dest="port", default="/dev/dxl", help="Serial device (default: /dev/dxl)")
    parser.add_argument("--baud", type=int, default=1000000, help="Baudrate (default: 1000000)")
    parser.add_argument("--protocol", type=float, default=2.0, help="Dynamixel protocol version (default: 2.0)")
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

    print(f"Enabling torque on ID {args.id} and writing goal {args.goal}")

    result, error = packet.write1ByteTxRx(port, args.id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if result != COMM_SUCCESS or error != 0:
        status = packet.getTxRxResult(result) if result != COMM_SUCCESS else packet.getRxPacketError(error)
        print(f"Torque enable failed: {status}")
        port.closePort()
        return 1

    result, error = packet.write4ByteTxRx(port, args.id, ADDR_GOAL_POSITION, args.goal)
    if result != COMM_SUCCESS or error != 0:
        status = packet.getTxRxResult(result) if result != COMM_SUCCESS else packet.getRxPacketError(error)
        print(f"Goal position write failed: {status}")
        port.closePort()
        return 1
    print("Goal written, reading present position...")

    position, result, error = packet.read4ByteTxRx(port, args.id, ADDR_PRESENT_POSITION)
    if result != COMM_SUCCESS:
        print("Read failed:", packet.getTxRxResult(result))
    elif error != 0:
        print("Read error:", packet.getRxPacketError(error))
    else:
        print(f"Present position: {position}")

    packet.write1ByteTxRx(port, args.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    port.closePort()
    print("Finished.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
