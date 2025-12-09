#!/usr/bin/env python3
"""
Send a goal position to a single Dynamixel and report the present position.

Useful for quick wiring/ID sanity checks.
"""

from __future__ import annotations

import argparse

from dxl_common import DEFAULT_BAUD, DEFAULT_PORT, DEFAULT_PROTOCOL, import_sdk

# MX-style control table for Protocol 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Write a goal position and read back the present position.")
    parser.add_argument("--port", "-p", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD, help=f"Baudrate (default: {DEFAULT_BAUD})")
    parser.add_argument("--protocol", type=float, default=DEFAULT_PROTOCOL, help=f"Protocol version (default: {DEFAULT_PROTOCOL})")
    parser.add_argument("--id", "-i", type=int, required=True, help="Servo ID to command")
    parser.add_argument("--goal", type=int, default=2048, help="Goal position in ticks (default: 2048)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    sdk = import_sdk()
    port = sdk.PortHandler(args.port)
    packet = sdk.PacketHandler(args.protocol)

    if not port.openPort():
        print(f"❌ Failed to open port {args.port}")
        return 1
    if not port.setBaudRate(args.baud):
        print(f"❌ Failed to set baudrate {args.baud}")
        port.closePort()
        return 1

    # Enable torque
    result, error = packet.write1ByteTxRx(port, args.id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if result != sdk.COMM_SUCCESS or error != 0:
        print("❌ Torque enable failed:", packet.getTxRxResult(result), packet.getRxPacketError(error))
        port.closePort()
        return 1
    print("✅ Torque enabled")

    # Write goal position
    result, error = packet.write4ByteTxRx(port, args.id, ADDR_GOAL_POSITION, args.goal)
    if result != sdk.COMM_SUCCESS or error != 0:
        print("❌ Goal write failed:", packet.getTxRxResult(result), packet.getRxPacketError(error))
        port.closePort()
        return 1
    print(f"✅ Goal position {args.goal} written")

    # Read present position
    position, result, error = packet.read4ByteTxRx(port, args.id, ADDR_PRESENT_POSITION)
    if result != sdk.COMM_SUCCESS:
        print("⚠️  Read failed:", packet.getTxRxResult(result))
    elif error != 0:
        print("⚠️  Read error:", packet.getRxPacketError(error))
    else:
        print(f"Present position: {position}")

    packet.write1ByteTxRx(port, args.id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
