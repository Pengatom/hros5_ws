#!/usr/bin/env python3
"""
dxl_fw_version.py

Query firmware version registers for one or more Dynamixels.

Example:
  python3 dxl_fw_version.py --port /dev/dxl --baud 1000000 --protocol 2.0 --ids 1 2 3
"""

import argparse
import sys
from typing import Optional

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)

# Default firmware register per protocol
FIRMWARE_ADDR = {
    1.0: 2,
    2.0: 6,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read firmware version for specified Dynamixel IDs.")
    parser.add_argument("--port", "--device", dest="port", default="/dev/dxl", help="Serial device path (default: /dev/dxl)")
    parser.add_argument("--baud", type=int, default=1000000, help="Baudrate (default: 1000000)")
    parser.add_argument("--protocol", type=float, choices=[1.0, 2.0], default=2.0, help="Dynamixel protocol (default: 2.0)")
    parser.add_argument("--ids", type=int, nargs="+", required=True, help="Servo IDs to query")
    parser.add_argument(
        "--firmware-addr",
        type=int,
        default=None,
        help="Override control table address for firmware version (default depends on protocol)",
    )
    return parser.parse_args()


def resolve_firmware_addr(protocol: float, override: Optional[int]) -> int:
    if override is not None:
        return override
    addr = FIRMWARE_ADDR.get(protocol)
    if addr is None:
        raise ValueError(f"Unsupported protocol version {protocol}")
    return addr


def read_firmware(packet: PacketHandler, port: PortHandler, dxl_id: int, addr: int) -> Optional[int]:
    value, result, error = packet.read1ByteTxRx(port, dxl_id, addr)
    if result == COMM_SUCCESS and error == 0:
        return value
    if result != COMM_SUCCESS:
        print(f"❌ ID {dxl_id} comm error: {packet.getTxRxResult(result)}")
    elif error != 0:
        print(f"❌ ID {dxl_id} packet error: {packet.getRxPacketError(error)}")
    return None


def main() -> int:
    args = parse_args()
    fw_addr = resolve_firmware_addr(args.protocol, args.firmware_addr)

    port = PortHandler(args.port)
    packet = PacketHandler(args.protocol)

    if not port.openPort():
        print(f"❌ Failed to open port: {args.port}", file=sys.stderr)
        return 1
    if not port.setBaudRate(args.baud):
        print(f"❌ Failed to set baudrate: {args.baud}", file=sys.stderr)
        port.closePort()
        return 1

    print(f"Reading firmware versions from IDs {args.ids} (protocol {args.protocol}, addr {fw_addr})")
    for dxl_id in args.ids:
        fw_version = read_firmware(packet, port, dxl_id, fw_addr)
        if fw_version is not None:
            print(f"✅ ID {dxl_id}: firmware {fw_version}")

    port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
