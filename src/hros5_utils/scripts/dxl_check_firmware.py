#!/usr/bin/env python3
"""
Read and report firmware versions for one or more Dynamixel IDs.
"""

from __future__ import annotations

import argparse
from typing import Iterable, List

from dxl_common import DEFAULT_BAUD, DEFAULT_PORT, DEFAULT_PROTOCOL, import_sdk


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Read firmware version registers from Dynamixel servos.")
    parser.add_argument("--port", "-p", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD, help=f"Baudrate (default: {DEFAULT_BAUD})")
    parser.add_argument("--protocol", type=float, default=DEFAULT_PROTOCOL, help=f"Protocol version (default: {DEFAULT_PROTOCOL})")
    parser.add_argument(
        "--ids",
        type=int,
        nargs="+",
        default=[1],
        help="List of IDs to query (default: 1)",
    )
    parser.add_argument(
        "--addr",
        type=int,
        default=None,
        help="Control table address for firmware version (default: 2 for protocol 1.0, 6 for protocol 2.0).",
    )
    return parser.parse_args()


def iter_ids(values: Iterable[int]) -> List[int]:
    seen = set()
    ordered = []
    for value in values:
        if value not in seen:
            ordered.append(value)
            seen.add(value)
    return ordered


def main() -> int:
    args = parse_args()
    sdk = import_sdk()
    packet = sdk.PacketHandler(args.protocol)
    port = sdk.PortHandler(args.port)
    fw_addr = args.addr if args.addr is not None else (2 if args.protocol == 1.0 else 6)

    if not port.openPort():
        print(f"❌ Failed to open port: {args.port}")
        return 1
    if not port.setBaudRate(args.baud):
        print(f"❌ Failed to set baudrate: {args.baud}")
        port.closePort()
        return 1

    ids = iter_ids(args.ids)
    print(f"Checking firmware at addr {fw_addr} for IDs: {ids}")

    for dxl_id in ids:
        fw_version, comm, dxl_err = packet.read1ByteTxRx(port, dxl_id, fw_addr)
        if comm == sdk.COMM_SUCCESS and dxl_err == 0:
            print(f"✅ ID {dxl_id}: firmware version {fw_version}")
        else:
            print(
                f"❌ ID {dxl_id}: "
                f"{packet.getTxRxResult(comm)}"
                f"{' / ' + packet.getRxPacketError(dxl_err) if dxl_err else ''}"
            )

    port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
