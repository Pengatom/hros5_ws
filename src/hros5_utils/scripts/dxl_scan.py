#!/usr/bin/env python3
"""
Scan a Dynamixel bus for active IDs and report model + firmware (when available).

Supports multiple baudrates and protocol versions in one pass to make
identifying unknown servos easier.
"""

from __future__ import annotations

import argparse
import sys
from typing import Dict, Iterable, List, Optional

from dxl_common import DEFAULT_BAUD, DEFAULT_PORT, DEFAULT_PROTOCOL, SDK_FALLBACK_PATH, import_sdk

PROTOCOL_FIRMWARE_ADDR = {
    1.0: 2,
    2.0: 6,
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Scan Dynamixel IDs and print model/firmware info.")
    parser.add_argument("--port", "-p", default=DEFAULT_PORT, help=f"Serial port/udev path (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", "-b", type=int, default=DEFAULT_BAUD, help=f"Default baudrate (default: {DEFAULT_BAUD})")
    parser.add_argument(
        "--baud-list",
        type=int,
        nargs="+",
        help="Optional list of baudrates to try instead of a single value.",
    )
    parser.add_argument(
        "--protocols",
        type=float,
        nargs="+",
        default=[DEFAULT_PROTOCOL],
        help="Protocol versions to try (e.g., 1.0 2.0).",
    )
    parser.add_argument("--id-start", type=int, default=0, help="Starting ID to scan (default: 0)")
    parser.add_argument("--id-end", type=int, default=25, help="Ending ID to scan (inclusive, default: 25)")
    parser.add_argument(
        "--firmware-addr",
        type=int,
        default=None,
        help="Override control-table address for firmware version. Defaults per protocol.",
    )
    parser.add_argument(
        "--no-firmware",
        action="store_true",
        help="Skip firmware reads (faster).",
    )
    parser.add_argument(
        "--stop-on-first",
        action="store_true",
        help="Stop scanning after the first device is found.",
    )
    return parser.parse_args()


def read_firmware(packet, port, dxl_id: int, firmware_addr: Optional[int], sdk) -> str:
    if firmware_addr is None:
        return "---"
    fw, res, err = packet.read1ByteTxRx(port, dxl_id, firmware_addr)
    if res == sdk.COMM_SUCCESS and err == 0:
        return str(fw)
    return "---"


def scan_ids(
    packet,
    port,
    ids: Iterable[int],
    sdk,
    firmware_addr: Optional[int],
    read_fw: bool,
) -> List[Dict[str, int | str | float]]:
    found = []
    print(f"{'ID':>3} | {'Model#':>7} | {'FW':>3} | Status")
    print("-" * 32)
    for dxl_id in ids:
        model, comm_result, dxl_error = packet.ping(port, dxl_id)
        if comm_result != sdk.COMM_SUCCESS:
            continue
        if dxl_error != 0:
            status = f"⚠ {packet.getRxPacketError(dxl_error)}"
            fw_str = "---"
        else:
            fw_str = read_firmware(packet, port, dxl_id, firmware_addr, sdk) if read_fw else "---"
            status = "OK" if fw_str != "---" else "RESP"

        found.append(
            {
                "id": dxl_id,
                "model": model,
                "firmware": fw_str,
            }
        )
        print(f"{dxl_id:>3} | {model:>7} | {fw_str:>3} | {status}")
    return found


def main() -> int:
    args = parse_args()
    sdk = import_sdk()

    baud_list = args.baud_list if args.baud_list else [args.baud]
    id_range = range(args.id_start, args.id_end + 1)

    port = sdk.PortHandler(args.port)
    if not port.openPort():
        print(f"❌ Failed to open port {args.port}")
        return 1

    all_found: List[Dict[str, int | str | float]] = []

    print(f"Using SDK path: {SDK_FALLBACK_PATH if str(SDK_FALLBACK_PATH) in sys.path else 'env site-packages'}")
    print(f"Port           : {args.port}")
    print(f"Baud options   : {baud_list}")
    print(f"Protocols      : {args.protocols}")
    print(f"ID range       : {args.id_start} to {args.id_end}")
    print(f"Read firmware  : {not args.no_firmware}")
    print("")

    try:
        for protocol in args.protocols:
            packet = sdk.PacketHandler(protocol)
            fw_addr = args.firmware_addr if args.firmware_addr is not None else PROTOCOL_FIRMWARE_ADDR.get(protocol)

            for baud in baud_list:
                if not port.setBaudRate(baud):
                    print(f"⚠️  Failed to set baudrate {baud}, skipping.")
                    continue

                print(f"=== Scanning protocol {protocol} @ {baud} bps ===")
                found = scan_ids(
                    packet=packet,
                    port=port,
                    ids=id_range,
                    sdk=sdk,
                    firmware_addr=fw_addr,
                    read_fw=not args.no_firmware,
                )
                if not found:
                    print("No responses.\n")
                else:
                    all_found.extend(
                        {**item, "protocol": protocol, "baud": baud} for item in found
                    )
                    print(f"Found {len(found)} device(s) on this pass.\n")
                    if args.stop_on_first:
                        raise SystemExit(0)

    finally:
        port.closePort()

    if not all_found:
        print("🚫 No devices found at any protocol or baudrate.")
        return 2

    print("=== Summary ===")
    for item in all_found:
        print(
            f"ID {item['id']} | Model {item['model']} | FW {item['firmware']} "
            f"| Protocol {item['protocol']} | Baud {item['baud']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
