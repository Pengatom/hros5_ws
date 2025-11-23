#!/usr/bin/env python3
"""
hros5_servo_check.py

Scan Dynamixel servos on /dev/dxl (default), IDs 0–25, using:
- Protocol 1.0 or 2.0
- Configurable baudrate (default 1,000,000)

Reports: ID, model number, firmware version (if readable).
"""

import argparse
import sys
from pathlib import Path

# --- DEFAULT CONFIG --------------------------------------------
DEFAULT_DEV_NAME = "/dev/dxl"          # udev symlink you just made
DEFAULT_BAUDRATE = 1000000             # 1 Mbps
DEFAULT_PROTOCOL = 2.0                 # MX(2.0) / X-series / etc
DEFAULT_ID_START = 0
DEFAULT_ID_END = 25                    # inclusive

# Firmware version address per protocol
# Protocol 1.0 servos expose FW at address 2, Protocol 2.0 at 6.
PROTOCOL_FIRMWARE_ADDR = {
    1.0: 2,
    2.0: 6,
}

# Path to your local DynamixelSDK Python module
SDK_PY_PATH = Path("/home/pengatom/hros5_ws/src/dynamixel/DynamixelSDK/python/src")
# ---------------------------------------------------------------

# Add SDK to Python path
sys.path.append(str(SDK_PY_PATH))

try:
    from dynamixel_sdk import *  # type: ignore
except ImportError as e:
    print(f"❌ Could not import dynamixel_sdk from {SDK_PY_PATH}")
    print("   Check that the path is correct and that the SDK is present.")
    sys.exit(1)


def parse_args():
    parser = argparse.ArgumentParser(description="Scan Dynamixel servos.")
    parser.add_argument(
        "--port",
        default=DEFAULT_DEV_NAME,
        help=f"Serial port/udev path (default: {DEFAULT_DEV_NAME})",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUDRATE,
        help=f"Baudrate (default: {DEFAULT_BAUDRATE})",
    )
    parser.add_argument(
        "--protocol",
        type=float,
        choices=sorted(PROTOCOL_FIRMWARE_ADDR.keys()),
        default=DEFAULT_PROTOCOL,
        help="Dynamixel protocol version (1.0 or 2.0).",
    )
    parser.add_argument(
        "--id-start",
        type=int,
        default=DEFAULT_ID_START,
        help=f"Starting ID to scan (default: {DEFAULT_ID_START})",
    )
    parser.add_argument(
        "--id-end",
        type=int,
        default=DEFAULT_ID_END,
        help=f"Ending ID to scan (default: {DEFAULT_ID_END})",
    )
    parser.add_argument(
        "--firmware-addr",
        type=int,
        default=None,
        help=(
            "Control table address for firmware version "
            "(default depends on protocol)."
        ),
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if args.id_start > args.id_end:
        print("❌ --id-start must be <= --id-end")
        sys.exit(1)
    firmware_addr = args.firmware_addr
    if firmware_addr is None:
        firmware_addr = PROTOCOL_FIRMWARE_ADDR.get(args.protocol)
        if firmware_addr is None:
            print(
                "❌ Firmware address unknown for protocol "
                f"{args.protocol}. Supply --firmware-addr."
            )
            sys.exit(1)

    print("=== HROS5 Servo Check ===")
    print(f"Using SDK path : {SDK_PY_PATH}")
    print(f"Port           : {args.port}")
    print(f"Baudrate       : {args.baud}")
    print(f"Protocol       : {args.protocol}")
    print(f"Firmware addr  : {firmware_addr}")
    print(f"Scan IDs       : {args.id_start} to {args.id_end}")
    print("")

    port = PortHandler(args.port)
    packet = PacketHandler(args.protocol)

    # Open port
    if not port.openPort():
        print(f"❌ Failed to open port {args.port}")
        sys.exit(1)
    print(f"✅ Opened port {args.port}")

    # Set baud rate
    if not port.setBaudRate(args.baud):
        print(f"❌ Failed to set baudrate to {args.baud}")
        port.closePort()
        sys.exit(1)
    print(f"✅ Baudrate set to {args.baud}")
    print("")

    found = []

    print("Scanning...")
    print(f"{'ID':>3} | {'Model#':>7} | {'FW':>3} | Status")
    print("-" * 32)

    for dxl_id in range(args.id_start, args.id_end + 1):
        dxl_model_number, dxl_comm_result, dxl_error = packet.ping(port, dxl_id)

        if dxl_comm_result != COMM_SUCCESS:
            # No response from this ID, skip noisy output:
            # Uncomment next line if you want to see all failures too.
            # print(f"{dxl_id:>3} |   ----  | --- | {packet.getTxRxResult(dxl_comm_result)}")
            continue

        if dxl_error != 0:
            status = f"⚠ Error: {packet.getRxPacketError(dxl_error)}"
            fw_str = "---"
        else:
            # Try to read firmware version
            dxl_fw, fw_comm_result, fw_error = packet.read1ByteTxRx(
                port, dxl_id, firmware_addr
            )
            if fw_comm_result == COMM_SUCCESS and fw_error == 0:
                fw_str = f"{dxl_fw}"
                status = "OK"
            else:
                fw_str = "---"
                status = "FW read failed"

        found.append((dxl_id, dxl_model_number, fw_str, status))
        print(f"{dxl_id:>3} | {dxl_model_number:>7} | {fw_str:>3} | {status}")

    print("")
    if not found:
        print("❌ No servos found in the specified ID range.")
    else:
        print(f"✅ Done. Found {len(found)} servo(s).")

    port.closePort()
    print("🔚 Port closed.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nInterrupted by user, exiting...")
