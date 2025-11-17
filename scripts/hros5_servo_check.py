#!/usr/bin/env python3
"""
hros5_servo_check.py

Scan Dynamixel servos on /dev/dxl, IDs 0–25, using:
- Protocol 2.0
- Baudrate 1,000,000

Reports: ID, model number, firmware version (if readable).
"""

import sys
from pathlib import Path

# --- CONFIG ----------------------------------------------------
DEV_NAME = "/dev/dxl"          # udev symlink you just made
BAUDRATE = 1000000             # 1 Mbps
PROTOCOL_VERSION = 2.0         # MX(2.0) / X-series / etc
ID_START = 0
ID_END = 25                    # inclusive

# Firmware version address in Control Table (Protocol 2.0)
# For most Dynamixel Protocol 2.0 servos, this is 6.
FIRMWARE_ADDR = 6

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


def main():
    print("=== HROS5 Servo Check ===")
    print(f"Using SDK path : {SDK_PY_PATH}")
    print(f"Port           : {DEV_NAME}")
    print(f"Baudrate       : {BAUDRATE}")
    print(f"Protocol       : {PROTOCOL_VERSION}")
    print(f"Scan IDs       : {ID_START} to {ID_END}")
    print("")

    port = PortHandler(DEV_NAME)
    packet = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if not port.openPort():
        print(f"❌ Failed to open port {DEV_NAME}")
        sys.exit(1)
    print(f"✅ Opened port {DEV_NAME}")

    # Set baud rate
    if not port.setBaudRate(BAUDRATE):
        print(f"❌ Failed to set baudrate to {BAUDRATE}")
        port.closePort()
        sys.exit(1)
    print(f"✅ Baudrate set to {BAUDRATE}")
    print("")

    found = []

    print("Scanning...")
    print(f"{'ID':>3} | {'Model#':>7} | {'FW':>3} | Status")
    print("-" * 32)

    for dxl_id in range(ID_START, ID_END + 1):
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
                port, dxl_id, FIRMWARE_ADDR
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
