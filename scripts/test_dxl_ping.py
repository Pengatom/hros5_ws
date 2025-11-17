#!/usr/bin/env python3

import sys
from pathlib import Path

# Add DynamixelSDK python path
sdk_path = Path("/home/pengatom/hros5_ws/src/dynamixel/DynamixelSDK/python/src")
sys.path.append(str(sdk_path))

from dynamixel_sdk import *  # Uses Dynamixel SDK library

# ===== CONFIG: adjust if needed =====
DEV_NAME = "/dev/dxl"         # our udev symlink
BAUDRATE = 1000000            # 1 Mbps
PROTOCOL_VERSION = 2.0        # you said protocol 2.0
DXL_ID = 19                   # change if your servo ID is not 1
# ====================================

def main():
    print(f"Using SDK from: {sdk_path}")
    print(f"Opening {DEV_NAME} @ {BAUDRATE} baud, protocol {PROTOCOL_VERSION}, ID {DXL_ID}")

    port = PortHandler(DEV_NAME)
    packet = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if not port.openPort():
        print(f"❌ Failed to open port {DEV_NAME}")
        return
    print(f"✅ Opened port {DEV_NAME}")

    # Set baud rate
    if not port.setBaudRate(BAUDRATE):
        print(f"❌ Failed to set baudrate to {BAUDRATE}")
        port.closePort()
        return
    print(f"✅ Baudrate set to {BAUDRATE}")

    # Ping the servo
    print(f"➡️  Pinging Dynamixel ID {DXL_ID}...")
    dxl_model_number, dxl_comm_result, dxl_error = packet.ping(port, DXL_ID)

    if dxl_comm_result != COMM_SUCCESS:
        print(f"❌ Communication failed: {packet.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"⚠️  Error from servo: {packet.getRxPacketError(dxl_error)}")
    else:
        print(f"✅ Success! ID {DXL_ID} responded, model number = {dxl_model_number}")

    port.closePort()
    print("🔚 Port closed.")

if __name__ == "__main__":
    main()

