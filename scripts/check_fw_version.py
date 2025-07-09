#!/usr/bin/env python3
from dynamixel_sdk import *

# === USER CONFIG ===
DEV_NAME = "/dev/ttyACM0"
BAUDRATE = 1000000
PROTOCOL_VERSION = 1.0
TARGET_IDS = [22, 24]

FIRMWARE_ADDR = 2
FIRMWARE_LEN = 1

def main():
    portHandler = PortHandler(DEV_NAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    if not portHandler.openPort():
        print(f"❌ Failed to open port: {DEV_NAME}")
        return
    if not portHandler.setBaudRate(BAUDRATE):
        print(f"❌ Failed to set baudrate: {BAUDRATE}")
        return

    for dxl_id in TARGET_IDS:
        print(f"\n📦 Reading firmware version for ID {dxl_id}...")
        fw_version, dxl_comm_result, dxl_error = packetHandler.read1ByteTxRx(
            portHandler, dxl_id, FIRMWARE_ADDR
        )

        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
            print(f"✅ ID {dxl_id} firmware version: {fw_version}")
        else:
            print(f"❌ Failed to read ID {dxl_id}: {packetHandler.getTxRxResult(dxl_comm_result)} / {packetHandler.getRxPacketError(dxl_error)}")

    portHandler.closePort()

if __name__ == "__main__":
    main()

