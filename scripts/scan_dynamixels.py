#!/usr/bin/env python3
import time
from dynamixel_sdk import *

# -----------------------------
DEV_NAME = "/dev/ttyUSB0"
SCAN_RANGE = range(1, 30)

# Common baudrates used by Dynamixel motors
BAUD_RATES = [
    57600,
 #   115200,
    1000000,
 #   2000000,
]

PROTOCOL_VERSIONS = [1.0, 2.0]
# -----------------------------

def scan_bus(portHandler, protocol_version, baudrate):
    packetHandler = PacketHandler(protocol_version)
    found = []

    if not portHandler.setBaudRate(baudrate):
        print(f"⚠️ Failed to set baudrate: {baudrate}")
        return found

    print(f"\n🔍 Scanning with Protocol {protocol_version} at {baudrate} bps...")
    for dxl_id in SCAN_RANGE:
        dxl_model_number, dxl_comm_result, dxl_error = packetHandler.ping(portHandler, dxl_id)
        if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
            print(f"✅ Found ID {dxl_id} | Model: {dxl_model_number}")
            found.append((dxl_id, dxl_model_number, protocol_version, baudrate))
        elif dxl_comm_result != COMM_RX_TIMEOUT:
            print(f"⚠️ ID {dxl_id} error: {packetHandler.getTxRxResult(dxl_comm_result)}")

    return found

def main():
    portHandler = PortHandler(DEV_NAME)

    if not portHandler.openPort():
        print(f"❌ Failed to open port: {DEV_NAME}")
        return

    all_found = []

    for protocol in PROTOCOL_VERSIONS:
        for baud in BAUD_RATES:
            found = scan_bus(portHandler, protocol, baud)
            if found:
                all_found.extend(found)
                break  # Comment this if you want to try all combos

        if all_found:
            break

    if not all_found:
        print("\n🚫 No devices found at any protocol or baudrate.")
    else:
        print(f"\n✅ Total devices found: {len(all_found)}")
        for entry in all_found:
            print(f" - ID {entry[0]} | Model {entry[1]} | Protocol {entry[2]} | Baudrate {entry[3]}")

    portHandler.closePort()

if __name__ == "__main__":
    main()

