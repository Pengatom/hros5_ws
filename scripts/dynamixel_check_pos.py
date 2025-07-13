from dynamixel_sdk import *  # pip install dynamixel-sdk
import time

# MX-28AT (protocol 2.0)
ADDR_PRESENT_POSITION  = 132
PROTOCOL_VERSION       = 2.0
DXL_ID                 = 24              # Set to your servo's ID
BAUDRATE               = 1000000
DEVICENAME             = "/dev/ttyUSB0"  # Adjust as needed

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

if not portHandler.openPort():
    print(f"Failed to open port {DEVICENAME}")
    quit()

if not portHandler.setBaudRate(BAUDRATE):
    print(f"Failed to set baudrate {BAUDRATE}")
    quit()

print("Port opened and baudrate set")

try:
    while True:
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
            portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        if dxl_comm_result != COMM_SUCCESS:
            print("Read failed:", packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("Read error:", packetHandler.getRxPacketError(dxl_error))
        else:
            print(f"Present position: {dxl_present_position}")
        time.sleep(1)  # 1 Hz
except KeyboardInterrupt:
    print("\nExiting...")

portHandler.closePort()
print("Finished.")
