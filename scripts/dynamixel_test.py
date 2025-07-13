from dynamixel_sdk import *  # pip install dynamixel-sdk

# MX-28AT (protocol 2.0)
ADDR_TORQUE_ENABLE     = 64
ADDR_GOAL_POSITION     = 116
ADDR_PRESENT_POSITION  = 132

PROTOCOL_VERSION       = 2.0
DXL_ID                 = 22                # Set to your servo's ID
BAUDRATE               = 1000000
DEVICENAME             = "/dev/ttyUSB0"   # Adjust as needed

TORQUE_ENABLE          = 1
TORQUE_DISABLE         = 0
DXL_MIN_POSITION       = 0
DXL_MAX_POSITION       = 4095
GOAL_POSITION          = 2048             # Center

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

# Enable torque
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
    portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print("Torque enable failed:", packetHandler.getTxRxResult(dxl_comm_result))
    quit()
elif dxl_error != 0:
    print("Torque enable error:", packetHandler.getRxPacketError(dxl_error))
    quit()
else:
    print("Torque enabled")

# Write goal position
dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
    portHandler, DXL_ID, ADDR_GOAL_POSITION, GOAL_POSITION)
if dxl_comm_result != COMM_SUCCESS:
    print("Goal position failed:", packetHandler.getTxRxResult(dxl_comm_result))
    quit()
elif dxl_error != 0:
    print("Goal position error:", packetHandler.getRxPacketError(dxl_error))
    quit()
else:
    print(f"Goal position {GOAL_POSITION} written")

# Read present position
dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
    portHandler, DXL_ID, ADDR_PRESENT_POSITION)
if dxl_comm_result != COMM_SUCCESS:
    print("Read failed:", packetHandler.getTxRxResult(dxl_comm_result))
elif dxl_error != 0:
    print("Read error:", packetHandler.getRxPacketError(dxl_error))
else:
    print(f"Present position: {dxl_present_position}")

# Disable torque before exit
packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
portHandler.closePort()
print("Finished.")
