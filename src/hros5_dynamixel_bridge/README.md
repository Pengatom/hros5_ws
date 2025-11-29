# hros5_dynamixel_bridge

ROS 2 interface layer for Dynamixel servos.

Implements:

- Bus setup (PortHandler/PacketHandler)
- Per-joint reading and writing
- Joint name → servo ID mapping
- One node per body region (head, hands, arms)

## Directory structure

```text
hros5_dynamixel_bridge/
├── CMakeLists.txt
├── config
│   └── joints.yaml
├── include
│   └── hros5_dynamixel_bridge
│       └── dynamixel_driver.hpp
├── launch
│   ├── bringup.launch.py
│   └── rviz_bringup.launch.py
├── package.xml
├── README.md
├── src
│   ├── dynamixel_bridge_node.cpp
│   ├── dynamixel_driver.cpp
│   ├── head_dxl_node.cpp
│   ├── left_arm_dxl_node.cpp
│   ├── left_hand_dxl_node.cpp
│   ├── right_arm_dxl_node.cpp
│   └── right_hand_dxl_node.cpp
└── urdf
    └── hros5_minimal.urdf.xacro
Status
Core hardware interface (P0–P2).
Future: improved diagnostics (temperature, load, voltage).