# hros5_dynamixel_bridge

ROS 2 interface layer for Dynamixel servos.

Implements:

- Bus setup (PortHandler/PacketHandler)
- Per-joint reading and writing
- Joint name → servo ID mapping
- One node per body region (head, hands, arms, legs)
- Bus configuration YAML now lives in `hros5_control/config/dxl_bus.yaml`.
- Servo IDs and limits come from `hros5_control/config/joints/<limb>.yaml`. These files are generated, so don't edit them by hand.

Implementation progress: see [`docs/STATUS.md`](../../docs/STATUS.md).

## Directory structure

```text
hros5_dynamixel_bridge/
├── CMakeLists.txt
├── config
│   ├── head_dxl.yaml
│   ├── joints.yaml
│   ├── left_arm_dxl.yaml
│   ├── left_leg_dxl.yaml
│   ├── right_arm_dxl.yaml
│   └── right_leg_dxl.yaml
├── include
│   └── hros5_dynamixel_bridge
│       ├── dxl_bus_config.hpp
│       └── dynamixel_driver.hpp
├── launch
│   ├── bringup.launch.py
│   ├── head_dxl.launch.py
│   ├── left_arm_dxl.launch.py
│   ├── left_leg_dxl.launch.py
│   ├── right_arm_dxl.launch.py
│   ├── right_leg_dxl.launch.py
│   └── rviz_bringup.launch.py
├── package.xml
├── README.md
├── src
│   ├── dynamixel_bridge_node.cpp
│   ├── dynamixel_driver.cpp
│   ├── head_dxl_node.cpp
│   ├── left_arm_dxl_node.cpp
│   ├── left_hand_dxl_node.cpp
│   ├── left_leg_dxl_node.cpp
│   ├── right_arm_dxl_node.cpp
│   ├── right_hand_dxl_node.cpp
│   └── right_leg_dxl_node.cpp
└── urdf
    └── hros5_minimal.urdf.xacro
```
