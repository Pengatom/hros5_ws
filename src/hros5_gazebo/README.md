# hros5_gazebo

Gazebo simulation support package for HR-OS5.

Currently includes a minimal test environment focused on the left hand,
with plans to expand toward full-robot simulation.

## Directory structure

```text
hros5_gazebo/
├── CMakeLists.txt
├── config
│   └── left_hand_controllers.yaml
├── launch
│   └── left_hand_gazebo.launch.py
├── package.xml
├── urdf
│   └── hands
│       └── left_hand_gazebo.xacro
└── worlds
    └── empty.world
Status
Early simulation environment (P2).
Will expand to full-body simulation.