# hros5_demos

Testing and demonstration package for HR-OS5.

Used as a sandbox for:

- Trying controller configs
- Isolated Dynamixel tests
- URDF experiments
- Small-scale servo tests

## Directory structure

```text
hros5_demos/
├── CMakeLists.txt
├── config
│   ├── dynamixel_controllers.yaml
│   ├── dynamixel_test_controllers.yaml
│   └── two_mx28.yaml
├── launch
│   ├── test_dynamixel.launch.py
│   └── test_two_servos.launch.py
├── package.xml
└── urdf
    ├── test_robot.urdf.xacro
    ├── test.urdf
    └── test_urdf.xacro
Status
Stable development sandbox.
Used during P0–P1 phases, and helpful for future debugging.