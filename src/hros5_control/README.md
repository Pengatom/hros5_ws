# hros5_control

ros2_control configuration package for HR-OS5.

This package provides position and trajectory controllers for arms, legs,
hands, and other Dynamixel-driven joints in the robot. It also includes
joint limits and controller configuration files.

## Features

- ros2_control configuration for HR-OS5 joints
- Joint limit YAML definitions
- Position and trajectory controller setups
- Per-limb controller test launch files

Joint limits live in `config/hros5_dynamixel_joints_with_limits.yaml`. Per-limb joint YAMLs under `config/joints/*.yaml` are generated from that source via:

```
python3 src/hros5_utils/scripts/generate_joint_yamls.py
```

Use `--check` in CI to ensure they stay in sync. The generated files include a `DO NOT EDIT` header.

## Directory structure

```text
hros5_control/
├── CMakeLists.txt
├── config
│   ├── hand_test_controllers.yaml
│   ├── hros5_dynamixel_joints_with_limits.yaml
│   ├── left_arm_position_controller.yaml
│   ├── left_leg_controller.yaml
│   ├── position_controllers.yaml
│   ├── right_arm_position_controller.yaml
│   ├── right_leg_controller.yaml
│   └── trajectory_controllers.yaml
├── launch
│   ├── bringup_with_control.launch.py
│   ├── launch_leg_trajectory_control.py
│   ├── left_leg_control.launch.py
│   ├── position.launch.py
│   └── trajectory.launch.py
└── package.xml
Status
Active foundation of the control stack (P0–P2).
Will expand with IK and balance layers (P3–P4).
