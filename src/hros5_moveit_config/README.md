# hros5_moveit_config

MoveIt2 configuration for HR-OS5 (currently the “hands_test” model only).

Includes:

- SRDF
- Kinematics parameters
- MoveIt controllers
- Launch files for MoveGroup + RViz

## Directory structure

```text
hros5_moveit_config/
├── CMakeLists.txt
├── config
│   ├── hands_test.ros2_control.xacro
│   ├── hands_test.srdf
│   ├── hands_test.urdf.xacro
│   ├── initial_positions.yaml
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── moveit_controllers.yaml
│   ├── moveit.rviz
│   ├── pilz_cartesian_limits.yaml
│   └── ros2_controllers.yaml
├── launch
│   ├── demo.launch.py
│   ├── move_group.launch.py
│   ├── moveit_rviz.launch.py
│   ├── rsp.launch.py
│   ├── setup_assistant.launch.py
│   ├── spawn_controllers.launch.py
│   ├── static_virtual_joint_tfs.launch.py
│   └── warehouse_db.launch.py
└── package.xml
Status
Initial MoveIt testbed (P2).
Will later expand to full-body configuration.