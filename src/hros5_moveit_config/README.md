# hros5_moveit_config

MoveIt2 configuration for the full HR-OS5 model.

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
│   ├── hros5.ros2_control.xacro
│   ├── hros5.srdf
│   ├── hros5.urdf.xacro
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
Initial MoveIt testbed for full-body HR-OS5.
