A ROS 2 C++ package for controlling a full biped (humanoid) robot with Dynamixel servos using YAML configuration.  
**YAML-driven:** Add/remove servos, change joint IDs or models, or set limits—all without recompiling.

---

## Features

- Control any number of Dynamixel servos (MX-28, MX-64, MX-106, etc.), with joint names/IDs/models read from a YAML file.
- **SyncRead/SyncWrite** for high-speed multi-servo operation.
- Publishes `sensor_msgs/msg/JointState` (compatible with RViz/MoveIt2).
- Subscribes to `trajectory_msgs/msg/JointTrajectory` (standard ROS 2 interface for planners and GUIs).
- Easily switch between real hardware and simulation (Gazebo/ros2_control).
- Minimal URDF included, plug in your own full robot xacros/meshes.

---

## Folder Structure

hros5_dynamixel_bridge/
├── CMakeLists.txt
├── package.xml
├── README.md
├── src/
│ ├── dynamixel_bridge_node.cpp
│ └── dynamixel_driver.cpp
├── include/
│ └── hros5_dynamixel_bridge/
│ └── dynamixel_driver.hpp
├── config/
│ └── joints.yaml
├── launch/
│ └── bringup.launch.py
└── urdf/
└── hros5_minimal.urdf.xacro

yaml
Always show details

Copy

---


## Quickstart

1. **Place this package inside your ROS 2 workspace:**
cd ~/hros5_ws/src

Copy or clone this folder here
markdown
Always show details

Copy

2. **Install dependencies:**
sudo apt update
sudo apt install ros-jazzy-dynamixel-sdk ros-jazzy-yaml-cpp

markdown
Always show details

Copy

3. **Build:**
cd ~/hros5_ws
colcon build
source install/setup.bash

markdown
Always show details

Copy

4. **Edit `config/joints.yaml` as needed.**  
Set joint names, IDs, models, and limits.

5. **Launch the driver:**
ros2 launch hros5_dynamixel_bridge bringup.launch.py

markdown
Always show details

Copy

6. **Test publishing a trajectory command:**
ros2 topic pub /joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names: ['RShoulderPitch']
points:

positions: [0.5]
time_from_start: {sec: 2, nanosec: 0 }
"

markdown
Always show details

Copy

7. **Visualize in RViz2:**
rviz2 -d <your rviz config, or add /joint_states display>

yaml
Always show details

Copy

8. **Gazebo Simulation:**  
Use the included `urdf/hros5_minimal.urdf.xacro` or plug in your own robot description with matching joint names.  
For sim, just launch Gazebo with the URDF as you normally would.

---


## Configuration

- **All joints are defined in `config/joints.yaml`**:
- `name`: Joint name (must match URDF/xacro)
- `id`: Servo ID (as set with DYNAMIXEL Wizard or other tool)
- `model`: Servo model (MX-28, MX-64, MX-106, etc. For info/debug—implementation assumes Protocol 2.0 0-4095 position range)
- `cw_limit`/`ccw_limit`: Hardware limits; use `TBD` for full range

- **Port and baudrate:**  
Default is `/dev/ttyUSB0` at `1000000`. Edit `launch/bringup.launch.py` or use ROS 2 params to override.

---


## Adding, Removing, or Modifying Joints

- **To add a new servo:**  
Add a new YAML entry with the correct name, ID, and model.
- **To remove a servo:**  
Delete its entry in YAML.
- **To change model, ID, or limits:**  
Edit YAML, save, and restart the node.
- **No code changes needed for config updates!**

---


## Switching Between Real Hardware and Simulation

- **Hardware:**  
Run the provided launch file. Your YAML controls which servos are driven.
- **Simulation (Gazebo):**  
Use the same URDF/xacro with a standard ros2_control simulation plugin.  
- *Tip:* Set up your MoveIt2/ros2_control config to match joint names/limits.
- You can mix and match: run hardware for some joints, sim for others (advanced).

---


## Troubleshooting

- **Node won't start, can't find serial port:**  
Check your U2D2 or USB2AX is plugged in and appears as `/dev/ttyUSB0`.  
Use `ls /dev/ttyUSB*` to check device names.

- **Servo doesn't move:**  
Double-check servo ID, power, and wiring.  
Confirm correct model in YAML.  
Use DYNAMIXEL Wizard to verify hardware.

- **No joint_states in RViz/MoveIt2:**  
Make sure YAML joint names match your robot URDF/xacro exactly.

- **Build errors:**  
Ensure all dependencies are installed (`dynamixel_sdk`, `yaml-cpp`, `rclcpp`, etc.).

---


## Extending

- **Torque on/off, advanced commands:**  
Can be added as ROS 2 services or topics—see `dynamixel_driver.cpp` for code hooks.
- **Diagnostics:**  
Add more SyncRead for currents, temperature, etc. in the driver.
- **Custom message mapping:**  
You can subscribe to other topics or bridge additional control modes as needed.

---


## License

Apache 2.0

---


**Happy Hacking!**

This package is meant to be readable, robust, and easy to extend.  
Got a feature request? Want to upstream improvements? Just ask!
""")