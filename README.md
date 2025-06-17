# 🤖 HR-OS5 ROS 2 Workspace (`hros5_ws`)

This repository contains the ROS 2 workspace for the **HR-OS5 humanoid robot**, built using **ROS 2 Jazzy**. It includes the robot description, visualization, navigation, teleoperation, and demo nodes, and will later support hardware interfaces and control.

---

## 🧠 Project Overview

- **Platform**: Ubuntu + ROS 2 Jazzy
- **Robot Type**: Humanoid
- **Development Flow**: Start in simulation → transition to real hardware
- **Core Features**:
  - Camera integration
  - Navigation (Nav2)
  - Remote control via PS4 controller
- **Tools Used**:
  - RViz 2
  - Gazebo
  - `joy` + `teleop_twist_joy`

---

## 🗂️ Repository Structure

```
hros5_ws/
├── src/
│   ├── hros5_description/   # URDF/XACRO files, robot meshes
│   ├── hros5_bringup/       # Launch files for RViz, Gazebo, etc.
│   ├── hros5_control/       # Future hardware interface implementation
│   ├── hros5_navigation/    # Navigation stack (Nav2) configuration
│   ├── hros5_teleop/        # PS4 controller integration
│   ├── hros5_demos/         # Test nodes, component demos
│   └── ...
├── install/
├── build/
├── log/
└── README.md
```

---

## 📦 Included Packages

- [`hros5_description`](src/hros5_description):  
  Contains the Xacro files, STL/DAE meshes, and launch configurations for visualizing the robot in RViz2.

- [`hros5_bringup`](src/hros5_bringup):  
  Central launch files to bring up the robot in simulation or real hardware, including RViz and Gazebo.

- [`hros5_control`](src/hros5_control):  
  Placeholder for future ROS 2 Control hardware interface and controller manager.

- [`hros5_navigation`](src/hros5_navigation):  
  Configuration for the Nav2 stack including map, localization, and planning setup.

- [`hros5_teleop`](src/hros5_teleop):  
  PS4 controller input using `joy` and `teleop_twist_joy` nodes.

- [`hros5_demos`](src/hros5_demos):  
  Standalone scripts and nodes to test motors, input devices, and subsystems.

---

## 🧰 Requirements

- **ROS 2 Jazzy**
- **colcon** build tool
- **Dynamixel SDK** (for servo communication)
- **USB2AX** or equivalent USB2Dynamixel adapter
- **PS4 controller** (optional, for input testing)

---

## 🔧 Build Instructions

```bash
# Clone the repository
git clone https://github.com/yourname/hros5_ws.git
cd hros5_ws

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Build workspace
colcon build

# Source workspace
source install/setup.bash
```

---

## 🧪 Run a Demo

Example: Launch the robot model in RViz2

```bash
ros2 launch hros5_description view_robot.launch.py
```

Example: Start navigation (Nav2)

```bash
ros2 launch hros5_navigation nav2.launch.py
```

---

## 📍 Notes

- Meshes are stored in `hros5_description/meshes/`, including both visual and collision STL/DAE files.
- A helper shell script was created to scaffold all packages with valid structure and sample nodes.

---

## 🛠️ Development Notes

This workspace is under active development.
- ROS 2 Control integration
- Hardware interface skeletons
- Gazebo/ignition simulation support

---

## 📄 License

MIT License (or specify yours)

---

## 👤 Author

- **Pengatom** – [@Pengatom](https://github.com/pengatom)

