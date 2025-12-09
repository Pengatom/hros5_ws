# HR-OS5 System Overview  
## Software Architecture, Packages, and Data Flow

This document provides a high-level overview of the HR-OS5 humanoid robot software stack.  
It explains how the major ROS 2 packages interact, how data flows through the system, and how real hardware and simulation share a unified architecture.

This overview defines *what the system is*, while `onboard_offboard.md` defines *where the system runs*.

---

# 1. System Goals

The HR-OS5 software stack supports:

- **Real hardware control** using Dynamixel MX-series servos  
- **High-fidelity simulation** (digital twin) using Gazebo Harmonic  
- **Operator teleoperation** using PS4 controller or UI tools  
- **Motion planning** with MoveIt2 (arms, legs, head)  
- **Whole-body control** (IK, balance, walking)  
- **Unified topics, services, and TFs** between real robot and simulation  

The full stack is modular so new subsystems (perception, behaviors, autonomy) can be added without breaking core control.

---

# 2. High-Level Architecture

The system is split across two machines:

- **Onboard (Raspberry Pi 5)** → hardware, ros2_control, safety, sensors  
- **Offboard (Base station/desktop)** → visualization, teleop, simulation, planning  

Both machines run ROS 2 nodes and communicate over DDS.  
Simulation follows the exact same topic and controller interfaces so planning and teleop nodes work identically for real and simulated robots.

---

# 3. Major Packages and Their Roles

This section describes each primary HR-OS5 package and its function in the system.

## 3.1 `hros5_description`

- Contains the **URDF/Xacro model** of the robot  
- Defines:
  - Links, joints, coordinate frames  
  - Visual and collision geometry  
  - Kinematic chains and naming conventions  
  - ros2_control configuration Xacros  
- Used by:
  - `robot_state_publisher` (real robot)  
  - Gazebo Harmonic (simulation)  
  - MoveIt2 (planning)  
  - RViz (visualization)

## 3.2 `hros5_control`

- Hosts the **ros2_control** stack and hardware interface integration  
- Includes:
  - `hros5_control.xacro` for control plugins  
  - Controller configuration YAMLs  
  - Onboard bring-up (Issue #60)  
- Manages:
  - Joint state interfaces  
  - Command interfaces  
  - Controller lifecycle  
- Runs exclusively **onboard** with real hardware, or inside Gazebo in simulation.

## 3.3 `hros5_dynamixel_bridge`

- Low-level interface to Dynamixel MX-series servos  
- Responsible for:
  - Opening USB2AX serial connection  
  - Reading state (position, velocity, load, temperature)  
  - Sending position/velocity/torque commands  
  - Publishing diagnostics  
- Serves as the hardware backend for ros2_control

## 3.4 `hros5_bringup`

- Provides **system-level launch files** for:  
  - Real hardware bring-up (onboard)  
  - Desktop visualization and teleop  
  - Digital twin / simulation startup  
  - Phase-specific launches (teleop, IK, balance, walking)  
- Centralizes all "start the system" logic and orchestrates cross-package flows.

## 3.5 `hros5_head_tracker_cpp`

- Perception-assisted head control  
- Subscribes to camera topics  
- Publishes high-level target angles for head servos  
- Runs **offboard by default**, but may run onboard if latency becomes critical

## 3.6 `hros5_teleop`

- Teleoperation nodes for:
  - PS4 controller  
  - Keyboard / GUI / future devices  
- Converts user input into consistent high-level command topics understood by ros2_control and behavior nodes

## 3.7 Optional or Future Packages

- `hros5_behaviors` → state machines, behavior trees, autonomous skills  
- `hros5_navigation` → torso/leg planning, locomotion  
- `hros5_perception` → SLAM, object detection, future onboard/offboard perception pipelines  

---

# 4. Data Flow Overview

This section summarizes how the robot’s real-time data moves through the system.

## 4.1 Real Robot (Onboard Pi 5)

**Sensors → Hardware interface → ros2_control → TF + joint states → Offboard nodes**

1. Sensors (camera/IMU/feet) publish data  
2. `hros5_dynamixel_bridge` reads servo states  
3. `ros2_control_node` exposes joint_state and command interfaces  
4. `robot_state_publisher` publishes TF  
5. Offboard nodes (teleop, IK, MoveIt2) send trajectories  
6. ros2_control executes the trajectories using actuators  
7. Safety nodes can override or torque-off at any time

## 4.2 Simulation (Gazebo Harmonic)

**Gazebo plugins → simulated ros2_control → TF + joint states → Offboard nodes**

Same architecture as real hardware:
- Simulated joints replace real servos  
- Simulated sensors replace physical sensors  
- Everything else (MoveIt2, teleop, planning) stays identical

This ensures a seamless switch between real robot and digital twin.

---

# 5. Unified Control Interface

Regardless of real hardware or simulation:

- The same topics are used for head, arm, hand, and leg commands  
- MoveIt2 sees the same kinematic chains  
- Teleop nodes publish to the same command interfaces  
- ros2_control manages joint lifecycle, states, safety, and execution  

This allows:

- Development without hardware present  
- Consistent debugging  
- Deterministic testing in simulation before deploying on the robot

---

# 6. Launch Architecture Summary

The system uses a layered launch design:

### Onboard (Pi 5)
- Hardware bring-up (`hros5_hardware.launch.py`)  
- robot_state_publisher  
- ros2_control  
- Sensors  
- Safety + watchdog nodes  
- Optional minimal teleop

### Offboard (desktop)
- RViz2  
- PS4 teleop nodes  
- Visualization tools  
- MoveIt2 planning  
- Digital twin / Gazebo  
- High-level behaviors

### Phase launches
- P1: teleop  
- P2: digital twin  
- P3: IK  
- P4: balance  
- P5: walking  
- P6: autonomy  

All reuse the onboard/offboard split defined in `onboard_offboard.md`.

---

# 7. Key Principles for All Development

1. **Hardware-critical code always runs onboard**  
2. **High-level computational tasks run offboard**  
3. **Simulation mirrors real hardware interfaces exactly**  
4. **Safety nodes must not depend on network availability**  
5. **Launch files must be layered and modular**  
6. **URDF/Xacro model must remain authoritative**  
7. **Nodes must not combine hardware access with teleop or planning logic**  

---

# 8. Relationship to Other Architecture Documents

| Document | Purpose |
|---------|---------|
| `system_overview.md` | High-level view of the whole software stack |
| `onboard_offboard.md` | Defines where nodes run (Pi vs desktop) |
| `bringup_flow.md` | Will define the exact real/sim startup sequence |
| `xacro_model_structure.md` | Will detail the robot model file hierarchy |

These documents together describe:
- The system  
- Where each node belongs  
- How to start the system  
- How the robot model is organized  

---

# 9. Conclusion

The HR-OS5 software architecture is modular, layered, and simulation-aware.  
It supports safe real hardware operation, high-performance offboard computation, and a fully functional digital twin.  

All development phases (teleop → digital twin → IK → balance → walking → autonomy) build on the structure defined in this document.

