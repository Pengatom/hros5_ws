# HR-OS5 System Architecture  
## Onboard (Pi 5) vs Offboard (Base Station) Node Split

This document defines **which ROS 2 nodes run onboard** the Raspberry Pi 5 and **which run offboard** on the base-station / desktop.  
The goal is to maintain a consistent pattern across **teleop, digital twin, IK, balance, and walking** phases.

This document does **not** implement the onboard bring-up (handled in Issue #60).  
Instead, it defines the architecture that all future bring-up and phase tasks must follow.

---

# 1. Overview

HR-OS5 uses a **distributed ROS 2 architecture**:

- The **Raspberry Pi 5** runs all **hardware-critical, safety-related, and low-latency nodes**.  
- The **base station** runs **visualization, planning, simulation, teleop frontends, and high-level behaviors**.

The system must remain safe and minimally operational **even if the base station disconnects**. Therefore, hardware and safety nodes must remain onboard.

---

# 2. Onboard Responsibilities (Raspberry Pi 5)

These nodes **must run on the Pi** to ensure deterministic timing, hardware safety, and standalone minimal usability.

## 2.1 Low-Level Hardware Interfaces

### Dynamixel Interfaces
- `hros5_dynamixel_bridge`
- Dynamixel bus access (USB2AX or equivalent)
- Actuator polling, status publication
- Low-level position/velocity/effort commands

### ros2_control
- `hros5_control`  
- `ros2_control_node` + Dynamixel hardware interface  
- Controller manager (joints for head/arms/hands/legs)

## 2.2 Directly Attached Sensors

### Camera (RealSense)
- RealSense driver node  
- Publishes RGB-D/stereo streams and camera info

### IMU
- Orientation, gyro, acceleration drivers

### Foot / Contact Sensors
- Force/pressure readings  
- Contact state publishers

## 2.3 Core Robot Model and Frames

### Robot description
- `robot_state_publisher` using `hros5.xacro` / URDF

### Frames maintained onboard
- `base_link`, torso, pelvis, feet  
- Head frames  
- Sensor frames (camera, IMU, etc.)

This ensures TF exists even in minimal offline mode.

## 2.4 Safety and Watchdog Nodes

- Hardware E-stop or soft-stop
- Torque disable logic
- Sensor/actuator health monitoring  
  - Overheat, comm errors, bus timeouts  
  - Stale sensor data  
- Emergency fallback actions (torque off, neutral pose)

These must remain operational **without the base station**.

## 2.5 Minimal Onboard Teleop Mode (Optional)

A small “bench-test” teleop node for:
- Moving a few joints
- Verifying servo wiring, offsets, limits  
- Running with **no network** and **no base station**

Useful for bring-up, debugging, and safe testing while robot is suspended.

---

# 3. Offboard Responsibilities (Base Station / Desktop)

These nodes are **not safety-critical**, may require GPU/CPU power, and primarily support the operator or higher-level reasoning.

## 3.1 Visualization & Monitoring Tools

- RViz2  
- rqt, rqt_plot, rqt_graph  
- Custom dashboards / diagnostics viewers  
- Long-term rosbag2 recording

## 3.2 Teleop Frontends

- PS4 controller node  
- Keyboard/web teleop UIs  
- High-level teleop commands (e.g., “look up”, “walk forward”, “gripper open”)

These nodes **never communicate with hardware directly** — they publish commands that onboard controllers execute.

## 3.3 High-Level Planning, IK, and Behaviors

- MoveIt2  
- Whole-body IK solvers  
- Trajectory generators  
- Gait planners, step planners  
- Coordination and behavior nodes (BT, HFSM, etc.)

These compute **desired trajectories**, which are executed on the Pi via ros2_control.

## 3.4 Simulation / Digital Twin

- Gazebo Harmonic simulation  
- Sim-side ros2_control adapters  
- Simulated sensor publishers  
- Perception stubs  
- MoveIt2 running against simulated robot

High-level code should work identically against **sim or real** hardware.

---

# 4. Nodes That *May* Run Either Side

Some nodes do not strictly belong to one machine; this section defines defaults.

## 4.1 Perception / Head Tracking / Vision Processing

Default: **Offboard**, unless:
- Latency requirements  
- Bandwidth constraints  
- CPU load on Pi becomes too high  

If eventual head-tracking servo loops require sub-10ms latency, parts may move onboard.

## 4.2 Command Routing / Muxing Nodes

If it influences **safety** — deciding which command source reaches the controllers:  
→ **Onboard**

If it only affects UX or non-critical features:  
→ **Offboard**

---

# 5. Launch Structure

To keep bring-up consistent across phases, HR-OS5 uses a layered launch convention.

## 5.1 Onboard Launches (Pi5)

### `hros5_control/launch/hros5_hardware.launch.py`  
(Implemented in Issue #60)

Starts:
- ros2_control + Dynamixel hardware  
- robot_state_publisher  
- onboard sensors  
- safety/watchdog nodes  

### `hros5_bringup/launch/onboard_minimal_teleop.launch.py`  
(Optional)
- Includes hardware launch  
- Adds a simple teleop node for bench testing

## 5.2 Offboard Launches (Desktop)

### `hros5_bringup/launch/offboard_desktop.launch.py`
- RViz2  
- PS4 teleop node  
- Diagnostics UIs  
- Logging tools

### Phase-specific offboard launches
- `p1_teleop_offboard.launch.py`  
- `p2_digital_twin_offboard.launch.py`  
- `p3_ik_offboard.launch.py`  
- `p4_balance_offboard.launch.py`  
- `p5_walking_offboard.launch.py`

Each assumes:
- Either real hardware is running via the Pi’s onboard launch, **or**  
- Gazebo twin is running in place of hardware.

---

# 6. Node Placement Summary Table

| Package / Node | Runs On | Notes |
|----------------|---------|-------|
| **hros5_control (ros2_control)** | **Onboard** | Hardware-critical |
| **hros5_dynamixel_bridge** | **Onboard** | Talks to USB2AX |
| **robot_state_publisher** | **Onboard** | Always available |
| **RealSense driver** | **Onboard** | Direct sensor |
| **IMU driver** | **Onboard** | Direct sensor |
| **Foot sensors** | **Onboard** | Safety + balance |
| **Safety/watchdog nodes** | **Onboard** | Must not depend on network |
| Minimal teleop | Onboard (optional) | Bench tests |
| PS4 teleop | Offboard | High-level commands only |
| RViz2 | Offboard | Developer tool |
| MoveIt2 | Offboard | Heavy CPU |
| IK solvers | Offboard | Heavy computation |
| Planning/gait nodes | Offboard | High-level behaviors |
| Gazebo Harmonic | Offboard | Full simulation |
| Head-tracking | Offboard (default) | Onboard only if latency-critical |

---

# 7. Requirements for Future Development

All future issues must follow these principles:

1. **Hardware-critical = onboard**  
2. **Operator tools = offboard**  
3. **High-level planning = offboard**  
4. **Safety = onboard**  
5. **Simulation = offboard**  
6. **Nodes must not mix responsibilities**  
   - e.g., a teleop node must not open the DXL port directly.
7. **Real robot and simulation must expose the same interfaces**  
   - High-level code should not care whether it talks to hardware or the Gazebo twin.

---

# 8. Relationship to Issues

- **Issue #60** implements the onboard hardware bring-up defined here.  
- Phase parent issues (P1–P6) must reference this document to ensure consistency across teleop, digital twin, IK, balance, and walking.

---

# 9. Conclusion

This architecture ensures:
- Deterministic and safe hardware behavior  
- Modular development  
- Identical API between simulation and real hardware  
- Clear distribution of responsibility between Pi and base station  
- Consistency across all HR-OS5 development phases  

All new packages, launch files, and behaviors must respect this split.
