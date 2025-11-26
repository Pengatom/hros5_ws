---
name: HR-OS5 Full Task List
about: All roadmap tasks for HR-OS5 humanoid, grouped by package and phase
title: ["HR-OS5 Roadmap Master Issue (P0–P6)"]
labels: ["hr-os5"]
assignees: []
---

# �️ HR-OS5 TASK OVERVIEW

This issue contains **all project tasks**, organized by package and roadmap phase (P0–P6).  
When creating a new issue, simply copy the relevant section into a fresh ticket.

Roadmap:
- **P0** – Foundation / cleanup  
- **P1** – Full teleop  
- **P2** – Digital twin (Gazebo + MoveIt2)  
- **P3** – IK framework  
- **P4** – Balance & standing  
- **P5** – Walking  
- **P6** – Autonomous demo  

---

# � 1. `hros5_description` — URDF / Meshes / Xacro

### **P0**
- [ ] Verify `urdf/hros5.xacro` is top-level.
- [ ] Remove ros2_control elements from component xacros.
- [ ] Add missing visual meshes for hands & legs.
- [ ] Add collision meshes (hands, fingers, feet).

### **P1**
- [ ] Validate joint limits (pos/vel/effort) for every joint.

### **P2**
- [ ] Add inertias & masses for all links.
- [ ] Confirm frame naming compatibility with MoveIt groups.

---

# � 2. `hros5_control` — ros2_control, IK, balance

### **P0**
- [ ] Create `config/joint_limits.yaml`.

### **P1**
- [ ] Create/update `config/controllers.yaml`.
- [ ] Add `config/hardware_params.yaml` (IDs, ports, baud).

### **P3**
- [ ] Add IK module stubs:
  - [ ] `src/ik/arm_ik.cpp`
  - [ ] `src/ik/leg_ik.cpp`
  - [ ] `src/ik/whole_body_ik.cpp`
- [ ] Define IK interfaces (topics/services).
  
### **P4**
- [ ] Add balance framework:
  - [ ] `com_estimator.cpp`
  - [ ] `zmp_controller.cpp`
  - [ ] `ankle_stabilizer.cpp`

### **P5**
- [ ] Add walking/gait logic:
  - [ ] Step planner
  - [ ] Foot trajectories
  - [ ] Torso compensation module

---

# � 3. `hros5_dynamixel_bridge` — DXL hardware interface

### **P0**
- [ ] Create `dxl_bus.yaml` with port, baud, protocol.
- [ ] Split servo configs:
  - [ ] `head_servos.yaml`
  - [ ] `arm_servos.yaml`
  - [ ] `leg_servos.yaml`
- [ ] Ensure nodes load IDs/limits from YAML (remove hardcoded values).
- [ ] Move all dxl nodes here (head, hands, arms).

### **P1**
- [ ] Add diagnostics publisher:
  - [ ] Present position
  - [ ] Load
  - [ ] Temperature
  - [ ] Voltage

### **P1–P2**
- [ ] Extend servo scan tool to validate FW, limits, and errors.

---

# � 4. `hros5_teleop` — PS4 controller input & endpoint teleop

### **P0**
- [ ] Validate head teleop node + launch file.

### **P1**
- [ ] Implement `hands_teleop_node.cpp`.
- [ ] Implement `arms_teleop_node.cpp` (joint stepping).
- [ ] Create `ps4_mapping.yaml`.
- [ ] Create deadzone and scaling configs.

### **P3**
- [ ] Implement endpoint control:
  - [ ] `endpoint_control_node.cpp`
  - [ ] `endpoint_limits.yaml`
  - [ ] IK request interface to `hros5_control`

---

# � 5. `hros5_head_tracker_cpp`

### **P0**
- [ ] Fix and finalize `head_params.yaml`.

### **P1**
- [ ] Validate output topic: `/hros5/head/target_angles_deg`.

### **P2**
- [ ] Keep package strictly head tracking (move walking follow logic elsewhere).

---

# � 6. `hros5_perception` — future object tracking

### **P2**
- [ ] Create package structure.

### **P3**
- [ ] Define outputs:
  - [ ] `/hros5/target/object`
  - [ ] `/hros5/target/person`

---

# � 7. `hros5_gazebo` — simulation

### **P2**
- [ ] Create `hros5_gazebo.launch.py`.
- [ ] Add simulation controller config (`sim_controllers.yaml`).
- [ ] Add worlds:
  - [ ] `empty_room.world`
- [ ] Add IMU plugin.
- [ ] Add foot contact sensor plugins.

---

# � 8. `hros5_moveit_config`

### **P2**
- [ ] Run MoveIt Setup Assistant for initial config.
- [ ] Create MoveIt groups:
  - [ ] `left_arm`
  - [ ] `right_arm`
  - [ ] `legs`
  - [ ] `whole_body`
  - [ ] `head`

### **P3**
- [ ] Tune `kinematics.yaml` (IK solvers).
- [ ] Sync joint limits with `hros5_control`.

---

# � 9. `hros5_behaviors` — walking, balance, high-level logic

### **P3**
- [ ] Create package skeleton with:
  - [ ] `src/walking/`
  - [ ] `src/behaviors/`

### **P4**
- [ ] Integrate balance modules:
  - [ ] COM estimator
  - [ ] ZMP
  - [ ] Torso compensation

### **P5**
- [ ] Walking engine:
  - [ ] Gait manager
  - [ ] Step planner
  - [ ] Walking state machine

### **P6**
- [ ] Autonomous behaviors:
  - [ ] walk_to_point
  - [ ] person_follow
  - [ ] demo scripts

---

# � 10. `hros5_bringup` — turn-key robot startup

### **P1**
- [ ] `hros5_minimal.launch.py` (head + hands + teleop).

### **P2**
- [ ] `hros5_simulation_full.launch.py`.

### **P3–P4**
- [ ] `hros5_full_hardware.launch.py`.

### **P6**
- [ ] `hros5_autonomous_demo.launch.py`.

---

# � 11. `hros5_utils` — tooling

### **P0**
- [ ] Add servo scan, limit readers, debug scripts.
- [ ] Add `make_compile_commands.sh`.

### **P1–P2**
- [ ] Add joint state recording/reporting tooling.

---

# ✔ When turning a section into its own issue:

### Fill out this checklist:

- [ ] Package: `hros5_<package>`
- [ ] Phase: `P?`
- [ ] Tasks copied from this master list
- [ ] Acceptance criteria defined
- [ ] Test notes included
- [ ] Linked from `ProjectStructure.md`

---

# � Notes

This file is the **master task registry**.  
Create *individual issues* by copying the relevant bullets into new tickets.

