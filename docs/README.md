# HR-OS5 Documentation  
## Architecture • Bringup • Model • Development Workflow

Welcome to the documentation hub for the HR-OS5 humanoid robot.  
This directory contains system-level design documents that describe **how the software stack works**, **how the robot is brought up**, and **how the simulation and hardware systems stay consistent**.

These documents support all development phases (P0–P6):  
Foundation → Teleop → Digital Twin → IK → Balance → Walking → Autonomy.

---

# 1. Project Structure

High-level map of packages, roadmap phases, and key TODOs for each package.

📄 **[`ProjectStructure.md`](ProjectStructure.md)**

---

# 2. Architecture Documents

These files define the overall structure of the HR-OS5 software stack and where components run.

### **System Overview**
High-level description of the entire robot software system:
- Package roles  
- Data flow (real + simulation)  
- TF, ros2_control, MoveIt2 interactions  
- How bring-up and simulation fit together  

📄 **[`system_overview.md`](architecture/system_overview.md)**

---

### **Onboard vs Offboard Responsibilities**
Defines the split between:
- Raspberry Pi 5 (hardware, safety, sensors, ros2_control)  
- Base station (RViz, teleop, planning, simulation)  

Used by: bring-up flow, teleop, digital twin, IK, balance, walking.

📄 **[`onboard_offboard.md`](architecture/onboard_offboard.md)**

---

# 3. Bring-up and Runtime Flow

### **Bring-up Flow (upcoming)**  
Will document:
- Real hardware startup sequence  
- Desktop/offboard launch structure  
- Gazebo digital twin startup  
- Connections between onboard and offboard nodes  
- Phase-specific launch layering (P1–P6)

📄 **`bringup_flow.md` (to be created after Issue #60)**

---

# 4. Robot Model Structure

### **URDF/Xacro Structure (upcoming)**  
Will describe:
- File hierarchy in `hros5_description/urdf/`  
- Include chain for `hros5.xacro`  
- Visual/collision mesh structure  
- Naming conventions (links/joints)  
- Where ros2_control integrates into the model  

📄 **`xacro_model_structure.md` (to be created after model stabilizes)**

---

# 5. Development Workflow and Conventions

(Optional future documents)
- Contribution guidelines  
- Coding & packaging standards (ROS 2 Jazzy)  
- Launch naming conventions  
- Simulation vs hardware testing workflow  
- Phase roadmap with references to GitHub Issues

---

# 6. Directory Structure

Intended documentation layout:

```text
docs/
├── ProjectStructure.md           ← Workspace/package map + roadmap tags
├── README.md ← You are here
├── architecture/
│   ├── system_overview.md        ← High-level system architecture
│   ├── onboard_offboard.md       ← Pi vs base-station node split
│   ├── bringup_flow.md           ← (future) real/sim startup flow
│   └── xacro_model_structure.md  ← (future) URDF/Xacro documentation
```

---

# 7. Usage Notes

- Architecture documents are referenced from GitHub Issues across P0–P6.
- All new packages, nodes, and launches must follow the onboard/offboard split.
- The documentation evolves as major milestones are completed (Issue #60, digital twin setup, full URDF mesh integration, etc.).
- Contributors should start with:  
  👉 **`system_overview.md`**  
  then  
  👉 **`onboard_offboard.md`**

---
