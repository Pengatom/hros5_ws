# HR-OS5 Project Structure
**Workspace:** `~/hros5_ws/`  

This document describes the current package layout of the HR-OS5 workspace and how it maps to the long-term roadmap (teleop → simulation → IK → walking → Nav2).

Roadmap phase tags:
- **[P0]** Foundation / cleanup  
- **[P1]** Teleop / manual control  
- **[P2]** Digital twin (Gazebo + MoveIt2)  
- **[P3]** IK framework  
- **[P4]** Balance & standing  
- **[P5]** Walking  
- **[P6]** Autonomous / Nav2 integration  

---

## Workspace Layout (current)

```text
hros5_ws/
└─ src/
   ├─ dynamixel/
   │  ├─ dynamixel_hardware_interface/   (vendor)
   │  ├─ dynamixel_interfaces/           (vendor)
   │  └─ DynamixelSDK/                   (vendor SDK)
   ├─ hros5_bringup/
   ├─ hros5_control/
   ├─ hros5_demos/
   ├─ hros5_description/
   ├─ hros5_dynamixel_bridge/
   ├─ hros5_gazebo/
   ├─ hros5_head_tracker_cpp/
   ├─ hros5_moveit_config/
   ├─ hros5_navigation/                  
   ├─ hros5_teleop/
   ├─ dynamixel.model
   └─ mx28at2.model
Vendor/third-party code lives under src/dynamixel/.
All HR-OS5 ROS 2 packages are prefixed hros5_….

1. hros5_description
Robot model: URDF/Xacro, meshes, RViz configs

Purpose
Defines the geometry, kinematics, and basic semantics of the full robot for RViz, Gazebo and MoveIt2.

Structure (key parts)
text
Copy code
hros5_description/
├─ launch/
│  ├─ display_arms.launch.py
│  ├─ display_hands.launch.py
│  ├─ display_hros5.launch.py
│  ├─ display.launch.py
│  ├─ display_left_hand.launch.py
│  └─ test_dynamixel.launch.py
├─ meshes/
│  ├─ arms/      # shoulders, upper/lower arms
│  ├─ hands/     # forearms, hands, grippers, FR07 parts
│  ├─ head/      # Upper/Lower head, motor covers
│  ├─ legs/      # hip, thigh, tibia, ankle, feet
│  └─ torso/     # torso shell
├─ rviz/
│  ├─ arms.rviz
│  ├─ hros5.rviz
│  └─ left_hand.rviz
├─ src/
│  └─ hros5_description_node.cpp
└─ urdf/
   ├─ arms/
   │  ├─ arms.xacro
   │  └─ arms_test.xacro
   ├─ base/
   │  └─ base.xacro
   ├─ hands/
   │  ├─ hands.xacro
   │  ├─ hands_control.xacro
   │  ├─ hands_test.xacro
   │  ├─ left_hand.xacro
   │  └─ left_hand_test.xacro
   ├─ head/
   │  └─ head.xacro
   ├─ legs/
   │  └─ legs.xacro
   ├─ torso/
   │  └─ torso.xacro
   ├─ robot/
   │  └─ robot.xacro
   ├─ hros5.xacro                     # top-level model
   ├─ hros5.urdf                      # generated URDF (export)
   ├─ hros5_control.xacro             # ros2_control tags
   ├─ hros5_gazebo.xacro              # Gazebo plugins/hooks
   ├─ hros5_visuals_collisions_endoskeleton.xacro
   └─ test_hand_control.urdf.xacro
TODOs
[P0] Keep component xacros (head/arms/hands/legs/torso/base) focused on kinematics; keep ros2_control bits in hros5_control.xacro.

[P0] Ensure all STL/DAE meshes used by hros5_visuals_collisions_endoskeleton.xacro are correct and up to date.

[P1] Verify <limit> tags for all joints match Dynamixel safe ranges.

[P2] Confirm inertias/masses are reasonable for Gazebo; adjust if sim is unstable.

2. hros5_control
Controllers, joint limits, ros2_control configs

Purpose
Provides controller configuration for ros2_control and joint limits for the whole robot. Currently focused on position/trajectory controllers for arms and legs.

Structure (key parts)
text
Copy code
hros5_control/
├─ config/
│  ├─ hardware_controllers.yaml
│  ├─ dynamixel.yaml
│  ├─ hand_test_controllers.yaml
│  ├─ hros5_dynamixel_joints_with_limits.yaml
│  ├─ joint_limits.yaml
│  ├─ left_arm_position_controller.yaml
│  ├─ left_leg_controller.yaml
│  ├─ position_controllers.yaml
│  ├─ right_arm_position_controller.yaml
│  ├─ right_leg_controller.yaml
│  ├─ safety_watchdog.yaml
│  └─ trajectory_controllers.yaml
├─ launch/
│  ├─ hros5_hardware.launch.py          # canonical onboard bring-up (Issue #63)
│  ├─ bringup_with_control.launch.py
│  ├─ launch_leg_trajectory_control.py
│  ├─ left_leg_control.launch.py
│  ├─ position.launch.py
│  └─ trajectory.launch.py
└─ package.xml / CMakeLists.txt
TODOs
[P0] Treat hros5_dynamixel_joints_with_limits.yaml as the single source of truth for limits and keep it in sync with URDF.

[P1] Consolidate controller configs into a clear structure, e.g.:

controllers_position.yaml

controllers_trajectory.yaml

[P2] Add separate configs for sim vs hardware if needed.

[P3+] Later add IK/balance code in a dedicated library package (or a new package like hros5_control_ik) instead of overloading configs.

3. hros5_dynamixel_bridge
Dynamixel SDK wrapper and body-part-specific nodes

Purpose
Encapsulates all communication with Dynamixel servos: bus setup, reading/writing positions, etc. Exposes per-body-part nodes and a more generic bridge node.

Structure (key parts)
text
Copy code
hros5_dynamixel_bridge/
├─ config/
│  └─ joints.yaml                # joint name ↔ ID mapping
├─ include/hros5_dynamixel_bridge/
│  └─ dynamixel_driver.hpp
├─ launch/
│  ├─ bringup.launch.py
│  └─ rviz_bringup.launch.py
├─ src/
│  ├─ dynamixel_bridge_node.cpp  # generic
│  ├─ dynamixel_driver.cpp       # low-level bus logic
│  ├─ head_dxl_node.cpp
│  ├─ left_arm_dxl_node.cpp
│  ├─ left_hand_dxl_node.cpp
│  ├─ right_arm_dxl_node.cpp
│  └─ right_hand_dxl_node.cpp
└─ urdf/
   └─ hros5_minimal.urdf.xacro   # minimal model for tests
TODOs
[P0] Ensure all DXL nodes read IDs/limits from config/joints.yaml (no hardcoded IDs).

[P0] Keep dynamixel_driver.cpp as the only place that touches the SDK (one abstraction layer).

[P1] Add diagnostic topics (load, temperature, voltage, error flags).

[P1–P2] Extend tools (e.g. servo scanner, limit reader) using this package’s config.

4. hros5_teleop
PS4 teleop for head, hands, left arm, right arm

Purpose
Converts joystick input into ROS messages for head, hands, and arms. This is your main manual control entry point.

Structure
text
Copy code
hros5_teleop/
├─ config/
│  ├─ ps4_hands_teleop.yaml
│  ├─ ps4_head_teleop.yaml
│  ├─ ps4_left_arm_teleop.yaml
│  └─ ps4_right_arm_teleop.yaml
├─ launch/
│  ├─ hands_teleop.launch.py
│  ├─ head_teleop.launch.py
│  ├─ left_arm_teleop.launch.py
│  └─ right_arm_teleop.launch.py
└─ src/
   ├─ hands_teleop_node.cpp
   ├─ head_teleop_node.cpp
   ├─ left_arm_teleop_node.cpp
   └─ right_arm_teleop_node.cpp
TODOs
[P1] Standardise PS4 mapping across all nodes (consistent sign conventions and button usage).

[P1] Document which topics each teleop node publishes to (e.g. /hros5/head/target_angles_deg, hand/arm topics, etc.).

[P3] Later: add endpoint/IK teleop node that converts stick motion into end-effector pose commands instead of raw joint steps.

5. hros5_head_tracker_cpp
Head camera tracking & ROI selection

Purpose
Reads camera input, lets you select an ROI, and publishes target angles for the pan/tilt head so the camera can follow a target.

Structure
text
Copy code
hros5_head_tracker_cpp/
├─ config/
│  └─ head_params.yaml
├─ launch/
│  └─ head_tracking.launch.py
└─ src/
   ├─ opencv_window_test.cpp
   └─ tracker_node.cpp
TODOs
[P0] Keep head_params.yaml fully in sync with what tracker_node expects (no “magical” defaults).

[P1] Confirm that the node publishes on the same topic + units used by the head teleop/DXL logic.

[P2+] Leave high-level behaviours (like “walk while looking at X”) to a separate behaviors layer later.

6. hros5_gazebo
Gazebo testbed (currently focused on left hand)

Purpose
Provides a minimal Gazebo world and config to simulate part of the robot (currently the left hand).

Structure
text
Copy code
hros5_gazebo/
├─ config/
│  └─ left_hand_controllers.yaml
├─ launch/
│  └─ left_hand_gazebo.launch.py
├─ urdf/
│  └─ hands/
│     └─ left_hand_gazebo.xacro
└─ worlds/
   └─ empty.world
TODOs
[P2] Gradually extend from left hand only to full-body simulation:

Add a full-robot Gazebo launch (using hros5_description’s hros5_gazebo.xacro).

Add sim controllers for head, arms, legs.

[P2] Keep physics tuning here (e.g. damping, friction configs).

7. hros5_moveit_config
MoveIt2 configuration (currently hands test)

Purpose
MoveIt2 config package currently targeted at a hands-only test robot.

Structure
text
Copy code
hros5_moveit_config/
├─ config/
│  ├─ hands_test.ros2_control.xacro
│  ├─ hands_test.srdf
│  ├─ hands_test.urdf.xacro
│  ├─ initial_positions.yaml
│  ├─ joint_limits.yaml
│  ├─ kinematics.yaml
│  ├─ moveit_controllers.yaml
│  ├─ moveit.rviz
│  ├─ pilz_cartesian_limits.yaml
│  └─ ros2_controllers.yaml
└─ launch/
   ├─ demo.launch.py
   ├─ move_group.launch.py
   ├─ moveit_rviz.launch.py
   ├─ rsp.launch.py
   ├─ setup_assistant.launch.py
   ├─ spawn_controllers.launch.py
   ├─ static_virtual_joint_tfs.launch.py
   └─ warehouse_db.launch.py
TODOs
[P2] Decide whether:

To keep this as a hands-only MoveIt sandbox, and

Later create a separate full-body MoveIt config package (e.g. hros5_moveit_full_config), or

Extend this package in place to include the entire robot.

[P2–P3] Align joint_limits.yaml with hros5_control and hros5_description.

8. hros5_demos
Small test packages for servos & URDF

Purpose
Contains one-off or focused demos for Dynamixel tests and URDF experiments.

Structure
text
Copy code
hros5_demos/
├─ config/
│  ├─ dynamixel_controllers.yaml
│  ├─ dynamixel_test_controllers.yaml
│  └─ two_mx28.yaml
├─ launch/
│  ├─ test_dynamixel.launch.py
│  └─ test_two_servos.launch.py
└─ urdf/
   ├─ test_robot.urdf.xacro
   ├─ test_urdf.xacro
   └─ test.urdf
TODOs
[P0–P1] Keep this as your safe playground for new controller configs and robot snippets before they are promoted into main packages.

[P2+] Optionally document each demo in a short README.md.

9. hros5_bringup
High-level bringup for RViz/Gazebo tests

Purpose
Provides combined launch files to bring up common subsets of the stack (especially RViz and Gazebo for quick testing).

Structure
text
Copy code
hros5_bringup/
├─ launch/
│  ├─ gazebo_launch.py
│  ├─ hros5_rviz_launch.py
│  ├─ rviz_launch.py
│  └─ test_hand_control.launch.py
└─ src/
   └─ hros5_bringup_node.cpp
TODOs
[P1] Define a minimal “daily dev” bringup (e.g. head + hands + teleop + DXL bridge + RViz).

[P2] Add a full simulation bringup (Gazebo + controllers + MoveIt + teleop).

[P3–P4] Add a full hardware bringup that includes everything except future Nav2.

10. hros5_navigation (Nav2 – future)
Future Nav2 integration package (COLCON_IGNORE for now)

Purpose
Reserved for Nav2-based navigation (maps, costmaps, path planning, behavior trees). This is separate from low-level walking/balance; Nav2 will eventually “ask” HR-OS5 to walk to a pose.

Structure
text
Copy code
hros5_navigation/
├─ COLCON_IGNORE          # intentionally disabled for now
├─ src/
│  └─ hros5_navigation_node.cpp
└─ CMakeLists.txt / package.xml
Plan
[P6] When you reach Nav2 work:

Add typical Nav2 configs:

maps, costmaps, behavior trees, nav params.

Connect Nav2 output (target base poses) to your walking/balance stack.

11. External Dynamixel Code
text
Copy code
src/dynamixel/
├─ DynamixelSDK/            # Official SDK (C/C++/Python/LabVIEW/etc.)
├─ dynamixel_hardware_interface/  (vendor)
└─ dynamixel_interfaces/          (vendor)
These provide SDKs and example interfaces. Your actual hardware integration is done through hros5_dynamixel_bridge.

Planned Future Packages (not yet created)
These are not in the repo yet but are part of the long-term architecture:

hros5_behaviors – walking, balance, high-level behaviors

hros5_perception – object/person tracking, 3D perception

hros5_utils – shared tools/scripts (servo scanners, reports, etc.)

They’ll be added when you reach those phases (P3–P6).

✔ Immediate Focus (P0–P1)
From this structure, the most important near-term tasks are:

Keep hros5_description + hros5_control limits consistent.

Solidify hros5_dynamixel_bridge as your single DXL interface.

Standardize PS4 teleop mappings in hros5_teleop.

Define a reliable “dev bringup” launch in hros5_bringup.

Keep hros5_navigation parked for later Nav2 work (no need to move it).
