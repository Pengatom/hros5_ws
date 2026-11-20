# hros5_gazebo

Gazebo simulation support package for HR-OS5.

Includes a full-robot Gazebo launch (`hros5_gazebo.launch.py`) and the original left-hand test environment.

## Directory structure

```text
hros5_gazebo/
├── CMakeLists.txt
├── config
│   ├── hros5_controllers.yaml
│   ├── sim_controllers.yaml
│   └── left_hand_controllers.yaml
├── launch
│   ├── hros5_gazebo.launch.py
│   ├── hros5_gazebo_teleop.launch.py
│   └── left_hand_gazebo.launch.py
├── package.xml
├── scripts
│   ├── reset_world_service.py
│   └── sim_dxl_bridge.py
├── urdf
│   ├── hros5_gazebo.xacro
│   └── hands
│       └── left_hand_gazebo.xacro
└── worlds
    └── empty.world
```

## Status (master, 2026-11-11)

Full-body simulation exists (P2: #34/#35/#36 closed). The torso IMU is wired to the world Imu system and publishes `/imu` (verified 2026-11-20, #37).
Still open: foot contact sensors (#38) and matching the real robot's interface (#64).
See `docs/STATUS.md` §8 for the 2026-11-11 bring-up fixes.

## Running the full robot

```bash
ros2 launch hros5_gazebo hros5_gazebo.launch.py                    # free standing on its feet
ros2 launch hros5_gazebo hros5_gazebo.launch.py fixed_base:=true spawn_z:=0.6   # hung on a "stand"
ros2 launch hros5_gazebo hros5_gazebo.launch.py launch_rqt_gui:=true # plus joint sliders
ros2 launch hros5_gazebo hros5_gazebo.launch.py headless:=true       # server only, no GUI (SSH/CI)
```

Verified on 2026-11-20: the robot stands, both controllers are active, `/imu` publishes, and trajectory goals (arms, legs, a symmetric squat) succeed. See `docs/STATUS.md` §8.

## PS4 teleop in simulation

```bash
ros2 launch hros5_gazebo hros5_gazebo_teleop.launch.py                   # whole robot: ps4_teleop_node (modes, hold L1)
ros2 launch hros5_gazebo hros5_gazebo_teleop.launch.py limb:=head        # one original per-limb node instead
ros2 launch hros5_gazebo hros5_gazebo_teleop.launch.py limb:=left_leg    # single legs start on a stand (fixed_base:=auto)
```

- **The launch:** starts Gazebo, `scripts/sim_dxl_bridge.py`, one `joy_node`, and either `ps4_teleop_node` (default) or the original teleop node for one limb.
  Controls are listed in the `hros5_teleop` README.
- **`sim_dxl_bridge.py`:** replaces the real `*_dxl_node` executables. It subscribes to the same `/hros5/<limb>/target_angles_deg` topics, applies the same servo hard limits and 2°-per-20 ms rate limit, and drives `whole_body_controller`.
- **`hros5_control/config/joint_directions.yaml`:** holds the servo-to-URDF sign per joint, with a confidence rating for each.
  In the URDF, left and right pitch joints use the same axis, while the servos are mounted mirrored.

See `docs/STATUS.md` §9.

Controllers: `joint_state_broadcaster` and `whole_body_controller` (JointTrajectoryController over all 24 joints,
topic `/whole_body_controller/joint_trajectory`). The torso IMU is bridged to `/imu`.
