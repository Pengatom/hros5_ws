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
│   └── left_hand_gazebo.launch.py
├── package.xml
├── scripts
│   └── reset_world_service.py
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

Verified on 2026-11-20: the robot stands, both controllers are active, `/imu` publishes, and trajectory goals (arms, squat) succeed. See `docs/STATUS.md` §8.

Controllers: `joint_state_broadcaster` and `whole_body_controller` (JointTrajectoryController over all 24 joints,
topic `/whole_body_controller/joint_trajectory`). The torso IMU is bridged to `/imu`.
