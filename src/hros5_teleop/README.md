# hros5_teleop

PS4 controller teleop for HR-OS5.

## Whole robot: `ps4_teleop_node` (recommended)

```bash
ros2 launch hros5_teleop ps4_teleop.launch.py                        # with the real DXL nodes running
ros2 launch hros5_gazebo hros5_gazebo_teleop.launch.py               # in Gazebo (sim_dxl_bridge)
```

One `joy_node` and one node drive all limbs through modes (#27/#28). Configuration:

| File | Contents |
|------|----------|
| `config/ps4_mapping.yaml` | Axis/button indices, named actions, and the modes with their stick-to-joint bindings |
| `config/deadzones.yaml` | Stick and D-pad deadzones, `/joy` timeout |
| `config/speed_scales.yaml` | Speed per mode (deg/s at full stick), grip and home speed, guard band inside the servo limits |
| `hros5_control/config/joint_directions.yaml` | Servo-to-URDF sign per joint (shared with the Gazebo bridge) |
| `hros5_control/config/hros5_dynamixel_joints_with_limits.yaml` | Absolute servo limits; targets stay `limit_guard_deg` inside them |

### Controls (default mapping)

| Input | Action |
|-------|--------|
| **L1 (hold)** | Dead-man. Nothing moves unless it's held. No `/joy` for 0.5 s counts as released |
| Options / Share | Next / previous mode: head → left_arm → right_arm → hands → left_leg → right_leg → legs |
| Triangle (+ L1) | Move the current mode's joints back to 0° |
| Cross | Ask the DXL node(s) of the current mode to log servo positions |
| R1 / R2 (+ L1) | Close / open the gripper (arm modes) |

| Mode | Left stick | Right stick | D-pad |
|------|-----------|-------------|-------|
| head | yaw, pitch (up = look up) | — | — |
| left_arm / right_arm | shoulder roll, shoulder pitch (up = arm forward) | wrist, elbow (up = bend) | — |
| hands | left wrist, left grip | right wrist, right grip | — |
| left_leg / right_leg | hip roll, hip pitch (up = leg forward) | ankle roll, knee (down = bend) | hip yaw, ankle pitch |
| legs (both) | sideways hip shift, feet kept flat | down = squat, feet kept flat | — |

### Behaviour

- **Jogging:** sticks move joints at a speed proportional to the stick deflection, and the joints hold when you let go.
  Motion is in URDF terms, so the same stick motion is the same physical motion on the left and right side, even though the servos are mounted mirrored.
- **Starting position:** every joint starts at 0°, like the DXL nodes.
- **Hard limits:** joints driven by the same stick move as a group. If one would pass its hard limit, the whole group stops.
  This keeps the feet flat in a squat, which goes down to about hip 63° / knee 125° (the knee limit).
- **Home (Triangle):** all joints of the mode move in sync and arrive at 0 together, so the robot can stand up from a deep squat.
- **Topics:** publishes `/hros5/<limb>/target_angles_deg` for head, arms and legs (same layouts as below) and the current mode on `/hros5/teleop/mode`.
  Wrist and grip go out on the arm topics only, because the hand topics would give those servos a second owner on the bus.

## Per-limb nodes (original)

Each limb has its own node and launch file. Every launch file starts `joy_node` together with that limb's node.

| Node | Launch | Publishes (Float32MultiArray, degrees) | Echo request topic |
|------|--------|----------------------------------------|--------------------|
| `head_teleop_node` | `head_teleop.launch.py` | `/hros5/head/target_angles_deg` [pan, tilt] | `/hros5/head/echo_positions` |
| `left_arm_teleop_node` | `left_arm_teleop.launch.py` | `/hros5/left_arm/target_angles_deg` [sh_pitch, sh_roll, elbow, wrist, grip] | `/hros5/left_arm/echo_positions` |
| `right_arm_teleop_node` | `right_arm_teleop.launch.py` | `/hros5/right_arm/target_angles_deg` (same order) | `/hros5/right_arm/echo_positions` |
| `left_leg_teleop_node` | `left_leg_teleop.launch.py` | `/hros5/left_leg/target_angles_deg` [hip_yaw, hip_roll, hip_pitch, knee, ankle_pitch, ankle_roll] | `/hros5/left_leg/echo_positions` |
| `right_leg_teleop_node` | `right_leg_teleop.launch.py` | `/hros5/right_leg/target_angles_deg` (same order) | `/hros5/right_leg/echo_positions` |
| `hands_teleop_node` | `hands_teleop.launch.py` | `/hros5/left_hand/...`, `/hros5/right_hand/target_angles_deg` | — |

The matching consumers are the `*_dxl_node` executables in `hros5_dynamixel_bridge`.
In simulation, `hros5_gazebo/scripts/sim_dxl_bridge.py` consumes the same topics. Run any of these nodes against Gazebo with
`ros2 launch hros5_gazebo hros5_gazebo_teleop.launch.py limb:=<head|left_arm|right_arm|left_leg|right_leg|hands>`.

- **Head:** absolute mapping. Stick position = angle, so releasing the stick re-centres the head. `center_button` zeroes it, and `echo_position_button` asks the DXL node to log positions.
- **Arms:** joint jogging at `step_deg_per_sec`. Sticks only move joints while debug mode is on (`debug_toggle_button`).
  Grip buttons always work. D-pad up/down changes the speed, and D-pad left/right applies the home/ready presets.
- **Legs:** a stick deflection sets the joint angle and holds it when released. D-pad applies presets.
  There's no enable/dead-man button, so only use this with the robot on a stand.

## Status and caveats (2026-11-20)

- **Verified:**
  - On hardware: **head** with `head_teleop_node` (#17).
  - In Gazebo with a real PS4 controller: **head** with `head_teleop_node`.
  - In Gazebo with scripted `/joy`, `ps4_teleop_node` passes these checks: dead-man, mode switching, squat and sway stopping as a group at the limits, hold on `/joy` timeout, and home.
- **Not run on the real robot yet:** `ps4_teleop_node`. Before that, check the low/medium-confidence signs in `joint_directions.yaml`.
- **Per-limb nodes:**
  - They all listen to the same `/joy` and the same sticks, with no mode switching. Run **one** of their launches at a time.
  - Their `config/ps4_*.yaml` button comments partly don't match the Linux PS4 layout that `joy_node` reports (8 axes, 13 buttons: 0 Cross, 1 Circle, 2 Triangle, 3 Square, 4 L1, 5 R1, 6 L2, 7 R2, 8 Share, 9 Options).
    The axis indices match.
  - `ps4_right_arm_teleop.yaml` uses the left arm's joint limits. The DXL node clamps to the real servo limits.
- **Build files:** `CMakeLists.txt`/`package.xml` depend on `dynamixel_sdk`, but no node uses it.
