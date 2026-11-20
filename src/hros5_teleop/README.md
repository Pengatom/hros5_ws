# hros5_teleop

PS4 controller teleop for HR-OS5. Each limb has its own node and launch file. Every launch file starts `joy_node` together with that limb's node.

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

## Behaviour

- **Head:** absolute mapping. Stick position = angle, so releasing the stick re-centres the head. `center_button` zeroes it, and `echo_position_button` asks the DXL node to log positions.
- **Arms:** joint jogging at `step_deg_per_sec`. Sticks only move joints while debug mode is on (`debug_toggle_button`).
  Grip buttons always work. D-pad up/down changes the speed, and D-pad left/right applies the home/ready presets.
- **Legs:** a stick deflection sets the joint angle and holds it when released. D-pad applies presets.
  There's no enable/dead-man button, so only use this with the robot on a stand.

## Status and caveats (master, 2026-11-11)

- Verified on hardware: **head** (#17). Verified in Gazebo with a real PS4 controller: **head** (2026-11-20).
- All nodes listen to the same `/joy` and the same sticks, with no mode switching.
  Run **one** teleop launch at a time. Running two also creates duplicate `joy_node` instances.
- The axis/button indices in `config/ps4_*.yaml` mix two layouts: button comments follow SDL2, axis indices follow Linux joydev.
  `joy_node` (Jazzy) uses SDL2. Verify every index with `ros2 topic echo /joy` before using arm or leg teleop.
- `ps4_right_arm_teleop.yaml` uses the left arm's joint limits. The DXL node clamps to the real servo limits.
- `CMakeLists.txt`/`package.xml` depend on `dynamixel_sdk`, but no node uses it. `package.xml` is missing `exec_depend` on `joy`.
- Planned (#27/#28): one `ps4_mapping.yaml`, a single teleop node with modes, and a dead-man button. See `docs/STATUS.md` §6.
