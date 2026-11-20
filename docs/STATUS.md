# HR-OS5 Current Status (master)

**Snapshot date:** 2026-11-11
**Basis:** source code on `master` plus the GitHub issues at `Pengatom/hros5_ws` (73 issues: 19 closed, 54 open).

The other docs (README, ProjectStructure, architecture) describe the **target design** for a fully walking robot.
This file tracks how far the code has come against that design. Anything described there but missing here is
**work to implement**, not a doc error.

**Current focus:** full-body robot in Gazebo (RViz already works). See §7.

---

## 1. Summary

| Area | Status |
|------|--------|
| Robot model (URDF/Xacro, meshes) | Exists, full body. Joint limits do not match hardware (see §4). |
| Dynamixel servo control | **Works per limb** through custom nodes (`*_dxl_node`). All servos on Protocol 2.0 (#57 closed). |
| PS4 teleop | **Head works** (#17 closed). Arm, leg and hand teleop nodes exist but can't safely run together (see §3). |
| ros2_control on real hardware | **Not wired up.** No hardware plugin and no `hros5_hardware.launch.py` (#60 open). |
| Gazebo | Full-body `hros5_gazebo.launch.py` and left-hand test exist. No IMU/contact sensors (#37/#38 open). |
| MoveIt 2 | Full-body config generated (`hros5.srdf` with head/arm/leg groups). Untuned (#40/#49 open). |
| Onboard/offboard split (Pi 5 / base station) | Documented only. No launch files implement it yet (#60/#63 open). |
| Bring-up launch files | None. Each limb is started with its own launch file. |
| Navigation, IK, balance, walking | Not started (P3–P6). `hros5_navigation` is `COLCON_IGNORE`d. |

---

## 2. What actually runs today

The working control path does **not** use ros2_control:

```text
PS4 ──► joy_node ──/joy──► <limb>_teleop_node ──/hros5/<limb>/target_angles_deg──► <limb>_dxl_node ──► /dev/dxl (USB2AX/U2D2)
                                              ──/hros5/<limb>/echo_positions────►   (logs present positions)
```

- Commands are `std_msgs/Float32MultiArray` in **degrees**, one array per limb.
- The DXL nodes step-limit motion (`max_step_deg` per 20 ms tick) and clamp to the servo tick limits from
  `hros5_control/config/joints/<limb>.yaml`. Because of that clamp, out-of-range teleop commands can't push a servo past its configured limit.
- Nothing publishes `/joint_states` on this path, so RViz can't show the real robot's pose.

### Launch files that exist per limb

| Limb | DXL node launch | Teleop launch |
|------|-----------------|---------------|
| Head | `hros5_dynamixel_bridge head_dxl.launch.py` | `hros5_teleop head_teleop.launch.py` |
| Left arm (incl. LWrist/LGrip) | `left_arm_dxl.launch.py` | `left_arm_teleop.launch.py` |
| Right arm (incl. RWrist/RGrip) | `right_arm_dxl.launch.py` | `right_arm_teleop.launch.py` |
| Left leg | `left_leg_dxl.launch.py` | `left_leg_teleop.launch.py` |
| Right leg | `right_leg_dxl.launch.py` | `right_leg_teleop.launch.py` |
| Hands (wrist + grip only) | *(no launch; `left/right_hand_dxl_node` executables)* | `hands_teleop.launch.py` |

Each teleop launch file starts its **own** `joy_node`.

---

## 3. Known problems that block full-body PS4 teleop

Ordered by impact:

1. **Every DXL node opens the same serial port.** All `*_dxl_node` processes open `/dev/dxl` on their own.
   If two of them run at once, two processes write to one half-duplex bus without coordination. That causes collisions,
   timeouts and "servos dropping off". This may be part of what #56 (intermittent servo connections) is seeing.
   Before more than one limb can run at a time, a **single bus owner** is needed.
2. **Servo ID overlap.** Wrists and grips (IDs 21–24) appear in both `joints/left_arm.yaml`/`right_arm.yaml` and
   `joints/hands.yaml`. If an arm node and a hand node run together, both command the same servos.
3. **All teleop nodes read the same sticks.** Head, arm and leg nodes all map the left stick (axes 0/1),
   so running them together moves several limbs at once. No mode selection or dead-man switch exists yet (#27/#28 open).
   Starting several teleop launches also creates duplicate `joy_node` nodes.
4. **The controller mapping doesn't match a single joystick driver.** Button comments follow the SDL2 layout
   (0 = Cross, 1 = Circle, 3 = Triangle), but axis indices follow the Linux joydev/hid-sony layout (3 = RX, 4 = RY, 6/7 = D-pad).
   `joy_node` from the Jazzy `joy` package uses SDL2, where axes 4/5 are the analog triggers and the D-pad is reported as buttons.
   Check every index with `ros2 topic echo /joy` before enabling arm or leg teleop.
   There are also button conflicts. For example, `left_arm` echo = button 1 (commented "Cross") and `right_arm` debug toggle = button 1 (Circle).
5. **Startup jump.** DXL nodes start with an internal position of 0° and don't read the present position first.
   On start, every joint drives toward 0° (up to ~100°/s, limited by `max_step_deg`). On legs this happens with torque on.
6. **No command timeout or e-stop.** If teleop or the network dies, the servos hold the last target with torque on.
   There's no torque-off button and no stale-command watchdog.
7. **Left/right arm teleop limits are copies.** `ps4_right_arm_teleop.yaml` uses the left arm's limits, but the right arm's
   hardware ranges are mirrored (for example, RElbowPitch hardware range is −101°..+146°, teleop −144°..+100°). The DXL clamp keeps this safe,
   but part of the right-arm range can't be reached from teleop.

### Build issues (not yet verified with `colcon build`)

- `hros5_dynamixel_bridge/CMakeLists.txt` installs a `description/` directory that doesn't exist in the package.
  CMake normally fails `colcon build` / install on a missing install directory.
- `hros5_teleop` depends on `dynamixel_sdk` even though no teleop source uses it. That breaks the "teleop never touches the DXL port" rule
  and forces the SDK onto the base station.
- `hros5_teleop/package.xml` has no `exec_depend` on `joy`.
- `src/dynamixel/` (vendor SDK and hardware interface) is gitignored. Install `ros-jazzy-dynamixel-sdk` from apt or clone it into `src/dynamixel/`.
- `/dev/dxl` is assumed everywhere, but no udev rule creating that symlink is in the repo.
- `dynamixel_bridge_node` (the generic bridge) and its `bringup.launch.py`/`rviz_bringup.launch.py` load `config/joints.yaml`,
  which doesn't exist. Treat them as legacy.
- `hros5_control/launch/bringup_with_control.launch.py` loads `config/dynamixel.yaml`, which doesn't exist, and
  `hros5_description/urdf/hros5_control.xacro` uses ROS 1 `<transmission>/<hardwareInterface>` syntax with no ros2_control hardware plugin.
  The ros2_control path on real hardware is therefore not functional yet.

---

## 4. Joint limit consistency

Results from `python3 src/hros5_utils/scripts/check_joint_limits.py --tolerance-deg 0.5` on 2026-11-11: **22 of 24 joints mismatch**.
Only the shoulder pitch joints agree across hardware, URDF, teleop and MoveIt.

- MoveIt copies the URDF limits, so it differs from hardware on every joint where the URDF does.
- Leg URDF limits are generic and symmetric (for example, knee ±160°), but hardware ranges are asymmetric (for example, LKneePitch −131.6°..+20°).
- Head URDF is ±70° yaw / −14°..+23° pitch. Head teleop allows ±90° / ±30°.

This is tracked by #66 and its children #68–#73. These issues block MoveIt/Gazebo parity, but **not** basic PS4 teleop,
because the DXL nodes clamp to hardware limits.

Note: `generate_joint_yamls.py --check` reports the joint YAMLs as stale on Windows. That's only because the `# Source:` header
is written with `\` separators there. Run it on Linux.

---

## 5. GitHub issues vs code

Some open issues are already partly or fully done in code:

| Issue | Says | Code on master |
|-------|------|----------------|
| #25 add hands_teleop_node | open | `hands_teleop_node` exists |
| #26 add arms_teleop_node | open | `left_arm_teleop_node` and `right_arm_teleop_node` exist (per arm, not combined) |
| #28 deadzones and speed scales | open | Per-node deadzone and speed params exist in each `ps4_*_teleop.yaml`, but there's no shared file |
| #29 head tracker output topic | open | Tracker and head teleop both publish `/hros5/head/target_angles_deg` (consistent, but they conflict if both run) |
| #12 create joint_limits.yaml | open | Not created. `hros5_dynamixel_joints_with_limits.yaml` is the de-facto source |
| #60 hros5_hardware.launch.py | open | Not created yet. Documented as the onboard bring-up, so it needs implementing |
| #30 hros5_minimal.launch.py | open | Not created |
| #56 intermittent servo connections | open | See §3 item 1. The shared serial port is a likely software contributor |

---

## 6. Path to PS4 teleop of the whole robot

Recommended order, smallest working step first:

1. **Environment.** Ubuntu 24.04 + ROS 2 Jazzy on the Pi 5. Install `ros-jazzy-dynamixel-sdk`, `ros-jazzy-joy`, `ros-jazzy-xacro`
   and `ros-jazzy-robot-state-publisher`. Add a udev rule for `/dev/dxl`. Pair the PS4 controller with `bluetoothctl`.
   Fix the bridge `CMakeLists.txt` install list, then `colcon build`.
2. **Head-only smoke test (works today).**
   `ros2 launch hros5_dynamixel_bridge head_dxl.launch.py` + `ros2 launch hros5_teleop head_teleop.launch.py`.
   Record the real `/joy` axis and button indices.
3. **One bus owner.** Replace the per-limb processes with a single `dxl_bus_node` that opens `/dev/dxl` once.
   It should subscribe to all `/hros5/<limb>/target_angles_deg` topics, use `GroupSyncWrite`/`GroupSyncRead`
   (already in `dynamixel_driver.cpp`), and publish `/joint_states`. Give each servo ID exactly one owner (resolves wrist/grip overlap).
   The longer-term alternative is a ros2_control hardware plugin (#60), which is bigger.
4. **One teleop node with modes** (#27/#28). Use one `joy_node` and one `ps4_mapping.yaml`. Select the mode with buttons (head / left arm / right arm / legs).
   Add a dead-man button that must be held for any motion.
5. **Safety basics.** Read present positions at startup and use them as the first target (no jump). Add a command timeout that holds or reduces torque,
   a torque-off button, and leg mode disabled unless explicitly enabled (robot on a stand).
6. **One launch file** (#30/#60). Onboard: bus node + `robot_state_publisher` (+ `joy_node` if the controller pairs with the Pi).
   Offboard (optional): RViz.
7. Then continue with limit alignment (#66) and diagnostics (#24).

Steps 1–2 can be done right away. Steps 3–6 are the code work needed before every limb can be driven from one PS4 controller.

---

## 7. Documented but not implemented (backlog)

| Documented in | Item | Status |
|---------------|------|--------|
| ProjectStructure, onboard_offboard (#60/#63) | `hros5_control/launch/hros5_hardware.launch.py` | Missing |
| ProjectStructure | `hros5_control/config/hardware_controllers.yaml`, `dynamixel.yaml`, `joint_limits.yaml` (#12), `safety_watchdog.yaml` | Missing |
| ProjectStructure | `hros5_dynamixel_bridge/config/joints.yaml` (generic bridge) | Missing |
| onboard_offboard | `hros5_bringup/launch/onboard_minimal_teleop.launch.py`, `offboard_desktop.launch.py`, `p1`–`p5` phase launches | Missing |
| #63 | `base_station.launch.py` | Missing |
| README | `hros5_navigation nav2.launch.py` (P6) | Missing; package is `COLCON_IGNORE`d |
| README (bringup) | `bringup_sim.launch.py`, `bringup_real.launch.py` | Missing |
| system_overview | Real robot and Gazebo expose the same ros2_control interface | Gazebo has it. Real hardware needs a ros2_control Dynamixel plugin |
| Legs (all phases) | Leg control in sim and on hardware | Sim: via `whole_body_controller`. Hardware: per-leg DXL nodes exist, but legs need the single bus owner (§3) |

---

## 8. Gazebo bring-up (step 1 toward walking)

Launch: `ros2 launch hros5_gazebo hros5_gazebo.launch.py` (add `fixed_base:=true` to hang the robot on a "stand").

Add `headless:=true` to run only the Gazebo server, without the GUI (for SSH sessions or machines without a GPU).

**Verified on 2026-11-20** (Ubuntu 24.04 VM, Jazzy, Gazebo Harmonic 8.10, headless):

- The build passes (`colcon build --packages-up-to hros5_gazebo`). `joint_state_broadcaster` and `whole_body_controller` both reach **active**.
- The robot lands on its feet at base z = 0.390 m and **stands upright** for 25+ s of sim time (roll/pitch < 0.01°).
- `/imu` publishes about 9.81 m/s² on z at rest. During a squat-and-lean move, roll, the gyro and acceleration respond (there's a short ~2 g spike when the robot settles), then it returns to level (#37).
  `/joint_states` lists all 24 joints.
- A `FollowJointTrajectory` goal with arms at ±0.8 rad and a squat (hip ∓0.4, knee ±0.8, ankle ∓0.4) **succeeds**. Base z drops to 0.371 m, the robot tilts 1.5° in roll, and it stays upright.
  Note that left and right leg pitch axes are mirrored: the same physical bend is +0.8 on `LKneePitch` and −0.8 on `RKneePitch`.
- On a real PC (12 cores, RTX 2060 SUPER on the nouveau driver) the same tests pass at a real-time factor of about 0.7–0.9, and the GUI runs.
  A VMware VM shows only a gray 3D view, because its virtual GPU can't run Gazebo's renderer.
- **Slow:** the real-time factor is only about 0.1–0.16 on the 2-core VM. The full STL mesh collisions are the likely cost; simpler collision shapes would help (backlog).

Changes made on 2026-11-11:

1. **Inertias fixed.** The torso, head, arm and leg inertias were Darwin-OP values that were 1000× too large
   (for example, torso ixx = 3.1 kg·m² for a 1 kg link). They're scaled to kg·m², matching the original Darwin-OP model. Real HR-OS5 values are still #32.
2. **Joints held at spawn.** `hold_joints` is set on `gz_ros2_control`, and controllers are started as soon as the robot is spawned
   (event chaining instead of fixed timers), so the robot no longer goes limp before the controllers are up.
3. **Spawn height.** The default `spawn_z` changed from 0.5 to 0.40. The foot soles are 0.389 m below `base_link`, so the robot drops ~1 cm instead of ~11 cm.
   The zero-pose centre of mass (x = 5 mm, y = −11 mm) is inside the foot support area (x −52..91 mm), so it should stand while joints are held.
4. **`fixed_base` option.** Pins `base_link` to the world for joint and controller testing without balance.
5. **World plugins.** Physics (1 ms step), UserCommands, SceneBroadcaster, Contact and Imu systems are declared explicitly, so the torso IMU publishes `/imu`.
   Foot friction is set to μ = 1.0.
6. **`package.xml`.** The Gazebo Classic `gazebo_ros` dependency (not in Jazzy) is replaced with `gz_ros2_control`, the controller packages and `ros_gz_*`.

Still open for Gazebo: foot contact sensors (#38), sim/real interface parity (#64), canonical joint limits in the URDF (#66/#72/#73), and real masses/inertias (#32).
