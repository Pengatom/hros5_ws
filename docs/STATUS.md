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

**Rule (2026-11-20):** the servo EEPROM limits in `hros5_control/config/hros5_dynamixel_joints_with_limits.yaml` are the absolute
maximum. Every other limit must be at least **2° inside** them. Limits that are narrower are allowed.

```bash
source install/setup.bash && python3 src/hros5_utils/scripts/check_joint_limits.py [--propose]
```

**How the checker compares limits:**
- **Teleop and head tracker configs** are in servo degrees and are compared directly.
- **URDF (from the xacro) and MoveIt** are compared after converting with `hros5_control/config/joint_directions.yaml`.
- **Low-confidence directions** (head yaw, wrists, grips, hip yaw, ankle roll) only get the part of the range that is valid with either sign.
- **MoveIt** must also equal the URDF. The checker exits non-zero on any violation.

**Before (2026-11-20):** 21 joints had a limit outside, for example:
- **URDF knees** ±160°, hip pitch ±90°, hip roll ±45° (the servos allow only ~5° inwards) and shoulder pitch ±180°.
- **Head teleop** ±90° / ±30°, and **head tracker** tilt up to +36° (servo +35.3°).
- **Leg teleop:** knee ±70°.
- **Right arm/hand teleop:** used the left grip's range (−58°, but the RGrip servo stops at −35.9°).

**Now:** the checker passes ("All limits are inside the servo hard limits (minus 2 deg)"). Changed:
- **URDF** (`legs.xacro`, `arms.xacro`, `hands.xacro`, `left_hand.xacro`): hip yaw ±19.8°, hip roll −2.9..+45° (L) / −45..+2.9° (R),
  hip pitch −88..+31.8°, knee −18..+125.6°, ankle pitch −16..+88°, shoulder pitch ±177.9°, wrist ±129.8°,
  grip −30.7..+28.6° (L) / −33.9..+28.6° (R). Leg pitch joints use one range that is inside both the left and the right servo.
- **MoveIt `joint_limits.yaml`:** same as the URDF.
- **Static `hros5_description/urdf/hros5.urdf`** (used by `hros5_bringup`'s rviz/gazebo launches): only its `<limit>` values were updated.
  It's an older export with mimic head-linkage joints that the current xacro no longer has, so it was not regenerated.
- **Teleop:** head ±88° / ±19.8°, legs ±(smaller side of each servo range), arm and hand wrist/grip ranges per side.
- **Head tracker:** pan ±88°, tilt −19.8..+33.3°.

**Consequences:**
- **Legs:** per-limb leg teleop (symmetric ±max) now only reaches knee ±18–20° and hip roll ±3°.
  `ps4_teleop_node` uses the real asymmetric servo ranges and isn't limited this way.
- **Grips:** the grip ranges are conservative until the grip directions are verified on the robot.
- **Real EEPROM limits:** whether these are the true mechanical limits is #68.

Tracked by #66 and its children #68–#73.

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
   *Done 2026-11-20: `ps4_teleop_node`, verified in Gazebo (§9). Not yet run on hardware.*
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
- `/imu` publishes about 9.81 m/s² on z at rest. During a leg move with side lean, roll, the gyro and acceleration respond (there's a short ~2 g spike when the robot settles), then it returns to level (#37).
  `/joint_states` lists all 24 joints.
- `FollowJointTrajectory` goals for arms and legs **succeed**, and the robot stays upright.
- A symmetric squat sent through the PS4/servo topic path (URDF hip −15°, knee +30°, ankle −15° on both legs) lowers the base by 1 cm with roll and pitch at 0.00°.
- **Leg directions:** in the URDF, left and right pitch joints use the **same** axis, so +knee bends both knees the natural way (shin back),
  and −hip swings the leg forward. The **servos** are mounted mirrored: the left knee servo bends towards −, the right towards +.
  The servo-to-URDF signs are in `hros5_control/config/joint_directions.yaml`.
  (An earlier version of this file wrongly said the URDF legs were mirrored.)
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

---

## 9. PS4 teleop in Gazebo (2026-11-20)

```bash
ros2 launch hros5_gazebo hros5_gazebo_teleop.launch.py limb:=head   # or left_arm, right_arm, left_leg, right_leg, hands
```

The launch starts Gazebo, a single `joy_node`, the unchanged `hros5_teleop` node for the chosen limb, and
`sim_dxl_bridge.py`, which stands in for the real `*_dxl_node` executables:

- It listens on the same `/hros5/<limb>/target_angles_deg` topics (servo degrees, 0° = tick 2048), with the same message layouts.
- It clamps to the absolute servo limits in `hros5_dynamixel_joints_with_limits.yaml` and moves at most 2° per 20 ms (grip 3°), like the real nodes.
  Like them, it starts every joint at 0°.
- It converts to URDF radians with the per-joint sign in `hros5_control/config/joint_directions.yaml` and streams all 24 joints to `whole_body_controller`.
- The echo topics (`/hros5/<limb>/echo_positions`) log the simulated servo angle and ticks.

Leg teleop starts with the robot on a stand (`fixed_base:=auto`), because moving one leg tips the free-standing robot over.

**Verified:**
- **Head:** with a real PS4 controller on skywalker, the head follows the left stick in the Gazebo GUI.
- **Bridge (scripted commands):**
  - Directions for legs, arm presets and head are as expected.
  - A left-knee command of +60° is clamped at the servo hard limit of +20° (tick 2275).
  - A symmetric squat keeps the robot level.
- **Not yet tried with the controller:** arms, legs and hands.

**Known gaps:**
- **Joint directions:** the servo-to-URDF signs marked low/medium confidence in `hros5_control/config/joint_directions.yaml` (head yaw, arms, hip yaw, ankle roll, wrist, grip) need checking on the real robot.
- **Button labels:** `joy_node` reports 8 axes and 13 buttons for the PS4 controller. That's the Linux `hid-sony` layout, which should be
  0 Cross, 1 Circle, 2 Triangle, 3 Square, 4 L1, 5 R1, 6 L2, 7 R2 (not yet confirmed button by button).
  The configs' axis indices match this layout, but some button comments don't (for example, `center_button: 3` would be Square, not Triangle).

### Whole-robot teleop: `ps4_teleop_node` (§6 step 4, #27/#28)

```bash
ros2 launch hros5_gazebo hros5_gazebo_teleop.launch.py      # default limb:=all
ros2 launch hros5_teleop ps4_teleop.launch.py               # against the real DXL nodes
```

- **Configuration:** one node and one `joy_node`. Mappings are in `ps4_mapping.yaml` (#27), deadzones and speeds in `deadzones.yaml` / `speed_scales.yaml` (#28).
- **Modes:** head, left_arm, right_arm, hands, left_leg, right_leg, and legs (both together), cycled with Options/Share.
- **Dead-man:** hold L1. If `/joy` goes silent for 0.5 s, the robot holds its position.
- **Motion:** sticks jog joints in URDF terms. `joint_directions.yaml` converts to servo degrees.
  Joints driven by the same stick stop as a group at the hard limits (servo limit minus 2°).
- **Verified in headless Gazebo with scripted `/joy`:**
  - no motion without the dead-man,
  - head yaw/pitch in the expected directions,
  - Share/Options mode switching,
  - a symmetric squat that stops as a group at the ankle limit,
  - a sideways hip shift,
  - position held on joy timeout,
  - Triangle returns joints to 0.
- **Started in the GUI** on skywalker with the real controller.
- **Hard-limit finding:** the servo limits allow only a shallow squat (hip 16°, knee 32°, limited by the right ankle's −18° servo limit)
  and only 3° of sideways hip shift (right hip roll servo max +5°). Check whether these EEPROM limits are the true mechanical limits before walking work (#68).
- **Wrist fix (#69, wrists only):** URDF wrist limits are now ±2.30 rad (servo hard limit, was ±3.14) and wrist velocity is 6.597 rad/s (was 1.0).
  With 1 rad/s the sim wrist overshot into its limit and stuck there in hands mode.
  `sim_dxl_bridge` also keeps every command 1° inside the URDF limits and logs the joints whose URDF range is narrower than the servo range (#66).
  Confirmed with the real controller in the GUI: the wrists now move freely in both directions.
- **Still open:**
  - `ps4_teleop_node` hasn't been run on the real robot yet.
  - Hardware still needs the single bus owner (§6 step 3) and the safety items (step 5).
