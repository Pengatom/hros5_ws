## hros5_utils Dynamixel scripts

All Dynamixel utilities live in `src/hros5_utils/scripts`. Run any tool with `--help` for arguments.

- `check_joint_limits.py` — Check that every limit (URDF, MoveIt, teleop, head tracker) is at least 2° inside the servo hard limits; prints a table and exits non-zero otherwise.
- `generate_joint_yamls.py` — Generate per-limb joint YAMLs from `hros5_dynamixel_joints_with_limits.yaml`; use `--check` to guard against stale files in CI.
- `dxl_scan.py` — Sweep an ID range across baudrates/protocols to see which servos respond.
- `dxl_ping.py` — Quick ping to verify comms and read the model number for a single ID.
- `dxl_servo_check.py` — Scan a range of IDs and report model + firmware version.
- `dxl_check_firmware.py` — Read firmware versions for specific IDs.
- `dxl_read_limits.py` — Read CW/CCW angle limits (and optional extra fields) and optionally write CSV.
- `dxl_apply_limits.py` — Write saved limits from a CSV (generated via `dxl_read_limits.py --all`) onto Protocol 2.0 servos.
- `dxl_report_mx_p2.py` — Report voltage/temp/status-return and angle limits for Protocol 2.0 MX-series servos.
- `dxl_health_check.py` — Ping IDs, read voltage/temp/hardware-error bits, and optionally do a small motion exercise.
- `dxl_read_positions.py` — Poll present positions for selected IDs or joint names from a YAML config.
- `dxl_check_position.py` — Watch the present position register for a single servo.
- `dxl_watch_position.py` — Continuously print the present position for a single servo.
- `dxl_goal_test.py` — Send a goal position to one servo and report the present position.
- `dxl_common.py` — Shared helpers (default port `/dev/dxl`).

> Stop all `*_dxl_node` processes before running these scripts. They open `/dev/dxl` directly.
> Since 2026-11-20, `check_joint_limits.py` passes: all limits are inside the servo hard limits (see `docs/STATUS.md` §4).
> `generate_joint_yamls.py --check` gives a false "stale" result on Windows because of path separators, so run it on Linux.

### Joint limit consistency

The servo EEPROM limits (`hros5_control/config/hros5_dynamixel_joints_with_limits.yaml`) are the absolute maximum.
Run the checker from the workspace root with the workspace sourced (it expands `hros5.xacro`):

```
source install/setup.bash
python3 src/hros5_utils/scripts/check_joint_limits.py            # --propose prints tightened URDF limits
```

**How it compares limits:**
- **Teleop and head tracker configs** are in servo degrees and are compared directly with "servo limit minus `--guard-deg`" (default 2°).
- **URDF and MoveIt** are converted to servo degrees with `hros5_control/config/joint_directions.yaml` first.
  For joints whose direction has confidence `low`, only the part of the range that is valid with either sign is allowed.
- **MoveIt** limits must also equal the URDF.

Limits that are narrower than needed are listed as info. The checker returns 1 when any limit is outside, so CI can gate on it.

### Generate per-limb joint YAMLs

Regenerate `hros5_control/config/joints/*.yaml` (or verify they are current):

```
# Write fresh files
python3 src/hros5_utils/scripts/generate_joint_yamls.py

# CI-friendly: exit non-zero if files are stale/missing
python3 src/hros5_utils/scripts/generate_joint_yamls.py --check
```

All generated YAMLs include a `DO NOT EDIT` header and are sourced from `hros5_control/config/hros5_dynamixel_joints_with_limits.yaml`.
