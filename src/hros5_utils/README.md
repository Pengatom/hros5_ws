## hros5_utils Dynamixel scripts

All Dynamixel utilities live in `src/hros5_utils/scripts`. Run any tool with `--help` for arguments.

- `dxl_scan_bus.py` — Sweep an ID range across baudrates/protocols to see which servos respond.
- `dxl_ping.py` — Quick ping to verify comms and read the model number for a single ID.
- `dxl_servo_check.py` — Scan a range of IDs and report model + firmware version.
- `dxl_fw_version.py` — Read firmware registers for specific IDs (protocol 1.0 or 2.0).
- `dxl_read_limits.py` — Read CW/CCW angle limits (and optional extra fields) and optionally write CSV.
- `dxl_apply_limits_p2.py` — Write saved limits from a CSV (generated via `dxl_read_limits.py --all`) onto Protocol 2.0 servos.
- `dxl_p2_report.py` — Report voltage/temp/status-return and angle limits for Protocol 2.0 MX-series servos.
- `dxl_read_positions.py` — Poll present positions for selected IDs or joint names from a YAML config.
- `dxl_check_position.py` — Watch the present position register for a single servo.
- `dxl_goal_position.py` — Enable torque, set a goal position, read back the result, and disable torque.
