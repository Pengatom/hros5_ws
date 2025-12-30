#!/usr/bin/env python3
"""
Compare joint limits across hardware configs, URDF, MoveIt, and teleop configs.

The script prints a per-joint summary and exits non-zero when mismatches are found.
"""

from __future__ import annotations

import argparse
import math
import sys
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from xml.etree import ElementTree

import yaml


DEG_PER_RAD = 180.0 / math.pi
TICKS_PER_DEG = 4095.0 / 360.0  # Matches existing utility scripts


@dataclass
class LimitEntry:
    min_deg: float
    max_deg: float
    source: Path


def ticks_to_deg(ticks: float) -> float:
    return (float(ticks) / TICKS_PER_DEG) - 180.0


def rad_to_deg(rad: float) -> float:
    return float(rad) * DEG_PER_RAD


def fmt_range(entry: LimitEntry | None) -> str:
    if entry is None:
        return "-"
    return f"{entry.min_deg:+7.1f}..{entry.max_deg:+7.1f}"


def fmt_span(min_val: float | None, max_val: float | None) -> str:
    if min_val is None or max_val is None:
        return "-"
    return f"{min_val:+7.1f}..{max_val:+7.1f}"


def range_differs(a: LimitEntry, b: LimitEntry, tolerance: float) -> bool:
    return (abs(a.min_deg - b.min_deg) > tolerance) or (abs(a.max_deg - b.max_deg) > tolerance)


def collect_limit(entries: list[LimitEntry]) -> LimitEntry | None:
    if not entries:
        return None
    min_deg = min(entry.min_deg for entry in entries)
    max_deg = max(entry.max_deg for entry in entries)
    return LimitEntry(min_deg=min_deg, max_deg=max_deg, source=entries[0].source)


def add_entry(bucket: dict[str, list[LimitEntry]], joint: str, entry: LimitEntry) -> None:
    bucket[joint].append(entry)


def load_control_limits(control_dir: Path, limits: dict[str, list[LimitEntry]]) -> list[str]:
    """Load hardware limits from hros5_control YAML files."""
    errors: list[str] = []
    control_files: list[Path] = []

    aggregate = control_dir / "hros5_dynamixel_joints_with_limits.yaml"
    if aggregate.exists():
        control_files.append(aggregate)
    joints_dir = control_dir / "joints"
    if joints_dir.is_dir():
        control_files.extend(sorted(joints_dir.glob("*.yaml")))

    for path in control_files:
        try:
            data = yaml.safe_load(path.read_text()) or {}
        except Exception as exc:  # pragma: no cover - defensive
            errors.append(f"Failed to parse {path}: {exc}")
            continue

        joints = data.get("joints", [])
        if not isinstance(joints, list):
            errors.append(f"'joints' not found or invalid in {path}")
            continue

        for entry in joints:
            if not isinstance(entry, dict):
                continue
            name = entry.get("name")
            cw_limit = entry.get("cw_limit")
            ccw_limit = entry.get("ccw_limit")
            if name is None or cw_limit is None or ccw_limit is None:
                continue
            try:
                min_deg = ticks_to_deg(cw_limit)
                max_deg = ticks_to_deg(ccw_limit)
            except Exception as exc:  # pragma: no cover - defensive
                errors.append(f"Bad limit values in {path}: {exc}")
                continue
            add_entry(limits, str(name), LimitEntry(min_deg=min_deg, max_deg=max_deg, source=path))

    return errors


def load_urdf_limits(urdf_path: Path, limits: dict[str, list[LimitEntry]]) -> list[str]:
    errors: list[str] = []
    if not urdf_path.exists():
        return [f"URDF not found: {urdf_path}"]

    try:
        tree = ElementTree.parse(urdf_path)
    except Exception as exc:
        return [f"Failed to parse URDF {urdf_path}: {exc}"]

    root = tree.getroot()
    for joint in root.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name is None or limit is None:
            continue
        lower = limit.get("lower")
        upper = limit.get("upper")
        if lower is None or upper is None:
            continue  # continuous joints have no bounds
        try:
            min_deg = rad_to_deg(float(lower))
            max_deg = rad_to_deg(float(upper))
        except Exception:
            continue
        add_entry(limits, name, LimitEntry(min_deg=min_deg, max_deg=max_deg, source=urdf_path))

    return errors


def load_moveit_limits(moveit_path: Path, limits: dict[str, list[LimitEntry]]) -> list[str]:
    errors: list[str] = []
    if not moveit_path.exists():
        return [f"MoveIt joint_limits.yaml not found: {moveit_path}"]

    try:
        data = yaml.safe_load(moveit_path.read_text()) or {}
    except Exception as exc:
        return [f"Failed to parse {moveit_path}: {exc}"]

    entries = data.get("joint_limits", {})
    if not isinstance(entries, dict):
        return [f"'joint_limits' mapping missing in {moveit_path}"]

    for name, cfg in entries.items():
        if not isinstance(cfg, dict):
            continue
        if not cfg.get("has_position_limits", False):
            continue
        min_pos = cfg.get("min_position")
        max_pos = cfg.get("max_position")
        if min_pos is None or max_pos is None:
            continue
        try:
            min_deg = rad_to_deg(float(min_pos))
            max_deg = rad_to_deg(float(max_pos))
        except Exception:
            continue
        add_entry(limits, str(name), LimitEntry(min_deg=min_deg, max_deg=max_deg, source=moveit_path))

    return errors


def load_arm_teleop(prefix: str, params: dict, path: Path, limits: dict[str, list[LimitEntry]]) -> None:
    mins = params.get("joint_min_deg")
    maxs = params.get("joint_max_deg")
    if not isinstance(mins, list) or not isinstance(maxs, list):
        return
    names = [
        f"{prefix}ShoulderPitch",
        f"{prefix}ShoulderRoll",
        f"{prefix}ElbowPitch",
        f"{prefix}Wrist",
        f"{prefix}Grip",
    ]
    for idx, joint in enumerate(names):
        if idx >= len(mins) or idx >= len(maxs):
            break
        try:
            add_entry(
                limits,
                joint,
                LimitEntry(min_deg=float(mins[idx]), max_deg=float(maxs[idx]), source=path),
            )
        except Exception:
            continue


def load_leg_teleop(prefix: str, params: dict, path: Path, limits: dict[str, list[LimitEntry]]) -> None:
    keys = [
        ("HipYaw", "hip_yaw_max_deg"),
        ("HipRoll", "hip_roll_max_deg"),
        ("HipPitch", "hip_pitch_max_deg"),
        ("KneePitch", "knee_pitch_max_deg"),
        ("AnklePitch", "ankle_pitch_max_deg"),
        ("AnkleRoll", "ankle_roll_max_deg"),
    ]
    for suffix, key in keys:
        if key not in params:
            continue
        try:
            span = float(params[key])
        except Exception:
            continue
        add_entry(
            limits,
            f"{prefix}{suffix}",
            LimitEntry(min_deg=-span, max_deg=span, source=path),
        )


def load_hand_teleop(params: dict, path: Path, limits: dict[str, list[LimitEntry]]) -> None:
    hand_keys = [
        ("left_wrist_min_deg", "left_wrist_max_deg", "LWrist"),
        ("left_grip_min_deg", "left_grip_max_deg", "LGrip"),
        ("right_wrist_min_deg", "right_wrist_max_deg", "RWrist"),
        ("right_grip_min_deg", "right_grip_max_deg", "RGrip"),
    ]
    for min_key, max_key, joint in hand_keys:
        if min_key not in params or max_key not in params:
            continue
        try:
            min_deg = float(params[min_key])
            max_deg = float(params[max_key])
        except Exception:
            continue
        add_entry(limits, joint, LimitEntry(min_deg=min_deg, max_deg=max_deg, source=path))


def load_head_teleop(params: dict, path: Path, limits: dict[str, list[LimitEntry]]) -> None:
    if "pan_max_deg" in params:
        try:
            span = float(params["pan_max_deg"])
            add_entry(limits, "HeadYaw", LimitEntry(min_deg=-span, max_deg=span, source=path))
        except Exception:
            pass
    if "tilt_max_deg" in params:
        try:
            span = float(params["tilt_max_deg"])
            add_entry(limits, "HeadPitch", LimitEntry(min_deg=-span, max_deg=span, source=path))
        except Exception:
            pass


def load_teleop_limits(teleop_dir: Path, limits: dict[str, list[LimitEntry]]) -> list[str]:
    errors: list[str] = []
    if not teleop_dir.is_dir():
        return [f"Teleop config dir not found: {teleop_dir}"]

    for path in sorted(teleop_dir.glob("*.yaml")):
        try:
            data = yaml.safe_load(path.read_text()) or {}
        except Exception as exc:
            errors.append(f"Failed to parse {path}: {exc}")
            continue
        node_cfg = None
        if isinstance(data, dict) and data:
            node_cfg = next(iter(data.values()))
        params = {}
        if isinstance(node_cfg, dict):
            params = node_cfg.get("ros__parameters", {}) or {}
        if not isinstance(params, dict):
            continue

        lower_name = path.name.lower()
        if "arm" in lower_name:
            prefix = "L" if "left" in lower_name else "R"
            load_arm_teleop(prefix, params, path, limits)
        if "leg" in lower_name:
            prefix = "L" if "left" in lower_name else "R"
            load_leg_teleop(prefix, params, path, limits)
        if "head" in lower_name:
            load_head_teleop(params, path, limits)
        if "hand" in lower_name:
            load_hand_teleop(params, path, limits)

    return errors


def parse_args() -> argparse.Namespace:
    default_root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser(
        description="Check joint limit consistency across hros5_control, URDF, teleop, and MoveIt."
    )
    parser.add_argument(
        "--root",
        type=Path,
        default=default_root,
        help="Workspace root (default: detected from script location).",
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        help="URDF/xacro path (default: <root>/src/hros5_description/urdf/hros5.urdf).",
    )
    parser.add_argument(
        "--control-dir",
        type=Path,
        help="hros5_control config directory (default: <root>/src/hros5_control/config).",
    )
    parser.add_argument(
        "--teleop-dir",
        type=Path,
        help="Teleop config directory (default: <root>/src/hros5_teleop/config).",
    )
    parser.add_argument(
        "--moveit",
        type=Path,
        help="MoveIt joint_limits.yaml (default: <root>/src/hros5_moveit_config/config/joint_limits.yaml).",
    )
    parser.add_argument(
        "--tolerance-deg",
        type=float,
        default=0.5,
        help="Tolerance in degrees when comparing limits (default: %(default)s).",
    )
    return parser.parse_args()


def summarize(
    buckets: dict[str, dict[str, list[LimitEntry]]], tolerance: float
) -> tuple[list[str], list[str]]:
    """Return (table_lines, issues)."""
    issues: list[str] = []
    lines: list[str] = []

    header = (
        f"{'Joint':<18} {'Hardware (deg)':<20} {'URDF (deg)':<20} "
        f"{'Teleop (deg)':<20} {'MoveIt (deg)':<20} Issues"
    )
    lines.append(header)
    lines.append("-" * len(header))

    all_joints = sorted(buckets.keys())
    for joint in all_joints:
        sources = buckets[joint]
        hw_entries = sources.get("hardware", [])
        urdf_entries = sources.get("urdf", [])
        moveit_entries = sources.get("moveit", [])
        teleop_entries = sources.get("teleop", [])

        hw_conflicts = []
        if hw_entries:
            baseline = hw_entries[0]
            for entry in hw_entries[1:]:
                if range_differs(baseline, entry, tolerance):
                    hw_conflicts.append(entry)
            if hw_conflicts:
                details = ", ".join(e.source.name for e in hw_conflicts)
                issues.append(f"{joint}: conflicting hardware limits ({baseline.source.name} vs {details})")

        hw_range = hw_entries[0] if hw_entries else None
        urdf_range = urdf_entries[0] if urdf_entries else None
        moveit_range = moveit_entries[0] if moveit_entries else None
        teleop_range = collect_limit(teleop_entries)

        joint_issues: list[str] = []

        if hw_conflicts:
            joint_issues.append("HW configs disagree")

        if hw_range:
            if urdf_range:
                if urdf_range.min_deg < hw_range.min_deg - tolerance:
                    joint_issues.append("URDF below HW")
                if urdf_range.max_deg > hw_range.max_deg + tolerance:
                    joint_issues.append("URDF above HW")
            else:
                joint_issues.append("URDF missing")

            if moveit_range:
                if range_differs(hw_range, moveit_range, tolerance):
                    joint_issues.append("MoveIt ≠ HW")
            else:
                joint_issues.append("MoveIt missing")
        else:
            if urdf_range or moveit_range or teleop_range:
                joint_issues.append("HW missing")

        if urdf_range and teleop_range:
            if teleop_range.min_deg < urdf_range.min_deg - tolerance:
                joint_issues.append("Teleop below URDF")
            if teleop_range.max_deg > urdf_range.max_deg + tolerance:
                joint_issues.append("Teleop above URDF")

        if joint_issues:
            issues.append(f"{joint}: {', '.join(joint_issues)}")

        lines.append(
            f"{joint:<18} "
            f"{fmt_range(hw_range):<20} "
            f"{fmt_range(urdf_range):<20} "
            f"{fmt_span(teleop_range.min_deg if teleop_range else None, teleop_range.max_deg if teleop_range else None):<20} "
            f"{fmt_range(moveit_range):<20} "
            f"{'; '.join(joint_issues)}"
        )

    return lines, issues


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    control_dir = (args.control_dir or (root / "src/hros5_control/config")).resolve()
    urdf_path = (args.urdf or (root / "src/hros5_description/urdf/hros5.urdf")).resolve()
    teleop_dir = (args.teleop_dir or (root / "src/hros5_teleop/config")).resolve()
    moveit_path = (args.moveit or (root / "src/hros5_moveit_config/config/joint_limits.yaml")).resolve()

    buckets: dict[str, dict[str, list[LimitEntry]]] = defaultdict(lambda: defaultdict(list))

    errors: list[str] = []
    errors.extend(load_control_limits(control_dir, buckets["hardware"]))
    errors.extend(load_urdf_limits(urdf_path, buckets["urdf"]))
    errors.extend(load_teleop_limits(teleop_dir, buckets["teleop"]))
    errors.extend(load_moveit_limits(moveit_path, buckets["moveit"]))

    # Collate buckets keyed by joint name
    per_joint: dict[str, dict[str, list[LimitEntry]]] = defaultdict(lambda: defaultdict(list))
    for source_name, source_limits in buckets.items():
        for joint, entries in source_limits.items():
            per_joint[joint][source_name].extend(entries)

    table_lines, issues = summarize(per_joint, args.tolerance_deg)

    for line in table_lines:
        print(line)

    exit_code = 0

    if errors:
        print("\nErrors while loading configurations:")
        for err in errors:
            print(f"- {err}")
        exit_code = 1

    if issues:
        print(f"\nFound {len(issues)} mismatch(es):")
        for issue in issues:
            print(f"- {issue}")
        exit_code = 1

    if exit_code == 0:
        print("\nAll limits match within tolerance.")

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
