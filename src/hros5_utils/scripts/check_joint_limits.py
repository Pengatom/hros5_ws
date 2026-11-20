#!/usr/bin/env python3
"""
Check that every joint limit in the workspace stays inside the servo hard limits.

Rule (2026-11-20): the servo EEPROM limits in
hros5_control/config/hros5_dynamixel_joints_with_limits.yaml are the absolute maximum.
Every other limit must be at least --guard-deg (default 2 deg) inside them:

  - teleop configs and the head tracker work in servo degrees (0 deg = tick 2048):
    compared directly with the servo range
  - URDF and MoveIt work in URDF radians: the servo range is first converted with
    hros5_control/config/joint_directions.yaml (URDF = direction * servo). For joints
    whose direction has confidence "low", only the part of the range that is valid with
    either sign is allowed, so a wrong sign can never put a limit outside the servo range.
  - MoveIt position limits must also equal the URDF limits.

Limits that are narrower than the servo range are allowed (reported as info only).
Exits non-zero when any limit is outside. --propose prints tightened URDF ranges.
"""

from __future__ import annotations

import argparse
import math
import shutil
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from xml.etree import ElementTree

import yaml

TICKS_PER_DEG = 4096.0 / 360.0  # same as the DXL nodes: 0 deg = tick 2048


@dataclass
class Range:
    lo: float
    hi: float
    source: str = ""

    def fmt(self) -> str:
        return f"{self.lo:+7.1f}..{self.hi:+7.1f}"


@dataclass
class JointLimits:
    servo: Range | None = None                      # servo degrees, hard limit
    direction: int = 1
    confidence: str = "low"
    urdf: Range | None = None                       # degrees, URDF frame
    moveit: Range | None = None                     # degrees, URDF frame
    servo_frame: list[Range] = field(default_factory=list)  # teleop / tracker, servo degrees


def ticks_to_deg(ticks: float) -> float:
    return float(ticks) / TICKS_PER_DEG - 180.0


# ----------------------------------------------------------------------------- loaders
def load_servo_limits(path: Path, joints: dict[str, JointLimits]) -> None:
    for j in (yaml.safe_load(path.read_text()) or {}).get("joints", []):
        joints.setdefault(j["name"], JointLimits()).servo = Range(
            ticks_to_deg(j["cw_limit"]), ticks_to_deg(j["ccw_limit"]), path.name)


def load_directions(path: Path, joints: dict[str, JointLimits]) -> None:
    data = yaml.safe_load(path.read_text()) or {}
    for name, d in (data.get("direction") or {}).items():
        joints.setdefault(name, JointLimits()).direction = int(d)
    for name, c in (data.get("confidence") or {}).items():
        joints.setdefault(name, JointLimits()).confidence = str(c)


def urdf_xml(urdf: Path) -> str:
    if urdf.suffix == ".xacro":
        exe = shutil.which("xacro")
        if not exe:
            sys.exit(f"xacro not found (source ROS 2 first) to expand {urdf}")
        res = subprocess.run([exe, str(urdf)], capture_output=True, text=True)
        if res.returncode != 0:
            sys.exit(f"xacro failed for {urdf} (source install/setup.bash first?):\n{res.stderr.strip()}")
        return res.stdout
    return urdf.read_text()


def load_urdf(urdf: Path, joints: dict[str, JointLimits]) -> None:
    root = ElementTree.fromstring(urdf_xml(urdf))
    for j in root.findall("joint"):
        lim = j.find("limit")
        if j.get("type") != "revolute" or lim is None or lim.get("lower") is None:
            continue
        joints.setdefault(j.get("name"), JointLimits()).urdf = Range(
            math.degrees(float(lim.get("lower"))), math.degrees(float(lim.get("upper"))), urdf.name)


def load_moveit(path: Path, joints: dict[str, JointLimits]) -> None:
    for name, cfg in ((yaml.safe_load(path.read_text()) or {}).get("joint_limits") or {}).items():
        if isinstance(cfg, dict) and cfg.get("has_position_limits") and "min_position" in cfg:
            joints.setdefault(name, JointLimits()).moveit = Range(
                math.degrees(float(cfg["min_position"])), math.degrees(float(cfg["max_position"])), path.name)


def params_of(path: Path) -> dict:
    data = yaml.safe_load(path.read_text()) or {}
    node = next(iter(data.values())) if isinstance(data, dict) and data else {}
    return (node or {}).get("ros__parameters", {}) or {}


def add_servo_frame(joints, name, lo, hi, source):
    joints.setdefault(name, JointLimits()).servo_frame.append(Range(float(lo), float(hi), source))


def load_teleop(teleop_dir: Path, joints: dict[str, JointLimits]) -> None:
    """Per-limb teleop configs. Their values are servo degrees (what they publish)."""
    for path in sorted(teleop_dir.glob("ps4_*_teleop.yaml")):
        p, name = params_of(path), path.name
        side = "L" if "left" in name else "R"
        if "arm" in name and "joint_min_deg" in p:
            for idx, suffix in enumerate(["ShoulderPitch", "ShoulderRoll", "ElbowPitch", "Wrist", "Grip"]):
                add_servo_frame(joints, side + suffix, p["joint_min_deg"][idx], p["joint_max_deg"][idx], name)
            if "grip_min_deg" in p:
                add_servo_frame(joints, side + "Grip", p["grip_min_deg"], p["grip_max_deg"], name + " (grip)")
        if "leg" in name:
            for suffix, key in [("HipYaw", "hip_yaw"), ("HipRoll", "hip_roll"), ("HipPitch", "hip_pitch"),
                                ("KneePitch", "knee_pitch"), ("AnklePitch", "ankle_pitch"), ("AnkleRoll", "ankle_roll")]:
                if f"{key}_max_deg" in p:  # the leg nodes use +-max
                    m = float(p[f"{key}_max_deg"])
                    add_servo_frame(joints, side + suffix, -m, m, name)
        if "head" in name:  # the head node uses +-max
            if "pan_max_deg" in p:
                add_servo_frame(joints, "HeadYaw", -float(p["pan_max_deg"]), float(p["pan_max_deg"]), name)
            if "tilt_max_deg" in p:
                add_servo_frame(joints, "HeadPitch", -float(p["tilt_max_deg"]), float(p["tilt_max_deg"]), name)
        if "hands" in name:
            for s, prefix in (("L", "left"), ("R", "right")):
                for part, joint in (("wrist", "Wrist"), ("grip", "Grip")):
                    if f"{prefix}_{part}_min_deg" in p:
                        add_servo_frame(joints, s + joint, p[f"{prefix}_{part}_min_deg"],
                                        p[f"{prefix}_{part}_max_deg"], name)


def load_tracker(path: Path, joints: dict[str, JointLimits]) -> None:
    data = yaml.safe_load(path.read_text()) or {}
    for node, cfg in data.items():
        p = (cfg or {}).get("ros__parameters", {}) or {}
        if "pan_min_deg" in p:
            add_servo_frame(joints, "HeadYaw", p["pan_min_deg"], p["pan_max_deg"], f"{path.name} ({node})")
        if "tilt_min_deg" in p:
            add_servo_frame(joints, "HeadPitch", p["tilt_min_deg"], p["tilt_max_deg"], f"{path.name} ({node})")


# ----------------------------------------------------------------------------- rules
def safe_servo(j: JointLimits, guard: float) -> Range:
    return Range(j.servo.lo + guard, j.servo.hi - guard)


def safe_urdf(j: JointLimits, guard: float) -> Range:
    s = safe_servo(j, guard)
    mapped = Range(*sorted((j.direction * s.lo, j.direction * s.hi)))
    if j.confidence == "low":  # must hold for either sign
        return Range(max(s.lo, -s.hi), min(s.hi, -s.lo))
    return mapped


def outside(r: Range, safe: Range, tol: float) -> bool:
    return r.lo < safe.lo - tol or r.hi > safe.hi + tol


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0].strip())
    p.add_argument("--root", type=Path, default=root, help="Workspace root (default: detected).")
    p.add_argument("--urdf", type=Path, help="URDF or xacro (default: src/hros5_description/urdf/hros5.xacro).")
    p.add_argument("--guard-deg", type=float, default=2.0, help="Required margin inside the servo limits.")
    p.add_argument("--tolerance-deg", type=float, default=0.05, help="Rounding tolerance.")
    p.add_argument("--propose", action="store_true", help="Print tightened URDF limits (rad) for joints that are outside.")
    return p.parse_args()


def main() -> int:
    a = parse_args()
    src = a.root.resolve() / "src"
    joints: dict[str, JointLimits] = {}
    load_servo_limits(src / "hros5_control/config/hros5_dynamixel_joints_with_limits.yaml", joints)
    load_directions(src / "hros5_control/config/joint_directions.yaml", joints)
    load_urdf((a.urdf or src / "hros5_description/urdf/hros5.xacro").resolve(), joints)
    load_moveit(src / "hros5_moveit_config/config/joint_limits.yaml", joints)
    load_teleop(src / "hros5_teleop/config", joints)
    tracker = src / "hros5_head_tracker_cpp/config/head_params.yaml"
    if tracker.exists():
        load_tracker(tracker, joints)

    g, tol = a.guard_deg, a.tolerance_deg
    problems: list[str] = []
    narrower: list[str] = []
    proposals: list[str] = []
    print(f"All values in degrees. Safe = servo hard limit minus {g:g} deg guard band.")
    print(f"{'Joint':<15} {'dir/conf':<10} {'Servo HW':<17} {'Safe servo':<17} {'Safe URDF':<17} "
          f"{'URDF':<17} {'MoveIt':<17} Issues")
    for name in sorted(joints):
        j = joints[name]
        if j.servo is None:
            if j.urdf is not None:
                problems.append(f"{name}: no servo limits")
            continue
        ss, su = safe_servo(j, g), safe_urdf(j, g)
        issues = []
        if j.urdf is None:
            issues.append("URDF missing")
        elif outside(j.urdf, su, tol):
            issues.append("URDF outside")
            lo, hi = max(j.urdf.lo, su.lo), min(j.urdf.hi, su.hi)
            proposals.append(f"{name:<15} {math.radians(lo):+.4f} .. {math.radians(hi):+.4f} rad "
                             f"({lo:+.1f} .. {hi:+.1f} deg, was {j.urdf.fmt()})")
        elif j.urdf.lo > su.lo + 1 or j.urdf.hi < su.hi - 1:
            narrower.append(f"{name}: URDF {j.urdf.fmt()} inside safe {su.fmt()}")
        if j.moveit is not None:
            if outside(j.moveit, su, tol):
                issues.append("MoveIt outside")
            if j.urdf is not None and (abs(j.moveit.lo - j.urdf.lo) > tol or abs(j.moveit.hi - j.urdf.hi) > tol):
                issues.append("MoveIt != URDF")
        for r in j.servo_frame:
            if outside(r, ss, tol):
                issues.append(f"{r.source} outside ({r.fmt()})")
        if issues:
            problems.append(f"{name}: " + "; ".join(issues))
        print(f"{name:<15} {j.direction:+d}/{j.confidence:<7} {j.servo.fmt():<17} {ss.fmt():<17} {su.fmt():<17} "
              f"{j.urdf.fmt() if j.urdf else '-':<17} {j.moveit.fmt() if j.moveit else '-':<17} {'; '.join(issues)}")

    if narrower:
        print("\nInfo, narrower than the servo range (allowed):")
        for n in narrower:
            print(f"- {n}")
    if a.propose and proposals:
        print("\nProposed URDF limits (current limit clipped to the safe URDF range):")
        for p in proposals:
            print(f"  {p}")
    if problems:
        print(f"\n{len(problems)} joint(s) with limits outside the servo hard limits (minus {g:g} deg):")
        for p in problems:
            print(f"- {p}")
        return 1
    print(f"\nAll limits are inside the servo hard limits (minus {g:g} deg).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
