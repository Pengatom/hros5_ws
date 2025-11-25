#!/usr/bin/env python3
"""
Read present positions from one or more Dynamixel servos.

Example:
  ./scripts/dynamixel_read_positions.py --joint-names LShoulderPitch LShoulderRoll
  ./scripts/dynamixel_read_positions.py --ids 11 12 --watch
"""

import argparse
import sys
import time
from pathlib import Path

import yaml
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler


ADDR_PRESENT_POSITION = 132
TICKS_PER_DEG = 4095.0 / 360.0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Poll the present position register for selected Dynamixel servos."
    )
    parser.add_argument(
        "--device", default="/dev/dxl",
        help="Serial device for the Dynamixel bus (default: %(default)s)",
    )
    parser.add_argument(
        "--baud", type=int, default=1_000_000,
        help="Bus baud rate (default: %(default)s)",
    )
    parser.add_argument(
        "--protocol", type=float, default=2.0,
        help="Dynamixel protocol version (default: %(default)s)",
    )
    parser.add_argument(
        "--ids", type=int, nargs="+",
        help="Servo IDs to query",
    )
    parser.add_argument(
        "--joint-names", nargs="+",
        help="Joint names to resolve to IDs using --config",
    )
    parser.add_argument(
        "--config", default="src/hros5_control/config/hros5_dynamixel_joints_with_limits.yaml",
        help="YAML file containing joint definitions (default: %(default)s)",
    )
    parser.add_argument(
        "--watch", action="store_true",
        help="Continuously report until interrupted",
    )
    parser.add_argument(
        "--interval", type=float, default=0.5,
        help="Interval in seconds between updates when --watch is set (default: %(default)s)",
    )
    return parser.parse_args()


def load_joint_ids(config_path: Path) -> dict:
    if not config_path.exists():
        sys.exit(f"Config file not found: {config_path}")
    with config_path.open("r", encoding="utf-8") as fh:
        data = yaml.safe_load(fh) or {}
    joints = data.get("joints")
    if not isinstance(joints, list):
        sys.exit(f"'joints' key missing or invalid in {config_path}")
    mapping = {}
    for entry in joints:
        name = entry.get("name")
        joint_id = entry.get("id")
        if name is None or joint_id is None:
            continue
        mapping[name] = int(joint_id)
    if not mapping:
        sys.exit(f"No joints found in {config_path}")
    return mapping


def resolve_ids(args: argparse.Namespace) -> list:
    ids = []
    seen = set()

    def append_if_new(value: int):
        if value not in seen:
            ids.append(value)
            seen.add(value)

    if args.ids:
        for value in args.ids:
            append_if_new(int(value))

    if args.joint_names:
        mapping = load_joint_ids(Path(args.config))
        for name in args.joint_names:
            if name not in mapping:
                sys.exit(f"Joint '{name}' not found in {args.config}")
            append_if_new(mapping[name])

    if not ids:
        sys.exit("Provide at least one servo ID via --ids or --joint-names")
    return ids


def ticks_to_deg(ticks: int) -> float:
    return (ticks / TICKS_PER_DEG) - 180.0


def format_position(ticks: int) -> str:
    return f"{ticks:5d} ticks ({ticks_to_deg(ticks):+7.2f} deg)"


def main() -> int:
    args = parse_args()
    ids = resolve_ids(args)

    port = PortHandler(args.device)
    packet = PacketHandler(args.protocol)

    if not port.openPort():
        print(f"Failed to open port {args.device}", file=sys.stderr)
        return 1
    if not port.setBaudRate(args.baud):
        print(f"Failed to set baud rate to {args.baud}", file=sys.stderr)
        port.closePort()
        return 1

    header = ", ".join(str(i) for i in ids)
    print(f"Polling servos: {header}")

    try:
        while True:
            timestamp = time.strftime("%H:%M:%S")
            for servo_id in ids:
                position, res, err = packet.read4ByteTxRx(port, servo_id, ADDR_PRESENT_POSITION)
                if res != COMM_SUCCESS:
                    status = packet.getTxRxResult(res)
                    print(f"[{timestamp}] ID {servo_id:3d}: READ ERROR ({status})")
                    continue
                if err:
                    status = packet.getRxPacketError(err)
                    print(f"[{timestamp}] ID {servo_id:3d}: PACKET ERROR ({status})")
                    continue
                print(f"[{timestamp}] ID {servo_id:3d}: {format_position(position)}")
            if not args.watch:
                break
            time.sleep(max(0.01, args.interval))
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        port.closePort()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
