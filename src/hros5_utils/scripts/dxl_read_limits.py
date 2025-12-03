#!/usr/bin/env python3
"""
Read position limits from Dynamixel MX-28 / MX-64 / MX-106 (Protocol 1.0 or 2.0).

The script automatically queries each servo for its model number,
matches it against known MX configurations, and then reads CW/CCW
angle limits using the appropriate control-table addresses for the
selected Dynamixel protocol version (2.0 default, 1.0 optional).

Default:
  - Device: /dev/dxl
  - Baudrate: 1,000,000
  - IDs: 1–24

Usage examples:
  python3 dxl_read_limits.py
  python3 dxl_read_limits.py --ids 1 3 5 7 9 --port /dev/ttyUSB0 --baud 57600
  python3 dxl_read_limits.py --protocol 2.0 --all --csv-path logs/mx_limits_after_upgrade.csv
"""

import sys
import argparse
import csv
from datetime import datetime
from pathlib import Path

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    COMM_SUCCESS,
)

# --- Servo definitions (Protocol 1.0 & 2.0) ---
SERVO_CONFIGS = {
    "mx28": {
        "label": "MX-28",
        "model_numbers": {29, 30},
        "ticks_per_rev": 4096.0,
    },
    "mx64": {
        "label": "MX-64",
        "model_numbers": {310, 311},
        "ticks_per_rev": 4096.0,
    },
    "mx106": {
        "label": "MX-106",
        "model_numbers": {320, 321},
        "ticks_per_rev": 4096.0,
    },
}
MODEL_TO_SERVO = {
    model: servo_type
    for servo_type, cfg in SERVO_CONFIGS.items()
    for model in cfg["model_numbers"]
}
ADDR_MODEL_NUMBER = 0

PROTOCOL_CONFIGS = {
    1.0: {
        "firmware_addr": 2,
        "cw_limit": {"addr": 6, "length": 2},
        "ccw_limit": {"addr": 8, "length": 2},
        "extra_fields": [
            ("temp_limit_c", 11, 1, lambda v: float(v)),
            ("voltage_min_v", 12, 1, lambda v: round(float(v) / 10.0, 2)),
            ("voltage_max_v", 13, 1, lambda v: round(float(v) / 10.0, 2)),
            ("max_torque_raw", 14, 2, int),
            ("status_return_level", 16, 1, int),
            ("alarm_shutdown_mask", 18, 1, int),
        ],
    },
    2.0: {
        "firmware_addr": 6,
        "cw_limit": {"addr": 52, "length": 4},   # Min position
        "ccw_limit": {"addr": 48, "length": 4},  # Max position
        "extra_fields": [
            ("temp_limit_c", 31, 1, lambda v: float(v)),
            ("voltage_min_v", 34, 2, lambda v: round(float(v) / 10.0, 2)),
            ("voltage_max_v", 32, 2, lambda v: round(float(v) / 10.0, 2)),
            ("max_torque_raw", 14, 2, int),
            ("status_return_level", 68, 1, int),
            ("alarm_shutdown_mask", 18, 1, int),
        ],
    },
}

BASE_FIELDS = [
    "id",
    "model",
    "servo_type",
    "firmware",
    "cw_limit_ticks",
    "cw_limit_deg",
    "ccw_limit_ticks",
    "ccw_limit_deg",
    "range_deg",
]

def read_word(packet_handler: PacketHandler, port_handler: PortHandler, dxl_id: int, address: int):
    """Read 2-byte value from control table."""
    value, result, error = packet_handler.read2ByteTxRx(port_handler, dxl_id, address)
    if result != COMM_SUCCESS:
        raise RuntimeError(f"COMM error: {packet_handler.getTxRxResult(result)}")
    if error != 0:
        raise RuntimeError(f"DXL error: {packet_handler.getRxPacketError(error)}")
    return value


def read_byte(packet_handler: PacketHandler, port_handler: PortHandler, dxl_id: int, address: int):
    """Read 1-byte value from control table."""
    value, result, error = packet_handler.read1ByteTxRx(port_handler, dxl_id, address)
    if result != COMM_SUCCESS:
        raise RuntimeError(f"COMM error: {packet_handler.getTxRxResult(result)}")
    if error != 0:
        raise RuntimeError(f"DXL error: {packet_handler.getRxPacketError(error)}")
    return value


def read_dword(packet_handler: PacketHandler, port_handler: PortHandler, dxl_id: int, address: int):
    """Read 4-byte value from control table."""
    value, result, error = packet_handler.read4ByteTxRx(port_handler, dxl_id, address)
    if result != COMM_SUCCESS:
        raise RuntimeError(f"COMM error: {packet_handler.getTxRxResult(result)}")
    if error != 0:
        raise RuntimeError(f"DXL error: {packet_handler.getRxPacketError(error)}")
    return value


def read_value(packet_handler, port_handler, dxl_id, address: int, length: int):
    if length == 1:
        return read_byte(packet_handler, port_handler, dxl_id, address)
    if length == 2:
        return read_word(packet_handler, port_handler, dxl_id, address)
    if length == 4:
        return read_dword(packet_handler, port_handler, dxl_id, address)
    raise ValueError(f"Unsupported data length {length}")


def ticks_to_deg(ticks: int, deg_per_tick: float) -> float:
    return ticks * deg_per_tick


def main():
    parser = argparse.ArgumentParser(
        description="Read CW/CCW angle limits (position range) from Dynamixel MX-series (Protocol 1.0 or 2.0)."
    )
    parser.add_argument(
        "--port", "--device", "-d",
        dest="port",
        default="/dev/dxl",
        help="Serial device for Dynamixel bus (default: /dev/dxl)",
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=1000000,
        help="Baudrate (default: 1000000)",
    )
    parser.add_argument(
        "--ids",
        nargs="+",
        type=int,
        default=[i for i in range(1, 25)],
        help="List of servo IDs to query (default: 1-24)",
    )
    parser.add_argument(
        "--protocol",
        type=float,
        choices=sorted(PROTOCOL_CONFIGS.keys()),
        default=2.0,
        help="Dynamixel protocol version to use (1.0 or 2.0).",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="Read all key limit registers and write them to CSV for later comparison.",
    )
    parser.add_argument(
        "--csv-path",
        type=str,
        default=None,
        help="Optional CSV output path (used with --all). Defaults to mx_limits_p<protocol>_<timestamp>.csv",
    )

    args = parser.parse_args()
    if args.csv_path and not args.all:
        print("⚠ --csv-path is ignored unless --all is specified.", file=sys.stderr)

    protocol_cfg = PROTOCOL_CONFIGS.get(args.protocol)
    if not protocol_cfg:
        print(f"❌ Unsupported protocol version: {args.protocol}", file=sys.stderr)
        sys.exit(1)

    port_handler = PortHandler(args.port)
    packet_handler = PacketHandler(args.protocol)

    if not port_handler.openPort():
        print(f"❌ Failed to open port {args.port}", file=sys.stderr)
        sys.exit(1)

    if not port_handler.setBaudRate(args.baud):
        print(f"❌ Failed to set baudrate {args.baud}", file=sys.stderr)
        sys.exit(1)

    print(f"Using device: {args.port}, baud: {args.baud}, protocol: {args.protocol}")
    print("IDs:", args.ids)
    print()
    print(" ID | Model | Type   | FW |  CW_limit (ticks/deg)  | CCW_limit (ticks/deg) | Range (deg)")
    print("----+-------+--------+----+------------------------+------------------------+-----------")

    csv_rows = []

    for dxl_id in args.ids:
        try:
            model = read_word(packet_handler, port_handler, dxl_id, ADDR_MODEL_NUMBER)
            servo_type = MODEL_TO_SERVO.get(model)
            if not servo_type:
                print(
                    f"{dxl_id:3d} | {model:5d} | {'?':<6} | -- | "
                    f"{'--':>5} ({'--':>7}) | {'--':>5} ({'--':>7}) | Unsupported model"
                )
                continue

            servo_cfg = SERVO_CONFIGS[servo_type]
            deg_per_tick = 360.0 / servo_cfg["ticks_per_rev"]

            fw    = read_value(
                packet_handler,
                port_handler,
                dxl_id,
                protocol_cfg["firmware_addr"],
                1,
            )
            cw    = read_value(
                packet_handler,
                port_handler,
                dxl_id,
                protocol_cfg["cw_limit"]["addr"],
                protocol_cfg["cw_limit"]["length"],
            )
            ccw   = read_value(
                packet_handler,
                port_handler,
                dxl_id,
                protocol_cfg["ccw_limit"]["addr"],
                protocol_cfg["ccw_limit"]["length"],
            )

            cw_deg  = ticks_to_deg(cw, deg_per_tick)
            ccw_deg = ticks_to_deg(ccw, deg_per_tick)
            span_deg = max(0.0, ccw_deg - cw_deg)

            row = {
                "id": dxl_id,
                "model": model,
                "servo_type": servo_cfg["label"],
                "firmware": fw,
                "cw_limit_ticks": cw,
                "cw_limit_deg": round(cw_deg, 3),
                "ccw_limit_ticks": ccw,
                "ccw_limit_deg": round(ccw_deg, 3),
                "range_deg": round(span_deg, 3),
            }

            if args.all:
                for field_name, addr, length, transform in protocol_cfg["extra_fields"]:
                    raw = read_value(packet_handler, port_handler, dxl_id, addr, length)
                    row[field_name] = transform(raw)
                csv_rows.append(row)

            print(
                f"{dxl_id:3d} | {model:5d} | {servo_cfg['label']:<6} | {fw:2d} | "
                f"{cw:5d} ({cw_deg:7.2f}°) | "
                f"{ccw:5d} ({ccw_deg:7.2f}°) | {span_deg:7.2f}°"
            )
        except RuntimeError as e:
            print(f"{dxl_id:3d} |  ---  | -- | ERROR: {e}")

    port_handler.closePort()

    if args.all:
        if not csv_rows:
            print("⚠ No servo data captured; CSV not written.")
            return
        if args.csv_path:
            csv_path = Path(args.csv_path)
        else:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            ids_suffix = "-".join(str(row["id"]) for row in csv_rows) or "none"
            proto_tag = f"p{int(args.protocol)}"
            csv_path = Path(f"mx_limits_{proto_tag}_{ts}_ids-{ids_suffix}.csv")
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        extra_field_names = [name for name, _, _, _ in protocol_cfg["extra_fields"]]
        fieldnames = BASE_FIELDS + extra_field_names
        with csv_path.open("w", newline="") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(csv_rows)
        print(f"\n✅ Saved extended limit data for {len(csv_rows)} servo(s) to {csv_path}")


if __name__ == "__main__":
    main()
