#!/usr/bin/env python3
"""
dxl_health_check.py — Quick health check for Protocol 2.0 Dynamixels.

What it does:
- Pings a list/range of IDs, reports model numbers.
- Reads present voltage, temperature, and hardware error status.
- Optional small motion exercise to see if commands succeed and watch for droop.

Usage examples:
  python3 dxl_health_check.py --port /dev/dxl --baud 1000000 --ids 11 12 13
  python3 dxl_health_check.py --port /dev/dxl --baud 1000000 --start-id 1 --end-id 30 --move-deg 5
"""

import argparse
import time
from typing import Iterable, List, Optional, Tuple

from dynamixel_sdk import COMM_RX_TIMEOUT, COMM_SUCCESS, PacketHandler, PortHandler

# Protocol 2.0 addresses (MX-series layout)
# Protocol 2.0 addresses (MX-series layout)
ADDR_TORQUE_ENABLE = 64          # 1B
ADDR_HARDWARE_ERROR = 70         # 1B
ADDR_GOAL_POSITION = 116         # 4B
ADDR_PRESENT_POSITION = 132      # 4B
ADDR_PRESENT_VOLTAGE_dV = 144    # 2B (0.1 V)
ADDR_PRESENT_TEMPERATURE = 146   # 1B (°C)

HW_ERROR_BITS = {
    0: "Input voltage",
    1: "Overheating",
    2: "Motor encoder",
    3: "Electrical shock",
    4: "Overload",
}

BROWNOUT_THRESHOLD_V = 10.5

def read1(pkt: PacketHandler, ph: PortHandler, dxl_id: int, addr: int, retries: int = 3) -> Optional[int]:
    for _ in range(retries):
        val, res, err = pkt.read1ByteTxRx(ph, dxl_id, addr)
        if res == COMM_SUCCESS and err == 0:
            return val
        time.sleep(0.002)
    return None


def read2(pkt: PacketHandler, ph: PortHandler, dxl_id: int, addr: int, retries: int = 3) -> Optional[int]:
    for _ in range(retries):
        val, res, err = pkt.read2ByteTxRx(ph, dxl_id, addr)
        if res == COMM_SUCCESS and err == 0:
            return val
        time.sleep(0.002)
    return None


def read4(pkt: PacketHandler, ph: PortHandler, dxl_id: int, addr: int, retries: int = 3) -> Optional[int]:
    for _ in range(retries):
        val, res, err = pkt.read4ByteTxRx(ph, dxl_id, addr)
        if res == COMM_SUCCESS and err == 0:
            return val
        time.sleep(0.002)
    return None


def write1(pkt: PacketHandler, ph: PortHandler, dxl_id: int, addr: int, value: int, retries: int = 3) -> Tuple[bool, str]:
    for _ in range(retries):
        res, err = pkt.write1ByteTxRx(ph, dxl_id, addr, value)
        if res == COMM_SUCCESS and err == 0:
            return True, ""
        time.sleep(0.002)
    return False, pkt.getTxRxResult(res) if res != COMM_SUCCESS else pkt.getRxPacketError(err)


def write4(pkt: PacketHandler, ph: PortHandler, dxl_id: int, addr: int, value: int, retries: int = 3) -> Tuple[bool, str]:
    for _ in range(retries):
        res, err = pkt.write4ByteTxRx(ph, dxl_id, addr, value)
        if res == COMM_SUCCESS and err == 0:
            return True, ""
        time.sleep(0.002)
    return False, pkt.getTxRxResult(res) if res != COMM_SUCCESS else pkt.getRxPacketError(err)


def ticks_from_deg(deg: float) -> int:
    # 0..4095 ticks ≈ 0..360°
    return int(round((deg / 360.0) * 4096.0))


def clamp(val: int, lo: int, hi: int) -> int:
    return max(lo, min(hi, val))


def format_hw_error(hw_err: int) -> str:
    if hw_err == 0:
        return "0 (OK)"
    flags = [name for bit, name in HW_ERROR_BITS.items() if hw_err & (1 << bit)]
    return f"{hw_err} ({', '.join(flags)})"


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Ping IDs, read voltage/temp/hw-error, and optionally do a small motion exercise.")
    ap.add_argument("--port", default="/dev/dxl", help="Serial port (default: /dev/dxl)")
    ap.add_argument("--baud", type=int, default=1000000, help="Baudrate (bps) for all operations (default: 1000000)")
    ap.add_argument("--ids", type=int, nargs="+", help="Explicit list of IDs to check")
    ap.add_argument("--start-id", type=int, default=1, help="Range start ID (inclusive) if --ids not given (default: 1)")
    ap.add_argument("--end-id", type=int, default=30, help="Range end ID (inclusive) if --ids not given (default: 30)")
    ap.add_argument("--move-deg", type=float, default=0.0, help="If >0, move each servo by this many degrees and back (default: 0 = no motion)")
    ap.add_argument("--move-wait", type=float, default=0.25, help="Seconds to wait after a goal write before reading back (default: 0.25)")
    ap.add_argument("--retries", type=int, default=3, help="Retries for each read/write (default: 3)")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    ids: Iterable[int] = args.ids if args.ids else range(args.start_id, args.end_id + 1)
    missing_ids: List[int] = []
    error_ids: List[int] = []
    voltages: List[float] = []
    found_ids: List[int] = []

    ph = PortHandler(args.port)
    if not ph.openPort():
        raise SystemExit(f"ERROR: Could not open port {args.port}")
    if not ph.setBaudRate(args.baud):
        raise SystemExit(f"ERROR: Could not set baudrate {args.baud}")
    pkt = PacketHandler(2.0)

    print(f"Health check on {args.port} @ {args.baud} bps | move: {args.move_deg} deg")

    try:
        for dxl_id in ids:
            model, res, err = pkt.ping(ph, dxl_id)
            if res == COMM_RX_TIMEOUT:
                if args.ids:
                    print(f"\nID {dxl_id}:")
                    print("  ❌ No response (timeout)")
                    missing_ids.append(dxl_id)
                continue  # silent skip for IDs that aren't present in range scan

            print(f"\nID {dxl_id}:")
            if res != COMM_SUCCESS or err != 0:
                print(f"  ❌ Ping failed: {pkt.getTxRxResult(res)}")
                error_ids.append(dxl_id)
                continue
            print(f"  ✅ Ping ok | Model: {model}")
            found_ids.append(dxl_id)

            volt_raw = read2(pkt, ph, dxl_id, ADDR_PRESENT_VOLTAGE_dV, retries=args.retries)
            temp_c = read1(pkt, ph, dxl_id, ADDR_PRESENT_TEMPERATURE, retries=args.retries)
            hw_err = read1(pkt, ph, dxl_id, ADDR_HARDWARE_ERROR, retries=args.retries)
            volt_v = None if volt_raw is None else volt_raw / 10.0

            if volt_v is not None:
                voltages.append(volt_v)
                print(f"  Voltage: {volt_v} V")
            else:
                print("  Voltage: read failed")
                error_ids.append(dxl_id)
            print(f"  Temp: {temp_c} °C" if temp_c is not None else "  Temp: read failed")
            if hw_err is not None:
                print(f"  HW Error: {format_hw_error(hw_err)}")
                if hw_err != 0:
                    error_ids.append(dxl_id)
            else:
                print("  HW Error: read failed")
                error_ids.append(dxl_id)

            if args.move_deg <= 0:
                continue

            # Motion exercise
            present_pos = read4(pkt, ph, dxl_id, ADDR_PRESENT_POSITION, retries=args.retries)
            if present_pos is None:
                print("  ⚠️ Skipping motion: could not read present position")
                error_ids.append(dxl_id)
                continue

            delta_ticks = ticks_from_deg(args.move_deg)
            direction = -1 if present_pos > 2048 else 1
            goal = clamp(present_pos + direction * delta_ticks, 0, 4095)

            torque_enabled = read1(pkt, ph, dxl_id, ADDR_TORQUE_ENABLE, retries=args.retries)
            if torque_enabled == 0 or torque_enabled is None:
                ok, err_msg = write1(pkt, ph, dxl_id, ADDR_TORQUE_ENABLE, 1, retries=args.retries)
                if not ok:
                    print(f"  ⚠️ Enable torque failed: {err_msg}")
                    error_ids.append(dxl_id)
                    continue

            ok, err_msg = write4(pkt, ph, dxl_id, ADDR_GOAL_POSITION, goal, retries=args.retries)
            if not ok:
                print(f"  ⚠️ Goal write failed: {err_msg}")
                error_ids.append(dxl_id)
                continue

            time.sleep(args.move_wait)
            moved_pos = read4(pkt, ph, dxl_id, ADDR_PRESENT_POSITION, retries=args.retries)
            moved_volt_raw = read2(pkt, ph, dxl_id, ADDR_PRESENT_VOLTAGE_dV, retries=args.retries)
            moved_volt = None if moved_volt_raw is None else moved_volt_raw / 10.0

            back_ok, back_err = write4(pkt, ph, dxl_id, ADDR_GOAL_POSITION, present_pos, retries=args.retries)
            if not back_ok:
                print(f"  ⚠️ Return-to-start failed: {back_err}")
                error_ids.append(dxl_id)
            else:
                time.sleep(args.move_wait)

            print(f"  Move -> pos: {moved_pos} (start {present_pos}), voltage after move: {moved_volt} V")

            if torque_enabled == 0:
                write1(pkt, ph, dxl_id, ADDR_TORQUE_ENABLE, 0, retries=args.retries)
    finally:
        ph.closePort()

    print("\n=== SUMMARY ===")
    if args.ids:
        print(f"Requested IDs: {list(ids)}")
    if found_ids:
        print(f"Responding IDs: {sorted(set(found_ids))}")
    if voltages:
        print(f"Bus voltage min/max: {min(voltages):.1f} V / {max(voltages):.1f} V")
        low_v = min(voltages)
        if low_v < BROWNOUT_THRESHOLD_V:
            print(f"⚠️ Voltage dipped below {BROWNOUT_THRESHOLD_V} V: min {low_v:.1f} V")
    else:
        print("No voltage readings captured.")
    if missing_ids:
        print(f"Missing IDs (no response): {missing_ids}")
    if error_ids:
        print(f"IDs reporting errors/read failures: {sorted(set(error_ids))}")

    if missing_ids:
        return 3
    if error_ids:
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
