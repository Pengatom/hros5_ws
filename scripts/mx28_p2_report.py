#!/usr/bin/env python3
"""
mx28_p2_report.py — Read & report key parameters from a live Protocol 2.0 MX (e.g., MX-28 2.0).

- No comparison; just queries the servo and prints values.
- Shows Min/Max Position Limits in ticks AND degrees.
- Also prints ID, Baud (decoded), Voltage limits, Temperature limit, Status Return Level,
  Goal/Present Position (ticks + degrees).

Usage:
  python mx28_p2_report.py --port COM5 --baud 1000000 --id 19
"""

import argparse, time
from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

# Protocol 2.0 addresses for MX-28 2.0 style layout
ADDR_ID                = 7    # 1B
ADDR_BAUD              = 8    # 1B (table-coded)
ADDR_TEMP_LIMIT        = 31   # 1B
ADDR_MAX_VOLT_dV       = 32   # 2B (0.1 V units)
ADDR_MIN_VOLT_dV       = 34   # 2B (0.1 V units)
ADDR_MAX_POS           = 48   # 4B (ticks)
ADDR_MIN_POS           = 52   # 4B (ticks)
ADDR_TORQUE_ENABLE     = 64   # 1B
ADDR_STATUS_RETURN_LVL = 68   # 1B
ADDR_GOAL_POSITION     = 116  # 4B (ticks)
ADDR_PRESENT_VELOCITY  = 128  # 4B
ADDR_PRESENT_POSITION  = 132  # 4B (ticks)

BAUD_TABLE_P2 = {
    0: 9600,
    1: 57600,
    2: 115200,
    3: 1000000,
    4: 2000000,
    5: 3000000,
    6: 4500000,
    7: 6000000,
    8: 10500000,
}

def ticks_to_deg(ticks: int | None) -> float | None:
    if ticks is None:
        return None
    # Typical MX uses 0..4095 (12-bit) for 0..360°, use 4096 for scaling
    return (int(ticks) * 360.0) / 4096.0

def read1(pkt, ph, dxl_id, addr, retries=3):
    for _ in range(retries):
        v, res, err = pkt.read1ByteTxRx(ph, dxl_id, addr)
        if res == COMM_SUCCESS and err == 0:
            return v
        time.sleep(0.002)
    return None

def read2(pkt, ph, dxl_id, addr, retries=3):
    for _ in range(retries):
        v, res, err = pkt.read2ByteTxRx(ph, dxl_id, addr)
        if res == COMM_SUCCESS and err == 0:
            return v
        time.sleep(0.002)
    return None

def read4(pkt, ph, dxl_id, addr, retries=3):
    for _ in range(retries):
        b0, r0, e0 = pkt.read1ByteTxRx(ph, dxl_id, addr)
        b1, r1, e1 = pkt.read1ByteTxRx(ph, dxl_id, addr+1)
        b2, r2, e2 = pkt.read1ByteTxRx(ph, dxl_id, addr+2)
        b3, r3, e3 = pkt.read1ByteTxRx(ph, dxl_id, addr+3)
        ok = (r0==COMM_SUCCESS and e0==0 and
              r1==COMM_SUCCESS and e1==0 and
              r2==COMM_SUCCESS and e2==0 and
              r3==COMM_SUCCESS and e3==0)
        if ok:
            return (b3<<24)|(b2<<16)|(b1<<8)|b0
        time.sleep(0.002)
    return None

def main():
    ap = argparse.ArgumentParser(description="Report key parameters from a Protocol 2.0 MX servo (e.g., MX-28 2.0).")
    ap.add_argument("--port", required=True, help="Serial port, e.g., COM5 or /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=1000000, help="Baud (bps), e.g., 1000000")
    ap.add_argument("--id", type=int, required=True, help="Dynamixel ID")
    args = ap.parse_args()

    ph = PortHandler(args.port)
    if not ph.openPort():
        raise SystemExit(f"ERROR: Could not open {args.port}")
    if not ph.setBaudRate(args.baud):
        raise SystemExit(f"ERROR: Could not set baud {args.baud}")
    pkt = PacketHandler(2.0)

    # Read values
    rid   = read1(pkt, ph, args.id, ADDR_ID)
    braw  = read1(pkt, ph, args.id, ADDR_BAUD)
    bps   = BAUD_TABLE_P2.get(braw, None) if braw is not None else None
    tlim  = read1(pkt, ph, args.id, ADDR_TEMP_LIMIT)
    vmax  = read2(pkt, ph, args.id, ADDR_MAX_VOLT_dV)
    vmin  = read2(pkt, ph, args.id, ADDR_MIN_VOLT_dV)
    sret  = read1(pkt, ph, args.id, ADDR_STATUS_RETURN_LVL)
    maxp  = read4(pkt, ph, args.id, ADDR_MAX_POS)
    minp  = read4(pkt, ph, args.id, ADDR_MIN_POS)
    goalp = read4(pkt, ph, args.id, ADDR_GOAL_POSITION)
    presp = read4(pkt, ph, args.id, ADDR_PRESENT_POSITION)

    # Print report
    print("=== MX-28 P2 Parameter Report ===\n")
    print(f"{'ID':24} {rid}")
    print(f"{'Baud Rate (bps)':24} {bps}  (raw={braw})")
    print(f"{'Min Voltage (V)':24} {None if vmin is None else vmin/10.0}")
    print(f"{'Max Voltage (V)':24} {None if vmax is None else vmax/10.0}")
    print(f"{'Temperature Limit (°C)':24} {tlim}")
    print(f"{'Status Return Level':24} {sret}")

    print("\n-- Angle/Position limits --")
    print(f"{'Min Position (ticks)':24} {minp}  ({ticks_to_deg(minp):.2f} deg)" if minp is not None else f"{'Min Position (ticks)':24} None")
    print(f"{'Max Position (ticks)':24} {maxp}  ({ticks_to_deg(maxp):.2f} deg)" if maxp is not None else f"{'Max Position (ticks)':24} None")

    print("\n-- Positions --")
    print(f"{'Goal Position (ticks)':24} {goalp} ({ticks_to_deg(goalp):.2f} deg)" if goalp is not None else f"{'Goal Position (ticks)':24} None")
    print(f"{'Present Position (ticks)':24} {presp} ({ticks_to_deg(presp):.2f} deg)" if presp is not None else f"{'Present Position (ticks)':24} None")

    ph.closePort()

if __name__ == "__main__":
    main()
