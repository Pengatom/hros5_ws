#!/usr/bin/env python3
"""
dxl_ping.py

Ping a Dynamixel ID to verify comms and report its model number.

Example:
  python3 dxl_ping.py --port /dev/dxl --baud 1000000 --protocol 2.0 --id 19
"""

import argparse
import sys
from pathlib import Path


DEFAULT_SDK_PATH = Path(__file__).resolve().parents[2] / "dynamixel" / "DynamixelSDK" / "python" / "src"


def import_sdk(sdk_path: Path):
    try:
        import dynamixel_sdk  # type: ignore
    except ImportError:
        sys.path.append(str(sdk_path))
        try:
            import dynamixel_sdk  # type: ignore
        except ImportError:
            print(f"❌ Could not import dynamixel_sdk from {sdk_path}")
            sys.exit(1)
    from dynamixel_sdk import PacketHandler, PortHandler, COMM_SUCCESS  # type: ignore
    return PacketHandler, PortHandler, COMM_SUCCESS


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Ping a Dynamixel and print the model number.")
    parser.add_argument("--port", "--device", dest="port", default="/dev/dxl", help="Serial device (default: /dev/dxl)")
    parser.add_argument("--baud", type=int, default=1000000, help="Baudrate (default: 1000000)")
    parser.add_argument("--protocol", type=float, default=2.0, help="Dynamixel protocol version (default: 2.0)")
    parser.add_argument("--id", type=int, required=True, help="Servo ID to ping")
    parser.add_argument("--sdk-path", type=Path, default=DEFAULT_SDK_PATH, help="Path to DynamixelSDK python/src folder (used if SDK not importable).")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    PacketHandler, PortHandler, COMM_SUCCESS = import_sdk(args.sdk_path)

    print(f"Using SDK from: {args.sdk_path}")
    print(f"Opening {args.port} @ {args.baud} baud, protocol {args.protocol}, ID {args.id}")

    port = PortHandler(args.port)
    packet = PacketHandler(args.protocol)

    if not port.openPort():
        print(f"❌ Failed to open port {args.port}")
        return 1
    print(f"✅ Opened port {args.port}")

    if not port.setBaudRate(args.baud):
        print(f"❌ Failed to set baudrate to {args.baud}")
        port.closePort()
        return 1
    print(f"✅ Baudrate set to {args.baud}")

    print(f"➡️  Pinging Dynamixel ID {args.id}...")
    dxl_model_number, dxl_comm_result, dxl_error = packet.ping(port, args.id)

    if dxl_comm_result != COMM_SUCCESS:
        print(f"❌ Communication failed: {packet.getTxRxResult(dxl_comm_result)}")
    elif dxl_error != 0:
        print(f"⚠️  Error from servo: {packet.getRxPacketError(dxl_error)}")
    else:
        print(f"✅ Success! ID {args.id} responded, model number = {dxl_model_number}")

    port.closePort()
    print("🔚 Port closed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
