#!/usr/bin/env python3
"""
Shared utilities for Dynamixel helper scripts.

- Provides a consistent default port/baud/protocol.
- Tries to import `dynamixel_sdk`, falling back to the workspace SDK checkout
  at src/dynamixel/DynamixelSDK/python/src when the module is not installed.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

DEFAULT_PORT = "/dev/dxl"
DEFAULT_BAUD = 1_000_000
DEFAULT_PROTOCOL = 2.0

# Fallback path for the in-tree SDK copy
SDK_FALLBACK_PATH = Path(__file__).resolve().parents[2] / "dynamixel" / "DynamixelSDK" / "python" / "src"


def import_sdk() -> Any:
    """
    Import and return the `dynamixel_sdk` module.

    If the module is not installed in the environment, attempt to import it from
    the repository checkout at SDK_FALLBACK_PATH.
    """
    try:
        import dynamixel_sdk as sdk  # type: ignore
        return sdk
    except ImportError:
        if SDK_FALLBACK_PATH.exists():
            sys.path.append(str(SDK_FALLBACK_PATH))
            import dynamixel_sdk as sdk  # type: ignore
            return sdk
        raise


def open_port(port: str, baud: int):
    """Open and configure a PortHandler. Raises RuntimeError on failure."""
    sdk = import_sdk()
    handler = sdk.PortHandler(port)
    if not handler.openPort():
        raise RuntimeError(f"Failed to open port {port}")
    if not handler.setBaudRate(baud):
        raise RuntimeError(f"Failed to set baudrate {baud}")
    return handler

