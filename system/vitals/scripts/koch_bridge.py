#!/usr/bin/env python3
"""koch_bridge.py — Koch arm hardware bridge (stdin/stdout JSON protocol).

Spawned by mock Soma as a long-running subprocess.  Reads JSON commands from
stdin and writes JSON responses to stdout (one line per response).

Usage: koch_bridge.py <serial_port> [baudrate]

Protocol:
  ← {"cmd":"collect"}
  → {"body_type":"arm","model":"koch","state":<int>,"message":"",
     "components":[{"name":"joint_1","kind":"joint",...}, ...]}

Motor mapping: Dynamixel ID 1..6 → joint_1..6.
  ID 1: shoulder_pan   (xl430-w250) → joint_1
  ID 2: shoulder_lift  (xl430-w250) → joint_2
  ID 3: elbow_flex     (xl330-m288) → joint_3
  ID 4: wrist_flex     (xl330-m288) → joint_4
  ID 5: wrist_roll     (xl330-m288) → joint_5
  ID 6: gripper        (xl330-m288) → joint_6

Health data (temperature, error code, torque enable) comes from Dynamixel
sync-read of control-table registers.  Arm-level state is derived from
per-motor Hardware_Error_Status.
"""

from __future__ import annotations

import json
import sys
from typing import Any

# ---------------------------------------------------------------------------
# dynamixel_sdk imports
# ---------------------------------------------------------------------------
DXL_AVAILABLE = False
try:
    from dynamixel_sdk import (  # type: ignore[import-untyped]
        COMM_SUCCESS,
        GroupSyncRead,
        PacketHandler,
        PortHandler,
    )

    DXL_AVAILABLE = True
except ImportError:
    pass

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
PROTOCOL_VERSION = 2.0
DEFAULT_BAUDRATE = 1_000_000

JOINT_COUNT = 6
JOINT_NAMES = [f"joint_{i}" for i in range(1, JOINT_COUNT + 1)]

# Dynamixel motor IDs 1..6 → joint_1..6
MOTOR_IDS = list(range(1, JOINT_COUNT + 1))

# X-series control-table addresses
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PRESENT_TEMPERATURE = 146
LEN_TORQUE_ENABLE = 1
LEN_HARDWARE_ERROR_STATUS = 1
LEN_PRESENT_TEMPERATURE = 1

# Hardware_Error_Status bit → fault label (Dynamixel X-series)
HW_ERROR_BITS: list[tuple[int, str]] = [
    (0, "input_voltage"),
    (2, "overheat"),
    (3, "motor_encoder"),
    (4, "electrical_shock"),
    (5, "overload"),
]


def _decode_hw_error(error_code: int) -> str:
    """Decode Hardware_Error_Status bitmask into a compact fault label string."""
    if error_code == 0:
        return ""
    labels = [label for bit, label in HW_ERROR_BITS if error_code & (1 << bit)]
    if not labels:
        return f"0x{error_code:02X}"
    return ",".join(labels)


def _sync_read_u8(reader: Any, address: int, length: int) -> tuple[list[int], list[bool]]:
    """Read one byte-like register from all Koch motors.

    Dynamixel SDK's GroupSyncRead.txRxPacket() returns one communication result,
    not a per-motor failure bitmap.  Per-motor availability must be checked with
    isAvailable() before getData().
    """
    reader.clearParam()
    params_ok: list[bool] = []
    for mid in MOTOR_IDS:
        params_ok.append(bool(reader.addParam(mid)))

    comm_result = reader.txRxPacket()
    values: list[int] = []
    available: list[bool] = []
    for i, mid in enumerate(MOTOR_IDS):
        ok = (
            params_ok[i]
            and comm_result == COMM_SUCCESS
            and bool(reader.isAvailable(mid, address, length))
        )
        if ok:
            values.append(int(reader.getData(mid, address, length)))
            available.append(True)
        else:
            values.append(-1)
            available.append(False)
    return values, available


# ---------------------------------------------------------------------------
# Koch collector
# ---------------------------------------------------------------------------


class KochCollector:
    """Read Koch arm joint motor health via Dynamixel sync-read."""

    def __init__(self, port: str, baudrate: int = DEFAULT_BAUDRATE) -> None:
        if not DXL_AVAILABLE:
            raise ImportError(
                "dynamixel-sdk is not installed. Install it in your Python environment "
                "to use the Koch hardware bridge."
            )
        self._port_handler = PortHandler(port)
        self._packet_handler = PacketHandler(PROTOCOL_VERSION)
        if not self._port_handler.openPort():
            raise ConnectionError(f"failed to open Dynamixel port {port}")
        if not self._port_handler.setBaudRate(baudrate):
            raise ConnectionError(f"failed to set baudrate {baudrate} on {port}")

        # Pre-build GroupSyncRead objects for each register we poll.
        self._temp_reader = GroupSyncRead(
            self._port_handler,
            self._packet_handler,
            ADDR_PRESENT_TEMPERATURE,
            LEN_PRESENT_TEMPERATURE,
        )
        self._error_reader = GroupSyncRead(
            self._port_handler,
            self._packet_handler,
            ADDR_HARDWARE_ERROR_STATUS,
            LEN_HARDWARE_ERROR_STATUS,
        )
        self._torque_reader = GroupSyncRead(
            self._port_handler,
            self._packet_handler,
            ADDR_TORQUE_ENABLE,
            LEN_TORQUE_ENABLE,
        )

    def collect(self) -> dict[str, Any]:
        """Read Koch sensors and return a BodyHealth-compatible dict."""
        components: list[dict[str, Any]] = []

        # ── Read temperatures ──────────────────────────────────────────
        temps, temp_ok = _sync_read_u8(
            self._temp_reader,
            ADDR_PRESENT_TEMPERATURE,
            LEN_PRESENT_TEMPERATURE,
        )

        # ── Read hardware error status ─────────────────────────────────
        errors, error_ok = _sync_read_u8(
            self._error_reader,
            ADDR_HARDWARE_ERROR_STATUS,
            LEN_HARDWARE_ERROR_STATUS,
        )

        # ── Read torque enable ─────────────────────────────────────────
        enable_values, enable_ok = _sync_read_u8(
            self._torque_reader,
            ADDR_TORQUE_ENABLE,
            LEN_TORQUE_ENABLE,
        )
        enables = [
            ok and value == 1 for value, ok in zip(enable_values, enable_ok)
        ]

        joint_ok = [
            temp_ok[i] and error_ok[i] and enable_ok[i] for i in range(JOINT_COUNT)
        ]

        # ── Build component list ───────────────────────────────────────
        for i in range(JOINT_COUNT):
            components.append(
                {
                    "name": JOINT_NAMES[i],
                    "kind": "joint",
                    "temperature": float(temps[i]),
                    "error_code": max(errors[i], 0),  # -1 (comm fail) → 0
                    "enabled": enables[i],
                }
            )

        # ── Arm-level state ────────────────────────────────────────────
        state = 0  # NORMAL
        message_parts: list[str] = []
        has_comm_failure = any(not ok for ok in joint_ok)
        has_hw_error = any(e > 0 for e in errors)

        if has_hw_error or has_comm_failure:
            state = 1  # FAULT

        # Decode fault labels and communication failures into arm-level message.
        for i, comp in enumerate(components):
            if not joint_ok[i]:
                message_parts.append(f"{comp['name']}:comm")
            if comp["error_code"] != 0:
                label = _decode_hw_error(comp["error_code"])
                message_parts.append(f"{comp['name']}:{label}")

        return {
            "body_type": "arm",
            "model": "koch",
            "state": state,
            "message": "; ".join(message_parts),
            "components": components,
        }


# ---------------------------------------------------------------------------
# Main: stdin/stdout JSON protocol
# ---------------------------------------------------------------------------


def main() -> None:
    if len(sys.argv) < 2:
        print(
            "usage: koch_bridge.py <serial_port> [baudrate]",
            file=sys.stderr,
            flush=True,
        )
        sys.exit(1)

    serial_port = sys.argv[1]
    baudrate = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_BAUDRATE
    try:
        collector = KochCollector(serial_port, baudrate)
    except Exception as exc:
        print(f"[koch_bridge] startup failed: {exc}", file=sys.stderr, flush=True)
        sys.exit(1)

    print(
        f"[koch_bridge] connected to Koch arm via {serial_port} @ {baudrate} bps",
        file=sys.stderr,
        flush=True,
    )

    for line in sys.stdin:
        try:
            cmd = json.loads(line.strip())
        except json.JSONDecodeError:
            continue
        if cmd.get("cmd") == "collect":
            try:
                result = collector.collect()
            except Exception as exc:
                print(f"[koch_bridge] collect failed: {exc}", file=sys.stderr, flush=True)
                continue
            print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
