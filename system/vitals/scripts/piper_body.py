#!/usr/bin/env python3
"""piper_body.py — Bridge between vitals (Rust) and Piper SDK (Python).

Reads JSON commands from stdin, writes JSON responses to stdout.  Designed
as a long-running subprocess spawned by vitals' BodyCollector.

Protocol (single-line JSON, newline-delimited):
  ← {"cmd":"collect"}
  → {"body_type":"arm","model":"piper","state":<int>,"joints":[...]}

Motor mapping: piper_sdk motor_1..6 → joint_1..6.
Error codes, temperatures, and enable status come from low-speed CAN
feedback (~10 Hz).  Arm-level state is derived from the arm_status enum.
"""

from __future__ import annotations

import json
import os
import sys
import time
import traceback
from typing import Any

# ---------------------------------------------------------------------------
# piper_sdk imports – allow running from the roboarm venv or a system install
# ---------------------------------------------------------------------------
PIPER_AVAILABLE = False
try:
    import piper_sdk  # noqa: F401
    from piper_sdk import C_PiperInterface_V2

    PIPER_AVAILABLE = True
except ImportError:
    pass

# ---------------------------------------------------------------------------
# Constants
# ---------------------------------------------------------------------------
JOINT_COUNT = 6
JOINT_NAMES = [f"joint_{i}" for i in range(1, JOINT_COUNT + 1)]

# arm_status enum → BodyHealth state
ARM_STATUS_TO_STATE = {
    0x00: 0,  # NORMAL
    0x01: 2,  # EMERGENCY_STOP  → ESTOP
    0x02: 1,  # NO_SOLUTION     → FAULT
    0x03: 1,  # SINGULARITY      → FAULT
    0x04: 1,  # TARGET_EXCEEDS   → FAULT
    0x05: 1,  # JOINT_COMM_ERR   → FAULT
    0x06: 1,  # BRAKE_NOT_OPEN   → FAULT
    0x07: 1,  # COLLISION        → FAULT
    0x08: 1,  # OVERSPEED        → FAULT
    0x09: 1,  # JOINT_STATUS_ERR → FAULT
    0x0A: 1,  # OTHER            → FAULT
    0x0E: 1,  # MAIN_NTC_OVER_TEMP → FAULT
    0x0F: 1,  # RESISTOR_OVER_TEMP → FAULT
}

# ---------------------------------------------------------------------------
# Piper collector
# ---------------------------------------------------------------------------

class PiperCollector:
    def __init__(self, can_port: str) -> None:
        self._piper = C_PiperInterface_V2(can_port)
        self._piper.ConnectPort()
        # Enable all 6 motors
        self._piper.EnableArm(7)

    def collect(self) -> dict[str, Any]:
        """Read Piper sensors and return a BodyHealth-compatible dict."""
        joints: list[dict[str, Any]] = []
        foc_statuses: list[int] = []
        temperatures: list[int] = []
        enables: list[bool] = []

        try:
            low = self._piper.GetArmLowSpdInfoMsgs()
            motors = [
                low.motor_1, low.motor_2, low.motor_3,
                low.motor_4, low.motor_5, low.motor_6,
            ]
            for m in motors:
                temperatures.append(m.motor_temp)
                foc_statuses.append(m.foc_status_code)
                enables.append(m.foc_status.driver_enable_status)
        except Exception:
            # If low-speed read fails, fill with defaults
            for _ in range(JOINT_COUNT):
                temperatures.append(-1)
                foc_statuses.append(0)
                enables.append(False)

        for i in range(JOINT_COUNT):
            joints.append({
                "name": JOINT_NAMES[i],
                "temperature": float(temperatures[i]),
                "error_code": foc_statuses[i],
                "enabled": enables[i],
            })

        # Arm-level state
        state = 0  # NORMAL
        try:
            status = self._piper.GetArmStatus()
            raw = status.arm_status.arm_status
            state = ARM_STATUS_TO_STATE.get(raw, 1)
        except Exception:
            state = 1  # FAULT on read failure

        return {
            "body_type": "arm",
            "model": "piper",
            "state": state,
            "joints": joints,
        }


# ---------------------------------------------------------------------------
# main — JSON command loop on stdin/stdout
# ---------------------------------------------------------------------------

def main() -> None:
    can_port = os.environ.get("PIPER_CAN_PORT", "can0")

    if not PIPER_AVAILABLE:
        # No piper_sdk installed — return a FAULT body so vitals can still start.
        fault: dict[str, Any] = {
            "body_type": "arm",
            "model": "piper",
            "state": 1,
            "joints": [
                {"name": n, "temperature": -1.0, "error_code": 0, "enabled": False}
                for n in JOINT_NAMES
            ],
        }
        print(json.dumps(fault), flush=True)
        print("[piper_body] piper_sdk not available — returning FAULT body", file=sys.stderr)
        for line in sys.stdin:
            try:
                cmd = json.loads(line.strip())
            except json.JSONDecodeError:
                continue
            if cmd.get("cmd") == "collect":
                print(json.dumps(fault), flush=True)
        return

    # Connect
    collector: PiperCollector | None = None
    for attempt in range(5):
        try:
            collector = PiperCollector(can_port)
            break
        except Exception:
            print(f"[piper_body] connect attempt {attempt + 1}/5 failed", file=sys.stderr)
            time.sleep(2)
    if collector is None:
        print("[piper_body] could not connect to Piper arm", file=sys.stderr)
        fault = {
            "body_type": "arm", "model": "piper", "state": 1, "joints": [],
        }
        for line in sys.stdin:
            cmd = json.loads(line.strip())
            if cmd.get("cmd") == "collect":
                print(json.dumps(fault), flush=True)
        return

    print("[piper_body] connected", file=sys.stderr, flush=True)

    for line in sys.stdin:
        try:
            cmd = json.loads(line.strip())
        except json.JSONDecodeError:
            continue
        if cmd.get("cmd") == "collect":
            try:
                result = collector.collect()
            except Exception:
                traceback.print_exc(file=sys.stderr)
                result = {
                    "body_type": "arm", "model": "piper", "state": 1, "joints": [],
                }
            print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
