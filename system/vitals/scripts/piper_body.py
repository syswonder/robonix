#!/usr/bin/env python3
"""piper_body.py — Piper SDK bridge (importable module).

Provides PiperCollector which reads joint motor health via piper_sdk CAN
interface.  Intended to be imported by collect.py, not run standalone.

Protocol (PiperCollector.collect):
  Returns {"body_type":"arm","model":"piper","state":<int>,"message":"",
           "components":[{"name":"joint_1","kind":"joint",...}, ...]}

Motor mapping: piper_sdk motor_1..6 → joint_1..6.
Error codes, temperatures, and enable status come from low-speed CAN
feedback (~10 Hz).  Arm-level state is derived from the arm_status enum.
"""

from __future__ import annotations

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

# foc_status bit → fault label (from piper_sdk CAN protocol)
FOC_FAULT_BITS: list[tuple[int, str]] = [
    (0, "undervoltage"),
    (1, "motor_overheat"),
    (2, "overcurrent"),
    (3, "driver_overheat"),
    (4, "collision"),
    (5, "driver_fault"),
    (7, "stall"),
]


def _decode_faults(error_code: int) -> str:
    """Decode foc_status bitmask into a compact fault label string."""
    if error_code == 0:
        return ""
    labels = [label for bit, label in FOC_FAULT_BITS if error_code & (1 << bit)]
    if not labels:
        return f"0x{error_code:02X}"
    return ",".join(labels)


# ---------------------------------------------------------------------------
# Piper collector
# ---------------------------------------------------------------------------

class PiperCollector:
    def __init__(self, can_port: str) -> None:
        if not PIPER_AVAILABLE:
            raise ImportError(
                "piper_sdk is not installed. Install it in your Python environment "
                "to use the Piper hardware bridge."
            )
        self._piper = C_PiperInterface_V2(can_port)
        self._piper.ConnectPort()

    def collect(self) -> dict[str, Any]:
        """Read Piper sensors and return a BodyHealth-compatible dict."""
        components: list[dict[str, Any]] = []
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
            components.append({
                "name": JOINT_NAMES[i],
                "kind": "joint",
                "temperature": float(temperatures[i]),
                "error_code": foc_statuses[i],
                "enabled": enables[i],
            })

        # Arm-level state
        state = 0  # NORMAL
        message_parts: list[str] = []
        try:
            status = self._piper.GetArmStatus()
            raw = status.arm_status.arm_status
            state = ARM_STATUS_TO_STATE.get(raw, 1)
        except Exception:
            state = 1  # FAULT on read failure
            message_parts.append("arm_status read failed")

        # Decode fault labels from per-component error_codes into message.
        for comp in components:
            if comp["error_code"] != 0:
                label = _decode_faults(comp["error_code"])
                message_parts.append(f"{comp['name']}:{label}")

        return {
            "body_type": "arm",
            "model": "piper",
            "state": state,
            "message": "; ".join(message_parts),
            "components": components,
        }
