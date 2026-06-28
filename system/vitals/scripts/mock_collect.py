#!/usr/bin/env python3
"""mock_collect.py — Simulated vitals health for testing without hardware.

Reads JSON commands from stdin, writes JSON responses to stdout.  Same
protocol as collect.py.  Produces deterministic fake data for both board
and body.

Protocol:
  ← {"cmd":"collect"}
  → {"battery_percent":<float>,"voltage":<float>,"charging":<bool>,
     "remaining_s":<int>,"components":[...],"bodies":[...]}

Scenarios:

  MOCK_COLLECT_SCENARIO (board data, default "normal"):
    normal       — stable temps (CPU 42°C, GPU 38°C, TJ 46°C, NVMe 40°C), 19.5V
    ramp         — CPU temp ramps 40→95°C over 30 collects (cycling)
    low_voltage  — voltage drops 19.5→9.0V over 20 collects (cycling)

  MOCK_BODY_SCENARIO (body data, default "none"):
    none    — "body": null (board-only testing)
    normal  — 6 joints, all OK, stable temps 35-47.5°C, all enabled
    ramp    — joint_1 temp ramps 40→83.5°C over 30 collects, crossing
              WARN (60°C) at collect≈13, ERROR (75°C) at collect≈23
    fault   — fault codes rotate every 5 collects on a rotating joint
    toggle  — joint_6 toggles enable every 4 collects
    mixed   — ramp + occasional faults + toggle
"""

from __future__ import annotations

import json
import math
import os
import sys
from typing import Any

# ── Board data generators ────────────────────────────────────────────────────

COMPONENT_DEFS = [
    ("cpu", 42.0),
    ("gpu", 38.0),
    ("tj", 46.0),
    ("nvme", 40.0),
]

JOINT_COUNT = 6
JOINT_NAMES = [f"joint_{i}" for i in range(1, JOINT_COUNT + 1)]

# Piper foc_status bit → fault label (mirrors piper_body.py)
FAULT_BITS: list[tuple[int, str]] = [
    (0, "欠压"),
    (1, "电机过热"),
    (2, "过流"),
    (3, "驱动器过热"),
    (4, "碰撞"),
    (5, "驱动器故障"),
    (7, "堵转"),
]


def _decode_faults(error_code: int) -> str:
    """Decode error_code bitmask into compact fault labels."""
    if error_code == 0:
        return ""
    labels = [label for bit, label in FAULT_BITS if error_code & (1 << bit)]
    if not labels:
        return f"0x{error_code:02X}"
    return ",".join(labels)


def _board_components(
    collect_n: int, scenario: str, voltage: float
) -> list[dict[str, Any]]:
    comps = []
    for name, base in COMPONENT_DEFS:
        variation = math.sin((collect_n + hash(name)) * 0.5) * 1.5
        temp = round(base + variation, 1)
        comps.append({
            "name": name,
            "temperature": temp,
            "voltage": -1.0,
            "current": -1.0,
            "battery_percent": -1.0,
        })

    # Expose voltage as a "battery" component so threshold rules can match it.
    comps.append({
        "name": "battery",
        "temperature": -1.0,
        "voltage": voltage,
        "current": -1.0,
        "battery_percent": -1.0,
    })

    if scenario == "ramp":
        cycle = collect_n % 30
        comps[0]["temperature"] = round(40.0 + (cycle * 55.0 / 29.0), 1)

    return comps


def _board_power(collect_n: int, scenario: str) -> dict[str, Any]:
    voltage = 19.5
    if scenario == "low_voltage":
        cycle = collect_n % 20
        voltage = round(19.5 - (cycle * 10.5 / 19.0), 2)
    return {
        "battery_percent": -1.0,
        "voltage": voltage,
        "charging": False,
        "remaining_s": -1,
    }


BOARD_SCENARIOS = {"normal", "ramp", "low_voltage"}


# ── Body data generators ─────────────────────────────────────────────────────

def _body_normal(collect_n: int) -> list[dict[str, Any]]:
    components = []
    for i in range(JOINT_COUNT):
        base = 35.0 + (i * 2.5)
        variation = math.sin((collect_n + i) * 0.7) * 2.0
        components.append({
            "name": JOINT_NAMES[i],
            "kind": "joint",
            "temperature": round(base + variation, 1),
            "error_code": 0,
            "enabled": True,
        })
    return components


def _body_ramp(collect_n: int) -> list[dict[str, Any]]:
    components = _body_normal(collect_n)
    cycle = collect_n % 30
    components[0]["temperature"] = round(40.0 + (cycle * 1.5), 1)
    return components


def _body_fault(collect_n: int) -> list[dict[str, Any]]:
    components = _body_normal(collect_n)
    epoch = collect_n // 5
    patterns = [
        (),              # 0: no faults
        (2, 5),          # 1: overcurrent + driver fault
        (1,),            # 2: motor overheat
        (7,),            # 3: stall
        (0, 1, 2),      # 4: undervoltage + overheat + overcurrent
    ]
    pattern = patterns[epoch % len(patterns)]
    target = epoch % JOINT_COUNT
    code = sum(1 << bit for bit in pattern)
    if code != 0:
        components[target] = {
            "name": JOINT_NAMES[target],
            "kind": "joint",
            "temperature": 52.0,
            "error_code": code,
            "enabled": True,
        }
    return components


def _body_toggle(collect_n: int) -> list[dict[str, Any]]:
    components = _body_normal(collect_n)
    components[5]["enabled"] = (collect_n // 4) % 2 == 0
    return components


def _body_mixed(collect_n: int) -> list[dict[str, Any]]:
    components = _body_normal(collect_n)
    # Ramp on joint_1
    cycle = collect_n % 30
    components[0]["temperature"] = round(40.0 + (cycle * 1.5), 1)
    # Fault on joint_3 every 8 collects
    if collect_n % 8 >= 6:
        code = (1 << 2) | (1 << 5)
        components[2] = {"name": "joint_3", "kind": "joint", "temperature": 55.0, "error_code": code, "enabled": True}
    # Toggle joint_6 every 5 collects
    components[5]["enabled"] = (collect_n // 5) % 2 == 0
    return components


BODY_GENERATORS = {
    "normal": _body_normal,
    "ramp": _body_ramp,
    "fault": _body_fault,
    "toggle": _body_toggle,
    "mixed": _body_mixed,
}


# ── main ─────────────────────────────────────────────────────────────────────

def collect(board_scenario: str, body_scenario: str | None, collect_n: int) -> dict[str, Any]:
    result = _board_power(collect_n, board_scenario)
    result["components"] = _board_components(collect_n, board_scenario, result["voltage"])

    if body_scenario is not None and body_scenario in BODY_GENERATORS:
        components = BODY_GENERATORS[body_scenario](collect_n)
        # Build message from any non-zero error_codes.
        msg_parts = [
            f"{c['name']}:{_decode_faults(c['error_code'])}"
            for c in components
            if c["error_code"] != 0
        ]
        result["bodies"] = [{
            "body_type": "arm",
            "model": "piper",
            "state": 0,
            "message": "; ".join(msg_parts),
            "components": components,
        }]
    else:
        result["bodies"] = []

    return result


def main() -> None:
    board_scenario = os.environ.get("MOCK_COLLECT_SCENARIO", "normal")
    if board_scenario not in BOARD_SCENARIOS:
        print(
            f"[mock_collect] unknown board scenario '{board_scenario}', using 'normal'",
            file=sys.stderr,
        )
        board_scenario = "normal"

    body_scenario_raw = os.environ.get("MOCK_BODY_SCENARIO", "none")
    body_scenario: str | None = None if body_scenario_raw == "none" else body_scenario_raw
    if body_scenario is not None and body_scenario not in BODY_GENERATORS:
        print(
            f"[mock_collect] unknown body scenario '{body_scenario}', using none. "
            f"Available: {', '.join(sorted(BODY_GENERATORS))}",
            file=sys.stderr,
        )
        body_scenario = None

    print(
        f"[mock_collect] board={board_scenario} body={body_scenario or 'none'} pid={os.getpid()}",
        file=sys.stderr, flush=True,
    )

    collect_n = 0
    for line in sys.stdin:
        try:
            cmd = json.loads(line.strip())
        except json.JSONDecodeError:
            continue
        if cmd.get("cmd") == "collect":
            result = collect(board_scenario, body_scenario, collect_n)
            print(json.dumps(result), flush=True)
            collect_n += 1


if __name__ == "__main__":
    main()
