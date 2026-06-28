#!/usr/bin/env python3
"""collect.py — Unified vitals bridge: board sysfs + optional body SDK.

Reads JSON commands from stdin, writes JSON responses to stdout.  Designed
as a long-running subprocess spawned by vitals' VitalsCollector.

Board data (sysfs) is always collected.  Body data (Piper, Go2, etc.) is
auto-detected: if the relevant Python SDK is importable and the hardware
connects successfully, body data is included in every response; otherwise
body is null.

Protocol (single-line JSON, newline-delimited):
  ← {"cmd":"collect"}
  → {"battery_percent":<float>,"voltage":<float>,"charging":<bool>,
     "remaining_s":<int>,"components":[...],"bodies":[...]}
"""

from __future__ import annotations

import json
import os
import sys
import time
import traceback
from pathlib import Path
from typing import Any


# ── sysfs helpers (same as original board.py) ────────────────────────────────

def read_sysfs(path: Path) -> str | None:
    """Read a sysfs file via raw syscall, returning trimmed content or None."""
    try:
        fd = os.open(str(path), os.O_RDONLY)
        try:
            data = os.read(fd, 4096)
        finally:
            os.close(fd)
        return data.decode("ascii").strip()
    except (OSError, PermissionError, UnicodeDecodeError, ValueError):
        return None


def scan_thermal_zones() -> dict[str, Path]:
    """Return {zone_name: temp_file_path} for all thermal zones."""
    zones: dict[str, Path] = {}
    thermal_base = Path("/sys/class/thermal")
    if not thermal_base.is_dir():
        return zones
    for entry in sorted(thermal_base.iterdir()):
        if not entry.name.startswith("thermal_zone"):
            continue
        type_path = entry / "type"
        type_str = read_sysfs(type_path)
        if type_str is None:
            continue
        zones[type_str] = entry / "temp"
    return zones


def scan_hwmon() -> dict[str, Path]:
    """Return {device_name: hwmon_dir_path} for all hwmon devices."""
    devices: dict[str, Path] = {}
    hwmon_base = Path("/sys/class/hwmon")
    if not hwmon_base.is_dir():
        return devices
    for entry in sorted(hwmon_base.iterdir()):
        name_path = entry / "name"
        name_str = read_sysfs(name_path)
        if name_str is None:
            continue
        devices[name_str] = entry
    return devices


def read_voltage(hwmon_devices: dict[str, Path]) -> float:
    """Read system voltage from INA3221 or INA238 (mV → V)."""
    for chip in ("ina3221", "ina238"):
        dev = hwmon_devices.get(chip)
        if dev is None:
            continue
        raw = read_sysfs(dev / "in1_input")
        if raw is not None:
            try:
                return float(raw) / 1000.0
            except ValueError:
                pass
    return -1.0


def read_nvme_temp(hwmon_devices: dict[str, Path]) -> float:
    """Read NVMe temperature from hwmon (milli-°C → °C)."""
    dev = hwmon_devices.get("nvme")
    if dev is None:
        return -1.0
    raw = read_sysfs(dev / "temp1_input")
    if raw is None:
        return -1.0
    try:
        return float(raw) / 1000.0
    except ValueError:
        return -1.0


def collect_board() -> dict[str, Any]:
    """Read all sysfs sensors and return board-health portion of the response."""
    components: list[dict[str, Any]] = []
    thermal_zones = scan_thermal_zones()
    hwmon_devices = scan_hwmon()

    for zone_type, temp_path in sorted(thermal_zones.items()):
        raw = read_sysfs(temp_path)
        temp_c = -1.0
        if raw is not None:
            try:
                temp_c = float(raw) / 1000.0
            except ValueError:
                pass
        name = zone_type.removesuffix("-thermal")
        components.append({
            "name": name,
            "temperature": temp_c,
            "voltage": -1.0,
            "current": -1.0,
            "battery_percent": -1.0,
        })

    nvme_temp = read_nvme_temp(hwmon_devices)
    if nvme_temp >= 0:
        components.append({
            "name": "nvme",
            "temperature": nvme_temp,
            "voltage": -1.0,
            "current": -1.0,
            "battery_percent": -1.0,
        })

    voltage = read_voltage(hwmon_devices)

    return {
        "battery_percent": -1.0,
        "voltage": voltage,
        "charging": False,
        "remaining_s": -1,
        "components": components,
    }


# ── Body auto-detection ──────────────────────────────────────────────────────

def _try_connect_piper() -> Any | None:
    """Try to import piper_sdk and connect.  Returns PiperCollector or None."""
    # Check whether piper_sdk itself is installed before importing PiperCollector.
    try:
        import piper_sdk  # noqa: F401
    except ImportError:
        print(
            "[collect] WARNING: piper_sdk is not installed — body monitoring disabled. "
            "Install piper_sdk or set ROBONIX_VITALS_SCRIPT to a different script.",
            file=sys.stderr, flush=True,
        )
        return None

    try:
        from piper_body import PiperCollector  # type: ignore[import-not-found]
    except ImportError:
        print(
            "[collect] WARNING: piper_body module not found — body monitoring disabled.",
            file=sys.stderr, flush=True,
        )
        return None

    can_port = os.environ.get("PIPER_CAN_PORT", "can0")
    last_err: str | None = None
    for attempt in range(5):
        try:
            collector = PiperCollector(can_port)
            print(f"[collect] piper connected (can={can_port})", file=sys.stderr, flush=True)
            return collector
        except Exception as e:
            last_err = str(e) or type(e).__name__
            print(
                f"[collect] piper connect attempt {attempt + 1}/5 failed: {last_err}",
                file=sys.stderr, flush=True,
            )
            time.sleep(2)
    print(
        f"[collect] WARNING: piper connection failed after 5 attempts ({last_err}) — "
        f"body monitoring disabled. Check CAN port ({can_port}) and arm power.",
        file=sys.stderr, flush=True,
    )
    return None


def _collect_piper_body(collector: Any, fail_count: list[int]) -> dict[str, Any]:
    """Call PiperCollector.collect() and return the body dict.

    fail_count is a mutable single-element list used to rate-limit warnings:
      - First failure: log a clear warning.
      - Subsequent failures: log only every 30th to avoid spam.
      - First success after failures: log recovery.
    """
    try:
        result = collector.collect()  # type: ignore[union-attr]
        if fail_count[0] > 0:
            print(
                f"[collect] piper body recovered after {fail_count[0]} failure(s)",
                file=sys.stderr, flush=True,
            )
            fail_count[0] = 0
        return result
    except Exception:
        fail_count[0] += 1
        if fail_count[0] == 1:
            print(
                "[collect] WARNING: piper body read failed — returning FAULT. "
                "Check CAN connection and arm state.",
                file=sys.stderr, flush=True,
            )
        elif fail_count[0] % 30 == 0:
            print(
                f"[collect] piper body read still failing ({fail_count[0]} consecutive failures)",
                file=sys.stderr, flush=True,
            )
        traceback.print_exc(file=sys.stderr)
        return {
            "body_type": "arm",
            "model": "piper",
            "state": 1,  # FAULT
            "message": "collect failed",
            "components": [],
        }


# ── main ─────────────────────────────────────────────────────────────────────

def main() -> None:
    # Try to connect all known body hardware at startup.
    body_collectors: list[tuple[str, Any, list[int]]] = []  # (label, collector, fail_count)

    piper = _try_connect_piper()
    if piper is not None:
        body_collectors.append(("piper", piper, [0]))
    # Future: koch = _try_connect_koch(); go2 = _try_connect_go2(); ...

    if not body_collectors:
        print(
            "[collect] body monitoring DISABLED — no body hardware detected. "
            "Board-only mode.",
            file=sys.stderr, flush=True,
        )

    for line in sys.stdin:
        try:
            cmd = json.loads(line.strip())
        except json.JSONDecodeError:
            continue
        if cmd.get("cmd") == "collect":
            # Always collect board.
            try:
                result = collect_board()
            except Exception:
                traceback.print_exc(file=sys.stderr)
                result = {
                    "battery_percent": -1.0,
                    "voltage": -1.0,
                    "charging": False,
                    "remaining_s": -1,
                    "components": [],
                }

            # Collect all bodies.
            bodies: list[dict[str, Any]] = []
            for _label, collector, fail_count in body_collectors:
                body_data = _collect_piper_body(collector, fail_count)
                bodies.append(body_data)

            result["bodies"] = bodies
            print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
