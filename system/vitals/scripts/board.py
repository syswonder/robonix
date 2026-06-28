#!/usr/bin/env python3
"""board.py — Bridge between vitals (Rust) and Jetson sysfs sensors.

Reads JSON commands from stdin, writes JSON responses to stdout.  Designed
as a long-running subprocess spawned by vitals' BoardCollector.

Protocol (single-line JSON, newline-delimited):
  ← {"cmd":"collect"}
  → {"battery_percent":<float>,"voltage":<float>,"charging":<bool>,
     "remaining_s":<int>,"components":[...]}

Sensors read:
  - Thermal zones: /sys/class/thermal/thermal_zone*/temp (milli-°C → °C)
  - NVMe temperature: hwmon nvme/temp1_input (milli-°C → °C)
  - System voltage: hwmon ina3221|ina238/in1_input (mV → V)

When sysfs is unavailable (non-Jetson), returns empty readings with voltage=-1.
"""

from __future__ import annotations

import json
import os
import sys
import traceback
from pathlib import Path
from typing import Any


def read_sysfs(path: Path) -> str | None:
    """Read a sysfs file, returning trimmed content or None.

    Uses os.open + os.read (single raw syscall, no buffering).  Python's
    higher-level I/O (pathlib.read_text, pathlib.read_bytes, open()) all
    go through buffered / incremental-decoder paths that break on certain
    sysfs files whose kernel-reported stat size doesn't match the actual
    read size.
    """
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
        temp_path = entry / "temp"
        zones[type_str] = temp_path
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


def collect() -> dict[str, Any]:
    """Read all sysfs sensors and return a board-health JSON dict."""
    components: list[dict[str, Any]] = []
    thermal_zones = scan_thermal_zones()
    hwmon_devices = scan_hwmon()

    # Thermal zone temperatures
    for zone_type, temp_path in sorted(thermal_zones.items()):
        raw = read_sysfs(temp_path)
        temp_c = -1.0
        if raw is not None:
            try:
                temp_c = float(raw) / 1000.0
            except ValueError:
                pass
        # Strip "-thermal" suffix to match threshold rule names
        name = zone_type.removesuffix("-thermal")
        components.append({
            "name": name,
            "temperature": temp_c,
            "voltage": -1.0,
            "current": -1.0,
            "battery_percent": -1.0,
        })

    # NVMe temperature
    nvme_temp = read_nvme_temp(hwmon_devices)
    if nvme_temp >= 0:
        components.append({
            "name": "nvme",
            "temperature": nvme_temp,
            "voltage": -1.0,
            "current": -1.0,
            "battery_percent": -1.0,
        })

    # System voltage
    voltage = read_voltage(hwmon_devices)

    return {
        "battery_percent": -1.0,
        "voltage": voltage,
        "charging": False,
        "remaining_s": -1,
        "components": components,
    }


def main() -> None:
    for line in sys.stdin:
        try:
            cmd = json.loads(line.strip())
        except json.JSONDecodeError:
            continue
        if cmd.get("cmd") == "collect":
            try:
                result = collect()
            except Exception:
                traceback.print_exc(file=sys.stderr)
                result = {
                    "battery_percent": -1.0,
                    "voltage": -1.0,
                    "charging": False,
                    "remaining_s": -1,
                    "components": [],
                }
            print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
