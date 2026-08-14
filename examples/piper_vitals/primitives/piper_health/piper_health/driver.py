#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0

"""Publish controllable demonstration health for a static Piper body model."""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path
import threading

from robonix_api import Err, Ok, Primitive


piper_health = Primitive(id="piper_health", namespace="robonix/primitive/health")

import health_pb2  # noqa: E402


DEFAULT_CONTROL_FILE = (
    Path(__file__).resolve().parents[3] / ".runtime" / "piper_health.json"
)
KNOWN_FAULT_TARGETS = frozenset(
    {*(f"joint_{index}" for index in range(1, 7)), "gripper"}
)


@dataclass(frozen=True)
class HealthSettings:
    scenario: str = "normal"
    interval_s: float = 0.5
    voltage: float = 24.0
    control_file: Path = DEFAULT_CONTROL_FILE

    @classmethod
    def from_config(cls, config: dict) -> "HealthSettings":
        """Validate lifecycle configuration for the deterministic profile."""
        scenario = str(config.get("scenario", "normal")).strip().lower()
        if scenario != "normal":
            raise ValueError(
                f"unsupported scenario '{scenario}'; only 'normal' is implemented"
            )
        interval_s = float(config.get("interval_s", 0.5))
        if interval_s <= 0:
            raise ValueError("interval_s must be greater than zero")
        voltage = float(config.get("voltage", 24.0))
        if voltage <= 0:
            raise ValueError("voltage must be greater than zero")
        control_file = Path(
            config.get("control_file") or DEFAULT_CONTROL_FILE
        ).expanduser()
        if not control_file.is_absolute():
            control_file = (Path.cwd() / control_file).resolve()
        return cls(
            scenario=scenario,
            interval_s=interval_s,
            voltage=voltage,
            control_file=control_file,
        )


_settings = HealthSettings()
_stop = threading.Event()
_control_error = ""


def normalize_fault_targets(targets) -> frozenset[str]:
    """Validate external target names before they affect health telemetry."""
    if not isinstance(targets, list):
        raise ValueError("fault targets must be a list")
    normalized = frozenset(str(target).strip().lower() for target in targets)
    unsupported = sorted(normalized - KNOWN_FAULT_TARGETS)
    if unsupported:
        raise ValueError(f"unsupported Piper fault target(s): {', '.join(unsupported)}")
    return normalized


def load_fault_targets(control_file: Path) -> frozenset[str]:
    """Read one atomically-written runtime fault profile from disk."""
    if not control_file.exists():
        return frozenset()
    document = json.loads(control_file.read_text(encoding="utf-8"))
    if not isinstance(document, dict):
        raise ValueError("Piper health control file must contain an object")
    mode = str(document.get("mode", "normal")).strip().lower()
    if mode == "normal":
        return frozenset()
    if mode != "fault":
        raise ValueError(f"unsupported Piper health control mode: {mode}")
    targets = normalize_fault_targets(document.get("targets", []))
    if not targets:
        raise ValueError("fault mode requires at least one target")
    return targets


def active_fault_targets(settings: HealthSettings) -> frozenset[str]:
    """Keep the stream alive if a manually edited control file is invalid."""
    global _control_error
    try:
        targets = load_fault_targets(settings.control_file)
    except (OSError, ValueError, json.JSONDecodeError) as exc:
        message = str(exc)
        if message != _control_error:
            print(f"[piper_health] invalid control file: {message}", flush=True)
            _control_error = message
        return frozenset()
    if _control_error:
        print("[piper_health] control file is valid again", flush=True)
        _control_error = ""
    return targets


def _reading(
    name: str,
    *,
    temp_c: float = -1.0,
    voltage: float = -1.0,
    current_a: float = -1.0,
) -> "health_pb2.SensorReading":
    """Create one reading while marking unavailable scalar fields explicitly."""
    return health_pb2.SensorReading(
        name=name,
        temp_c=temp_c,
        voltage=voltage,
        current_a=current_a,
        battery_percent=-1.0,
    )


def _control(name: str, value: float) -> "health_pb2.SensorReading":
    return _reading(name, current_a=value)


def _nominal_actuator(
    component_id: str,
    *,
    temp_c: float,
    voltage: float,
    current_a: float,
    faulted: bool = False,
) -> list["health_pb2.SensorReading"]:
    """Return telemetry and controls for one nominal or injected actuator."""
    return [
        _reading(
            component_id,
            temp_c=86.0 if faulted else temp_c,
            voltage=voltage,
            current_a=current_a * 4.0 if faulted else current_a,
        ),
        _control(f"{component_id}/enabled", 0.0 if faulted else 1.0),
        _control(f"{component_id}/communication_ok", 0.0 if faulted else 1.0),
        _control(f"{component_id}/error", 17.0 if faulted else 0.0),
    ]


def build_health_state(
    settings: HealthSettings,
    fault_targets: frozenset[str] | None = None,
) -> "health_pb2.HealthState":
    """Build one frame matching every component in the Piper Soma YAML."""
    faults = fault_targets or frozenset()
    readings = [
        _reading("body", temp_c=34.0),
        _control("body/online", 1.0),
        _control("body/error", 0.0),
        _reading("body/arm", temp_c=35.0),
        _control("body/arm/online", 1.0),
        _control("body/arm/error", 0.0),
    ]
    for joint_index in range(1, 7):
        readings.extend(
            _nominal_actuator(
                f"body/arm/joint_{joint_index}",
                temp_c=36.0 + joint_index * 0.5,
                voltage=settings.voltage,
                current_a=0.25 + joint_index * 0.02,
                faulted=f"joint_{joint_index}" in faults,
            )
        )
    gripper_faulted = "gripper" in faults
    readings.extend(
        [
            _reading(
                "body/arm/gripper",
                temp_c=82.0 if gripper_faulted else 36.5,
            ),
            _control("body/arm/gripper/online", 0.0 if gripper_faulted else 1.0),
            _control("body/arm/gripper/error", 23.0 if gripper_faulted else 0.0),
        ]
    )
    readings.extend(
        _nominal_actuator(
            "body/arm/gripper/actuator",
            temp_c=36.5,
            voltage=settings.voltage,
            current_a=0.18,
        )
    )
    readings.append(_control("body/state", 0.0))
    return health_pb2.HealthState(
        voltage=settings.voltage,
        charging=False,
        remaining_s=-1,
        readings=readings,
    )


@piper_health.grpc("robonix/primitive/health/state")
def get_health_state(_request) -> "health_pb2.GetHealthState_Response":
    """Return the latest controlled Piper health frame."""
    return health_pb2.GetHealthState_Response(
        state=build_health_state(_settings, active_fault_targets(_settings))
    )


@piper_health.grpc("robonix/primitive/health/stream")
def stream_health_state(_request, context):
    """Yield controlled frames until the consumer disconnects or shutdown begins."""
    while context.is_active() and not _stop.is_set():
        yield build_health_state(_settings, active_fault_targets(_settings))
        _stop.wait(_settings.interval_s)


@piper_health.on_init
def init(config):
    """Apply the lifecycle profile before Soma opens the health stream."""
    global _settings
    try:
        _settings = HealthSettings.from_config(config)
    except (TypeError, ValueError) as exc:
        return Err(str(exc))
    _stop.clear()
    print(
        "[piper_health] initialized "
        f"scenario={_settings.scenario} interval_s={_settings.interval_s} "
        f"control_file={_settings.control_file}",
        flush=True,
    )
    return Ok()


@piper_health.on_shutdown
def shutdown():
    """Stop active server-stream iterators before provider teardown."""
    _stop.set()
    return Ok()


if __name__ == "__main__":
    piper_health.run()
