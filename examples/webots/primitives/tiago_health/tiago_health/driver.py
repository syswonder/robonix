#!/usr/bin/env python3
# SPDX-License-Identifier: MulanPSL-2.0
"""Publish deterministic nominal health for the Webots TIAGo deployment."""

from __future__ import annotations

from dataclasses import dataclass
import threading

from robonix_api import Err, Ok, Primitive


tiago_health = Primitive(id="tiago_health", namespace="robonix/primitive/health")

import health_pb2  # noqa: E402


@dataclass(frozen=True)
class HealthSettings:
    scenario: str = "normal"
    variant: str = "lite"
    interval_s: float = 0.5
    battery_percent: float = 82.0
    voltage: float = 24.8
    remaining_s: int = 10800

    @classmethod
    def from_config(cls, cfg: dict) -> "HealthSettings":
        """Validate lifecycle config and return one immutable runtime setup."""
        scenario = str(cfg.get("scenario", "normal")).strip().lower()
        if scenario != "normal":
            raise ValueError(
                f"unsupported scenario '{scenario}'; only 'normal' is implemented"
            )
        variant = str(cfg.get("variant", "lite")).strip().lower() or "lite"
        if variant not in {"lite", "full"}:
            raise ValueError(
                f"unsupported TIAGo variant '{variant}'; choose 'lite' or 'full'"
            )
        interval_s = float(cfg.get("interval_s", 0.5))
        if interval_s <= 0:
            raise ValueError("interval_s must be greater than zero")
        battery_percent = float(cfg.get("battery_percent", 82.0))
        if not 0.0 <= battery_percent <= 100.0:
            raise ValueError("battery_percent must be between 0 and 100")
        voltage = float(cfg.get("voltage", 24.8))
        remaining_s = int(cfg.get("remaining_s", 10800))
        return cls(
            scenario=scenario,
            variant=variant,
            interval_s=interval_s,
            battery_percent=battery_percent,
            voltage=voltage,
            remaining_s=remaining_s,
        )


_settings = HealthSettings()
_stop = threading.Event()


def _reading(
    name: str,
    *,
    temp_c: float = -1.0,
    voltage: float = -1.0,
    current_a: float = -1.0,
    battery_percent: float = -1.0,
) -> "health_pb2.SensorReading":
    """Create one reading with explicit unavailable sentinels."""
    return health_pb2.SensorReading(
        name=name,
        temp_c=temp_c,
        voltage=voltage,
        current_a=current_a,
        battery_percent=battery_percent,
    )


def _control(name: str, value: float) -> "health_pb2.SensorReading":
    return _reading(name, current_a=value)


def _full_variant_readings(settings: HealthSettings) -> list["health_pb2.SensorReading"]:
    """Return nominal arm and gripper readings for the full Webots model."""
    readings = [
        _reading("body/arm", temp_c=37.0),
        _control("body/arm/online", 1.0),
        _control("body/arm/error", 0.0),
    ]
    for joint_index in range(1, 8):
        component_id = f"body/arm/joint_{joint_index}"
        readings.extend(
            [
                _reading(
                    component_id,
                    temp_c=38.0 + joint_index * 0.4,
                    voltage=settings.voltage,
                    current_a=0.35,
                ),
                _control(f"{component_id}/enabled", 1.0),
                _control(f"{component_id}/communication_ok", 1.0),
                _control(f"{component_id}/error", 0.0),
            ]
        )
    readings.extend(
        [
            _reading("body/arm/gripper", temp_c=38.0),
            _control("body/arm/gripper/online", 1.0),
            _control("body/arm/gripper/error", 0.0),
            _reading(
                "body/arm/gripper/actuator",
                temp_c=38.0,
                voltage=settings.voltage,
                current_a=0.2,
            ),
            _control("body/arm/gripper/actuator/enabled", 1.0),
            _control("body/arm/gripper/actuator/communication_ok", 1.0),
            _control("body/arm/gripper/actuator/error", 0.0),
        ]
    )
    return readings


def build_health_state(settings: HealthSettings) -> "health_pb2.HealthState":
    """Build one self-contained nominal frame using Soma component paths."""
    readings = [
        _reading("body", temp_c=36.0),
        _reading("body/base", temp_c=34.0),
        _reading(
            "body/base/left_wheel",
            temp_c=38.0,
            voltage=settings.voltage,
            current_a=0.7,
        ),
        _reading("body/base/left_wheel/driver_temp", temp_c=41.0),
        _control("body/base/left_wheel/enabled", 1.0),
        _control("body/base/left_wheel/communication_ok", 1.0),
        _control("body/base/left_wheel/error", 0.0),
        _reading(
            "body/base/right_wheel",
            temp_c=38.5,
            voltage=settings.voltage,
            current_a=0.7,
        ),
        _reading("body/base/right_wheel/driver_temp", temp_c=41.5),
        _control("body/base/right_wheel/enabled", 1.0),
        _control("body/base/right_wheel/communication_ok", 1.0),
        _control("body/base/right_wheel/error", 0.0),
        _reading(
            "body/base/battery",
            temp_c=32.0,
            voltage=settings.voltage,
            current_a=0.0,
            battery_percent=settings.battery_percent,
        ),
        _control("body/base/battery/online", 1.0),
        _control("body/base/battery/error", 0.0),
        _reading("body/head_camera", temp_c=42.0),
        _control("body/head_camera/online", 1.0),
        _control("body/head_camera/error", 0.0),
        _reading("body/hokuyo_lidar", temp_c=39.0),
        _control("body/hokuyo_lidar/online", 1.0),
        _control("body/hokuyo_lidar/error", 0.0),
        _reading("body/audio", temp_c=35.0),
        _control("body/audio/online", 1.0),
        _control("body/audio/error", 0.0),
    ]
    if settings.variant == "full":
        readings.extend(_full_variant_readings(settings))
    readings.append(_control("body/state", 0.0))
    return health_pb2.HealthState(
        voltage=settings.voltage,
        charging=False,
        remaining_s=settings.remaining_s,
        readings=readings,
    )


@tiago_health.grpc("robonix/primitive/health/state")
def get_health_state(_request) -> "health_pb2.GetHealthState_Response":
    """Return the latest deterministic simulated health frame."""
    return health_pb2.GetHealthState_Response(state=build_health_state(_settings))


@tiago_health.grpc("robonix/primitive/health/stream")
def stream_health_state(_request, context):
    """Yield nominal health at the configured interval until disconnected."""
    while context.is_active() and not _stop.is_set():
        yield build_health_state(_settings)
        _stop.wait(_settings.interval_s)


@tiago_health.on_init
def init(cfg):
    """Load the simulated profile before Soma opens the health stream."""
    global _settings
    try:
        _settings = HealthSettings.from_config(cfg)
    except (TypeError, ValueError) as exc:
        return Err(str(exc))
    _stop.clear()
    print(
        "[tiago_health] initialized "
        f"scenario={_settings.scenario} variant={_settings.variant} "
        f"interval_s={_settings.interval_s}",
        flush=True,
    )
    return Ok()


@tiago_health.on_shutdown
def shutdown():
    """Stop active server-stream iterators before provider teardown."""
    _stop.set()
    return Ok()


if __name__ == "__main__":
    tiago_health.run()
