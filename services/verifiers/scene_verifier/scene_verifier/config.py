"""Validated service configuration; no Robonix runtime dependencies."""
from dataclasses import dataclass
import math


def finite_number(value: object, field: str) -> float:
    """Reject coercions, booleans and non-finite JSON numbers."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field} must be a number")
    try:
        result = float(value)
    except OverflowError as exc:
        raise ValueError(f"{field} must be finite") from exc
    if not math.isfinite(result):
        raise ValueError(f"{field} must be finite")
    return result


@dataclass(frozen=True)
class VerifierConfig:
    distance_tolerance_m: float = 0.5
    yaw_tolerance_rad: float = 0.35
    observation_timeout_s: float = 5.0


def parse_config(raw: dict) -> VerifierConfig:
    """Validate lifecycle configuration, applying documented defaults."""
    if not isinstance(raw, dict):
        raise ValueError("config must be an object")
    defaults = VerifierConfig()
    values = {}
    for key in defaults.__dataclass_fields__:
        value = finite_number(raw.get(key, getattr(defaults, key)), key)
        if value <= 0:
            raise ValueError(f"{key} must be positive")
        values[key] = value
    if values["yaw_tolerance_rad"] > math.pi:
        raise ValueError("yaw_tolerance_rad must not exceed pi")
    if values["observation_timeout_s"] >= 60:
        raise ValueError("observation_timeout_s must be less than 60")
    return VerifierConfig(**values)