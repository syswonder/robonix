# SPDX-License-Identifier: MulanPSL-2.0
"""Deterministic Scene perception deployment profiles."""

from __future__ import annotations

from typing import Any, Mapping


_PROFILES: dict[str, dict[str, Any]] = {
    "full": {
        "enabled": True,
        "period_s": 0.6,
        "input_size": 640,
        "max_detections": 30,
        "stationary_input_size": 640,
        "stationary_max_detections": 8,
        "inference_precision": "auto",
        "tensor_rt": "off",
        "yolo_weights_path": "/opt/models/yolov8l-world-baked.safetensors",
    },
    "jetson": {
        "enabled": True,
        "period_s": 1.0,
        "input_size": 512,
        "max_detections": 20,
        "stationary_input_size": 512,
        "stationary_max_detections": 4,
        "inference_precision": "auto",
        "tensor_rt": "auto",
        "yolo_weights_path": "/opt/models/yolov8s-worldv2-baked.safetensors",
    },
    "lite": {
        "enabled": False,
        "period_s": None,
        "input_size": None,
        "max_detections": 0,
        "stationary_input_size": None,
        "stationary_max_detections": 0,
        "inference_precision": "fp32",
        "tensor_rt": "off",
        "yolo_weights_path": None,
    },
}


def resolve_perception_profile(
    perception_config: Mapping[str, Any],
    *,
    environment_default: str = "",
) -> dict[str, Any]:
    """Resolve full/jetson/lite defaults without overriding explicit knobs."""

    name = str(
        perception_config.get("profile") or environment_default or "full"
    ).strip().lower()
    if name not in _PROFILES:
        raise ValueError(
            "perception.profile must be one of full, jetson, or lite"
        )
    resolved = dict(_PROFILES[name])
    resolved["name"] = name
    return resolved
