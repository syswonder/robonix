# SPDX-License-Identifier: MulanPSL-2.0
"""Scene deployment-profile contracts."""

import pytest

from scene_service.ingest.perception_profiles import resolve_perception_profile


def test_full_profile_preserves_quality_defaults() -> None:
    profile = resolve_perception_profile({})
    assert profile["name"] == "full"
    assert profile["enabled"] is True
    assert profile["input_size"] == 640
    assert profile["max_detections"] == 30
    assert profile["tensor_rt"] == "off"
    assert profile["yolo_weights_path"].endswith(
        "yolov8l-world-baked.safetensors"
    )


def test_jetson_profile_uses_the_small_model_and_bounded_work() -> None:
    profile = resolve_perception_profile(
        {"profile": "jetson"},
        environment_default="full",
    )
    assert profile["input_size"] == 512
    assert profile["period_s"] == 1.0
    assert profile["max_detections"] == 20
    assert profile["tensor_rt"] == "auto"
    assert profile["yolo_weights_path"].endswith(
        "yolov8s-worldv2-baked.safetensors"
    )


def test_lite_profile_has_no_perception_model_path() -> None:
    profile = resolve_perception_profile({"profile": "lite"})
    assert profile["enabled"] is False
    assert profile["yolo_weights_path"] is None
    assert profile["max_detections"] == 0


def test_unknown_profile_is_rejected() -> None:
    with pytest.raises(ValueError, match="full, jetson, or lite"):
        resolve_perception_profile({"profile": "maximum"})
