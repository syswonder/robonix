# SPDX-License-Identifier: MulanPSL-2.0
"""Scene deployment-profile and perception-tuning contracts."""

import pytest

from scene_service.ingest.perception_profiles import resolve_perception_profile
from scene_service.ingest.perception_tuning import (
    CANDIDATE_CLIP_RERANK_GROUPS,
    CANDIDATE_LABEL_ALIASES,
    CANDIDATE_SURFACE_SNAP_LABELS,
    PerceptionTuning,
)


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


def test_label_correction_ships_scoped_to_nothing() -> None:
    """The three correction mechanisms must stay off until they are measured.

    Their F1 / label-accuracy baselines were taken with all three inert, so a
    default that quietly switches one on would invalidate those numbers.
    """
    tuning = PerceptionTuning()
    assert tuning.clip_rerank_groups == []
    assert tuning.clip_rerank_routes == {}
    assert tuning.surface_snap_labels == []
    assert tuning.stationary_refinement is False


def test_candidate_scopes_stay_valid_against_the_shipped_vocabulary() -> None:
    """Adopting a candidate scope must not fail at boot.

    The candidates name labels from `docker/yolo_world_classes.json`. Editing
    that vocabulary without editing them would leave a scope that raises the
    moment an operator copies it into their config, so validate them here.
    """
    tuning = PerceptionTuning(
        clip_rerank_groups=[list(group) for group in CANDIDATE_CLIP_RERANK_GROUPS],
        label_aliases=dict(CANDIDATE_LABEL_ALIASES),
        surface_snap_labels=list(CANDIDATE_SURFACE_SNAP_LABELS),
    )
    vocabulary = set(tuning.resolved_classes)
    scoped = {
        label for group in tuning.clip_rerank_groups for label in group
    }
    assert scoped <= vocabulary
    assert set(tuning.surface_snap_labels) <= vocabulary
    # A label in two groups would make the rerank scope ambiguous.
    flattened = [label for group in CANDIDATE_CLIP_RERANK_GROUPS for label in group]
    assert len(flattened) == len(set(flattened))
