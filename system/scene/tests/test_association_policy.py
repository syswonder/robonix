# SPDX-License-Identifier: MulanPSL-2.0
"""Focused regressions for Scene's identity-association safety gates."""

import math

import numpy as np
import pytest

from scene_service.ingest.perception_concept_graphs import (
    _adaptive_association_distance_limit,
    _disjoint_periodic_merge_mask,
    _horizontal_extent_ratio,
    _identity_evidence_mask,
    _is_floor_noise_detection,
    _one_to_one_association_mask,
)


def test_association_is_globally_one_to_one() -> None:
    selected = _one_to_one_association_mask(
        np.asarray([[1.20, 1.19], [1.18, 0.10], [0.20, 0.30]]),
        threshold=0.85,
    )
    assert selected.tolist() == [
        [False, True],
        [True, False],
        [False, False],
    ]


def test_identity_requires_independent_spatial_visual_and_extent_evidence() -> None:
    admitted = _identity_evidence_mask(
        [[0.30, 0.10, 0.35, 0.40]],
        [[0.70, 0.95, 0.55, 0.90]],
        [[1.20, 1.10, 1.30, 8.00]],
        min_spatial_similarity=0.20,
        min_visual_similarity=0.65,
        max_extent_ratio=4.0,
    )
    assert admitted.tolist() == [[True, False, False, False]]


def test_horizontal_extent_accepts_tabletop_view_not_small_object() -> None:
    full_table = (2.28, 2.34, 0.51)
    tabletop = _horizontal_extent_ratio(
        full_table,
        (2.13, 1.94, 0.05),
        extent_floor=0.025,
    )
    monitor = _horizontal_extent_ratio(
        full_table,
        (0.40, 0.42, 0.32),
        extent_floor=0.025,
    )
    assert tabletop < 1.25
    assert monitor > 5.0
    assert not _identity_evidence_mask(
        [[1.0]],
        [[0.99]],
        [[monitor]],
        min_spatial_similarity=0.03,
        min_visual_similarity=0.65,
        max_extent_ratio=4.0,
    )[0, 0]


def test_horizontal_extent_and_distance_fail_closed() -> None:
    assert math.isinf(
        _horizontal_extent_ratio(
            (float("nan"), 1.0, 1.0),
            (1.0, 1.0, 1.0),
            extent_floor=0.025,
        )
    )
    assert _adaptive_association_distance_limit(
        (float("nan"), 1.0),
        (1.0, 1.0),
        minimum_m=0.45,
        maximum_m=1.5,
        extent_scale=0.8,
    ) == pytest.approx(0.45)


def test_periodic_cleanup_selects_disjoint_pairs() -> None:
    selected = _disjoint_periodic_merge_mask(
        np.asarray(
            [[0.0, 0.90, 0.0], [0.90, 0.0, 0.80], [0.0, 0.80, 0.0]]
        ),
        spatial_threshold=0.50,
        visual_scores=np.asarray(
            [[1.0, 0.95, 0.20], [0.95, 1.0, 0.93], [0.20, 0.93, 1.0]]
        ),
        visual_threshold=0.65,
    )
    assert selected.tolist() == [
        [False, True, False],
        [True, False, False],
        [False, False, False],
    ]


def test_floor_filter_rejects_flat_furniture_not_tall_geometry() -> None:
    floor_patch = np.asarray(
        [[x, y, 0.10] for x in (0.0, 0.2) for y in (0.0, 0.2)]
    )
    chair = np.asarray(
        [[0.0, 0.0, z] for z in (0.05, 0.35, 0.65, 0.95)]
    )
    assert _is_floor_noise_detection("chair", floor_patch)
    assert not _is_floor_noise_detection("book", floor_patch)
    assert not _is_floor_noise_detection("chair", chair)
