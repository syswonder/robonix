# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for Scene RGB-D geometry admission and robust boxes."""

import math
from types import SimpleNamespace

import numpy as np
import pytest

from scene_service.ingest.perception_concept_graphs import (
    _occupancy_contains_points,
    _refine_masks_with_depth,
    _robust_yaw_bbox,
)


def test_depth_refinement_removes_mask_edge_background() -> None:
    masks = np.ones((1, 7, 7), dtype=bool)
    depth = np.full((7, 7), 5.0, dtype=np.float32)
    depth[1:6, 1:6] = 2.0
    refined = _refine_masks_with_depth(
        masks,
        depth,
        erosion_px=1,
        min_depth_m=0.15,
        max_depth_m=6.0,
        mad_scale=3.5,
        min_band_m=0.12,
        min_points=4,
    )
    assert refined.shape == masks.shape
    assert int(refined.sum()) == 25
    assert np.all(refined[0, 1:6, 1:6])
    assert not np.any(refined[0, 0])
    assert not np.any(refined[0, -1])


def test_depth_refinement_preserves_small_valid_mask_when_erosion_erases_it() -> None:
    masks = np.zeros((1, 6, 6), dtype=bool)
    masks[0, 2:4, 2:4] = True
    depth = np.full((6, 6), 2.0, dtype=np.float32)
    refined = _refine_masks_with_depth(
        masks,
        depth,
        erosion_px=1,
        min_depth_m=0.15,
        max_depth_m=6.0,
        mad_scale=3.5,
        min_band_m=0.12,
        min_points=4,
    )
    assert np.array_equal(refined, masks)


def _grid(*, yaw: float = 0.0):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id="map"),
        info=SimpleNamespace(
            resolution=0.5,
            width=20,
            height=10,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-2.0, y=-1.0),
                orientation=SimpleNamespace(
                    x=0.0,
                    y=0.0,
                    z=math.sin(yaw / 2.0),
                    w=math.cos(yaw / 2.0),
                ),
            ),
        ),
    )


def test_occupancy_bounds_reject_far_ghost_and_frame_mismatch() -> None:
    inside = np.asarray([[0.0, 0.0, 0.5], [0.1, -0.1, 0.6]])
    far = np.asarray([[100.0, 100.0, 0.5], [101.0, 100.0, 0.6]])
    assert _occupancy_contains_points(inside, _grid(), expected_frame="map")
    assert not _occupancy_contains_points(far, _grid(), expected_frame="map")
    assert not _occupancy_contains_points(inside, _grid(), expected_frame="odom")


def test_occupancy_bounds_support_rotated_map_origin() -> None:
    # Local grid coordinate (2, 1) rotated +90 degrees and translated by
    # origin (-2, -1) becomes world (-3, 1).
    inside_rotated = np.asarray([[-3.0, 1.0, 0.5], [-3.1, 1.1, 0.6]])
    assert _occupancy_contains_points(
        inside_rotated,
        _grid(yaw=math.pi / 2.0),
        expected_frame="map",
    )


def test_robust_bbox_resists_single_depth_spike() -> None:
    rng = np.random.default_rng(7)
    points = np.column_stack(
        (
            rng.uniform(0.5, 1.5, 500),
            rng.uniform(1.75, 2.25, 500),
            rng.uniform(0.2, 0.8, 500),
        )
    )
    points = np.vstack((points, np.asarray([[30.0, -40.0, 20.0]])))
    result = _robust_yaw_bbox(points)
    assert result is not None
    center, extent, yaw = result
    assert np.linalg.norm(center - np.asarray([1.0, 2.0, 0.5])) < 0.15
    assert sorted(extent[:2]) == pytest.approx([0.45, 0.90], abs=0.12)
    assert extent[2] == pytest.approx(0.54, abs=0.08)
    assert math.isfinite(yaw)
