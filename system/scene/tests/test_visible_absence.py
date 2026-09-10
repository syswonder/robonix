# SPDX-License-Identifier: MulanPSL-2.0
"""Visible-absence eviction: the burden of proof is on deletion.

Synthetic depth frames against a hand-built map object exercise the four
verdicts (observed / occluded / similar-depth / clear absence) plus the
consecutive-miss streak that gates actual removal.
"""
from __future__ import annotations

import numpy as np

from scene_service.ingest.perception_concept_graphs import (
    ConceptGraphsDetector,
    _visible_missing_uuids,
)


class _K:
    fx = fy = 100.0
    cx = cy = 32.0
    width = height = 64


class _Pcd:
    def __init__(self, points):
        self.points = np.asarray(points, dtype=np.float64)


def _map_object(uuid="obj-1"):
    # A shoebox-sized cluster 2 m in front of the camera along world +X,
    # standing ~1 m off the floor. World Z is up (the function's vertical
    # axis), so the object has real vertical extent and the adaptive
    # depth margin behaves as it would on a robot.
    pts = [
        [2.0, dy, 1.0 + dz]
        for dy in (-0.05, 0.0, 0.05)
        for dz in (-0.10, -0.05, 0.0, 0.05, 0.10)
    ]
    return {"id": uuid, "pcd": _Pcd(pts)}


# Camera at the origin looking along world +X: camera z→world x,
# camera x→world -y, camera y→world -z (right-handed, det=+1).
CAM_TO_WORLD = np.array([
    [0.0,  0.0, 1.0, 0.0],
    [-1.0, 0.0, 0.0, 0.0],
    [0.0, -1.0, 0.0, 1.0],
    [0.0,  0.0, 0.0, 1.0],
])


def _depth(value: float) -> np.ndarray:
    return np.full((64, 64), value, dtype=np.float32)


def test_observed_objects_are_never_missing():
    misses = _visible_missing_uuids(
        [_map_object()],
        observed_uuids={"obj-1"},
        depth_m=_depth(10.0),
        intrinsics=_K(),
        camera_to_world=CAM_TO_WORLD,
        depth_margin_m=0.10,
    )
    assert misses == set()


def test_closer_surface_is_occlusion_not_absence():
    # Measured surface at 1.0 m, object stored at 2.0 m: something is in
    # front of it. That is occlusion and must not delete state.
    misses = _visible_missing_uuids(
        [_map_object()],
        observed_uuids=set(),
        depth_m=_depth(1.0),
        intrinsics=_K(),
        camera_to_world=CAM_TO_WORLD,
        depth_margin_m=0.10,
    )
    assert misses == set()


def test_similar_depth_is_a_detector_miss_not_absence():
    misses = _visible_missing_uuids(
        [_map_object()],
        observed_uuids=set(),
        depth_m=_depth(2.02),
        intrinsics=_K(),
        camera_to_world=CAM_TO_WORLD,
        depth_margin_m=0.10,
    )
    assert misses == set()


def test_invalid_depth_is_unknown_not_absence():
    misses = _visible_missing_uuids(
        [_map_object()],
        observed_uuids=set(),
        depth_m=_depth(0.0),   # holes everywhere
        intrinsics=_K(),
        camera_to_world=CAM_TO_WORLD,
        depth_margin_m=0.10,
    )
    assert misses == set()


def test_clear_background_is_absence():
    # Wall at 10 m where the object used to be: clearly gone.
    diagnostics: dict = {}
    misses = _visible_missing_uuids(
        [_map_object()],
        observed_uuids=set(),
        depth_m=_depth(10.0),
        intrinsics=_K(),
        camera_to_world=CAM_TO_WORLD,
        depth_margin_m=0.10,
        diagnostics=diagnostics,
    )
    assert misses == {"obj-1"}
    assert diagnostics["obj-1"]["status"] == "clear_absence"


def _detector_with_map(objs):
    d = ConceptGraphsDetector.__new__(ConceptGraphsDetector)
    d._map_objects = list(objs)
    d._cg = None
    d._tick_idx = 7
    d.cfg = {
        "visibility_depth_margin_m": 0.10,
        "visibility_min_clear_samples": 3,
        "visibility_min_clear_fraction": 0.60,
        "visibility_miss_ticks": 3,
    }
    return d


def test_streak_gates_removal():
    obj = _map_object()
    obj["image_idx"] = [1]          # last matched long ago → unmatched now
    d = _detector_with_map([obj])
    for tick in range(2):
        d._update_visible_absence(depth=_depth(10.0), K=_K(), trans_pose=CAM_TO_WORLD)
        assert len(d._map_objects) == 1, f"removed too early on tick {tick}"
    d._update_visible_absence(depth=_depth(10.0), K=_K(), trans_pose=CAM_TO_WORLD)
    assert len(d._map_objects) == 0


def test_one_observation_resets_the_streak():
    obj = _map_object()
    obj["image_idx"] = [1]
    d = _detector_with_map([obj])
    d._update_visible_absence(depth=_depth(10.0), K=_K(), trans_pose=CAM_TO_WORLD)
    d._update_visible_absence(depth=_depth(10.0), K=_K(), trans_pose=CAM_TO_WORLD)
    #

    obj["image_idx"] = [d._tick_idx]     # matched this tick
    d._update_visible_absence(depth=_depth(10.0), K=_K(), trans_pose=CAM_TO_WORLD)
    assert len(d._map_objects) == 1
    assert d._visible_miss_streak == {}
