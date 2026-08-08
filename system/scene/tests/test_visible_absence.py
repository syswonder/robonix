# SPDX-License-Identifier: MulanPSL-2.0
"""Negative-evidence contracts: when Scene may delete a vanished object.

This is the path that fixes objects surviving forever after they are carried
out of the room, and it is the one path where a false positive destroys user-
visible state. The rule it must obey: delete only on *positive* evidence of
absence — the depth image showing clear space materially behind where the
object used to be. Occlusion, a similar depth reading, invalid depth, and
out-of-frame all mean "unknown" and must never delete.
"""

from __future__ import annotations

from types import SimpleNamespace

import numpy as np

from scene_service.ingest.cg_kernels import PointCloud
from scene_service.ingest.perception_concept_graphs import (
    ConceptGraphsDetector,
    _import_cg,
    _visible_missing_uuids,
)
from scene_service.ingest.perception_tuning import PerceptionTuning
from scene_service.state import ObjectRegistry


_INTRINSICS = SimpleNamespace(fx=100.0, fy=100.0, cx=50.0, cy=50.0)
_HEIGHT = _WIDTH = 100
_OBJECT_DEPTH_M = 2.0

# Camera optical frame (x right, y down, z forward) → world (x forward, y left,
# z up), placed at the world origin. Using a real rotation rather than identity
# keeps "vertical extent" a genuinely different axis from depth, which is what
# the upper-sample filter selects on.
_CAMERA_TO_WORLD = np.array(
    [
        [0.0, 0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0, 0.0],
        [0.0, -1.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ],
    dtype=np.float64,
)


def _object(uuid_value: str = "u1"):
    """A 3x3 patch of points 2 m in front of the camera, spanning ~0.2 m.

    Nine distinct projected pixels clear the `min_clear_samples` floor of 3
    with room to spare, so a test that expects "not missing" is failing the
    evidence rule rather than merely running out of samples.
    """
    offsets = (-0.1, 0.0, 0.1)
    camera_points = np.array(
        [[x, y, _OBJECT_DEPTH_M] for x in offsets for y in offsets],
        dtype=np.float64,
    )
    homogeneous = np.column_stack(
        (camera_points, np.ones(len(camera_points)))
    )
    world_points = (_CAMERA_TO_WORLD @ homogeneous.T).T[:, :3]
    return {"id": uuid_value, "pcd": PointCloud(world_points.astype(np.float32))}


def _depth(value: float) -> np.ndarray:
    return np.full((_HEIGHT, _WIDTH), value, dtype=np.float32)


def _missing(depth: np.ndarray, *, observed=frozenset(), **overrides) -> set[str]:
    kwargs = {
        "observed_uuids": set(observed),
        "depth_m": depth,
        "intrinsics": _INTRINSICS,
        "camera_to_world": _CAMERA_TO_WORLD,
        "depth_margin_m": 0.10,
    }
    kwargs.update(overrides)
    return _visible_missing_uuids([_object()], **kwargs)


def test_clear_space_behind_the_object_reports_absence() -> None:
    """The measured surface is 1 m behind the stored cloud: the object is gone."""
    assert _missing(_depth(_OBJECT_DEPTH_M + 1.0)) == {"u1"}


def test_occlusion_never_reports_absence() -> None:
    """Something closer than the object hides it; that is not evidence of absence."""
    assert _missing(_depth(_OBJECT_DEPTH_M - 0.5)) == set()


def test_a_similar_depth_reading_never_reports_absence() -> None:
    """The object is still there and the detector simply missed it this frame."""
    assert _missing(_depth(_OBJECT_DEPTH_M)) == set()


def test_invalid_depth_never_reports_absence() -> None:
    """Zero and non-finite depth are unknown, not empty."""
    assert _missing(_depth(0.0)) == set()
    assert _missing(_depth(np.nan)) == set()
    assert _missing(_depth(np.inf)) == set()


def test_an_object_observed_this_frame_is_never_a_candidate() -> None:
    """Positive evidence outranks the projection test entirely."""
    assert _missing(_depth(_OBJECT_DEPTH_M + 1.0), observed={"u1"}) == set()


def test_absence_requires_both_a_sample_count_and_a_clear_fraction() -> None:
    """One clear pixel in a mostly-occluded footprint must not delete anything.

    The nine projected pixels are split so that only two read as clear. That
    clears neither the absolute floor (3) nor the 60% fraction.
    """
    depth = _depth(_OBJECT_DEPTH_M - 0.5)
    depth[44:47, 44:47] = _OBJECT_DEPTH_M + 1.0
    assert _missing(depth) == set()
    # Raising the requirement above what a fully-clear view supplies also
    # withholds deletion, so the floor is doing the work and not a side effect.
    assert _missing(_depth(_OBJECT_DEPTH_M + 1.0), min_clear_samples=50) == set()


def test_a_cloud_outside_the_frame_reports_nothing() -> None:
    """Behind the camera and out of view are both unknown."""
    behind = np.array(_CAMERA_TO_WORLD)
    behind[:3, :3] = -behind[:3, :3]
    assert (
        _visible_missing_uuids(
            [_object()],
            observed_uuids=set(),
            depth_m=_depth(_OBJECT_DEPTH_M + 1.0),
            intrinsics=_INTRINSICS,
            camera_to_world=behind,
            depth_margin_m=0.10,
        )
        == set()
    )


def test_diagnostics_explain_each_verdict() -> None:
    """Operators need to see why an object was or was not declared absent."""
    diagnostics: dict = {}
    _missing(_depth(_OBJECT_DEPTH_M + 1.0), diagnostics=diagnostics)
    assert diagnostics["u1"]["status"] == "clear_absence"
    assert diagnostics["u1"]["clear_samples"] >= 3

    diagnostics = {}
    _missing(_depth(_OBJECT_DEPTH_M - 0.5), diagnostics=diagnostics)
    assert diagnostics["u1"]["status"] == "insufficient_clear_support"
    assert diagnostics["u1"]["occluded_samples"] >= 3


def _detector() -> ConceptGraphsDetector:
    async def _ignore(_detections):
        return None

    detector = ConceptGraphsDetector(
        rgb_fetcher_msg=lambda: None,
        depth_fetcher_msg=lambda: None,
        camera_info_fetcher=lambda: None,
        on_detections=_ignore,
        registry=ObjectRegistry(),
        tuning=PerceptionTuning(classes=["chair"]),
    )
    detector._cg = _import_cg()
    return detector


def test_expired_uuids_are_purged_from_the_concept_graphs_map() -> None:
    """The step that actually closes the loop.

    Registry pruning alone left the uuid in the persistent MapObjectList, so the
    next tick re-projected the object straight back into the registry and it
    never disappeared. Purging the map is what makes removal stick.
    """
    detector = _detector()
    detector._map_objects = detector._cg["MapObjectList"](
        [{"id": "gone"}, {"id": "kept"}]
    )
    generation = detector._map_generation
    detector._expired_uuids = {"gone"}

    detector._purge_expired_map_objects_locked()

    assert [obj["id"] for obj in detector._map_objects] == ["kept"]
    assert detector._map_generation > generation, (
        "dropping an object is a new map generation; an in-flight cleanup "
        "planned against the old snapshot must not overwrite it"
    )
    assert detector._expired_uuids == set()


def test_purging_nothing_leaves_the_map_generation_alone() -> None:
    """A no-op purge must not invalidate an in-flight background cleanup."""
    detector = _detector()
    detector._map_objects = detector._cg["MapObjectList"]([{"id": "kept"}])
    generation = detector._map_generation
    detector._expired_uuids = set()

    detector._purge_expired_map_objects_locked()

    assert [obj["id"] for obj in detector._map_objects] == ["kept"]
    assert detector._map_generation == generation
