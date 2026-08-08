# SPDX-License-Identifier: MulanPSL-2.0
"""Concurrency and transactional-apply tests for periodic Scene cleanup."""

from __future__ import annotations

import concurrent.futures
import copy
import threading
import time
from types import MethodType

import numpy as np

from scene_service.ingest.cg_kernels import PointCloud
from scene_service.ingest.perception_concept_graphs import (
    ConceptGraphsDetector,
    _import_cg,
    _periodic_object_state_signature,
)
from scene_service.ingest.perception_tuning import PerceptionTuning
from scene_service.state import ObjectRegistry


def _map_object(
    uuid_value: str,
    *,
    x_offset: float = 0.0,
    scale: float = 1.0,
    frames: tuple[int, ...] = (1,),
    operator_label: str = "",
) -> dict:
    axis = (
        np.asarray((-0.04, 0.0, 0.04), dtype=np.float32)
        * float(scale)
    )
    points = np.stack(
        np.meshgrid(axis, axis, axis, indexing="ij"),
        axis=-1,
    ).reshape((-1, 3))
    points[:, 0] += float(x_offset)
    obj = {
        "id": uuid_value,
        "pcd": PointCloud(points),
        "clip_ft": np.asarray((1.0, 0.0), dtype=np.float32),
        "image_idx": list(frames),
        "num_detections": len(frames),
        "class_name": "chair",
    }
    if operator_label:
        obj["operator_label"] = operator_label
    return obj


async def _ignore(_detections) -> None:
    return None


def _minimal_detector(objects=None) -> ConceptGraphsDetector:
    """Build a real detector wired to stub I/O, then swap in a test map.

    Constructing through `__init__` rather than `__new__` keeps the fixture
    honest: every counter, cache, and tuning attribute the cleanup path touches
    comes from the production initialiser, so a renamed attribute fails here
    instead of silently reading a stale one the fixture happened to set.
    """
    detector = ConceptGraphsDetector(
        rgb_fetcher_msg=lambda: None,
        depth_fetcher_msg=lambda: None,
        camera_info_fetcher=lambda: None,
        on_detections=_ignore,
        registry=ObjectRegistry(),
        tuning=PerceptionTuning(classes=["chair"], scale_aware_geometry=False),
        # Cleanup cadence and merge gates tightened so a two-object fixture
        # exercises a full plan/apply cycle on the first tick.
        cfg_overrides={
            "denoise_interval_ticks": 1,
            "merge_overlap_interval_ticks": 1,
            "downsample_voxel_size": 0.025,
            "dbscan_remove_noise": False,
            "dbscan_eps": 0.1,
            "dbscan_min_points": 2,
            "spatial_sim_type": "overlap",
            "merge_overlap_thresh": 0.5,
            "merge_visual_sim_thresh": 0.8,
            "merge_text_sim_thresh": 0.8,
            "max_merge_dist_m": 0.5,
            "adaptive_merge_distance": True,
            "adaptive_merge_min_dist_m": 0.06,
            "adaptive_merge_extent_scale": 1.0,
            "association_max_extent_ratio": 2.0,
            "obj_min_points": 10,
            "obj_min_detections": 1,
        },
    )
    detector._cg = _import_cg()
    if objects is None:
        detector._map_objects = [{"id": "original"}]
    else:
        detector._map_objects = detector._cg["MapObjectList"](objects)
    # `start()` normally owns these; the tests drive the cleanup path directly.
    detector._cleanup_executor = concurrent.futures.ThreadPoolExecutor(
        max_workers=1,
    )
    detector._stop = threading.Event()
    detector._map_generation = 7
    detector._tick_idx = 1
    detector._stabilize_map_labels = lambda: None
    return detector


def _wait_for_callback(detector: ConceptGraphsDetector) -> None:
    deadline = time.monotonic() + 2.0
    while detector._cleanup_future is not None:
        if time.monotonic() >= deadline:
            raise AssertionError("cleanup callback did not publish its result")
        time.sleep(0.005)


def _install_blocking_planner(
    detector: ConceptGraphsDetector,
    started: threading.Event,
    release: threading.Event,
    *,
    merge_plan: list[dict] | None = None,
    object_cleanup_plan: list[dict] | None = None,
) -> None:
    def compute(
        self,
        snapshot,
        run_denoise,
        run_merge,
        generation,
        started_at,
        uuid_to_oid,
    ):
        del self, started_at, uuid_to_oid
        started.set()
        assert release.wait(timeout=2.0)
        objects_before = len(snapshot)
        snapshot.append({"id": "stale-shadow"})
        return {
            "generation": generation,
            "objects": snapshot,
            "ran": True,
            "run_denoise": run_denoise,
            "run_merge": run_merge,
            "objects_before": objects_before,
            "objects_after": len(snapshot),
            "duration_ms": 10.0,
            "quality_counters": {
                "periodic_merge_candidate_pairs": len(merge_plan or ()),
                "periodic_merge_selected_pairs": len(merge_plan or ()),
                "periodic_merge_deferred_pairs": 0,
            },
            "merge_gate_diagnostics": {"planned": True},
            "coobserved_merge_admissions": [],
            "coobserved_merge_events": [],
            "merge_plan": list(merge_plan or ()),
            "object_cleanup_plan": list(object_cleanup_plan or ()),
        }

    detector._compute_periodic_cleanup_snapshot = MethodType(
        compute,
        detector,
    )


def _plan(left: str = "a", right: str = "b") -> list[dict]:
    return [
        {
            "left_uuid": left,
            "right_uuid": right,
            "snapshot_spatial_similarity": 1.0,
            "snapshot_visual_similarity": 1.0,
        }
    ]


def _geometry_update_plan(detector, obj: dict, point_count: int) -> list[dict]:
    updated = copy.deepcopy(obj)
    points = np.asarray(updated["pcd"].points)[:point_count]
    updated["pcd"] = PointCloud(points)
    updated["bbox"] = detector._cg["get_bounding_box"](
        detector.cfg["spatial_sim_type"],
        updated["pcd"],
    )
    updated["n_points"] = len(updated["pcd"])
    return [
        {
            "uuid": str(obj["id"]),
            "action": "update_geometry",
            "input_signature": _periodic_object_state_signature(obj),
            "geometry": {
                key: copy.deepcopy(updated[key])
                for key in ("pcd", "bbox", "n_points")
            },
        }
    ]


def test_cleanup_planning_releases_lock_and_applies_current_snapshot() -> None:
    detector = _minimal_detector()
    started = threading.Event()
    release = threading.Event()
    _install_blocking_planner(detector, started, release)
    try:
        with detector._inference_lock:
            detector._schedule_periodic_cleanup_locked()
        assert started.wait(timeout=1.0)

        # Heavy planning is outside the sensor transaction.
        assert detector._inference_lock.acquire(timeout=0.2)
        detector._inference_lock.release()

        release.set()
        _wait_for_callback(detector)
        assert [obj["id"] for obj in detector._map_objects] == [
            "original",
            "stale-shadow",
        ]
        assert detector._quality_counters["periodic_cleanup_applied"] == 1
        assert detector._map_generation == 8
        assert detector._cleanup_registry_projection_pending is True
        assert (
            detector._periodic_cleanup_recent[-1]["outcome"]
            == "applied_current"
        )
    finally:
        release.set()
        detector._cleanup_executor.shutdown(wait=True)


def test_unrelated_new_object_preserved_while_safe_pair_revalidated() -> None:
    detector = _minimal_detector(
        [
            _map_object("a", frames=(1, 2, 3)),
            _map_object("b", x_offset=0.005, frames=(4,)),
        ]
    )
    detector._uuid_to_oid = {"a": "scene.object.chair_001"}
    started = threading.Event()
    release = threading.Event()
    _install_blocking_planner(
        detector,
        started,
        release,
        merge_plan=_plan(),
    )
    try:
        with detector._inference_lock:
            detector._schedule_periodic_cleanup_locked()
        assert started.wait(timeout=1.0)
        with detector._inference_lock:
            detector._map_objects.append(
                _map_object("new-observation", x_offset=2.0)
            )
            detector._map_generation += 1
        release.set()
        _wait_for_callback(detector)

        assert [obj["id"] for obj in detector._map_objects] == [
            "a",
            "new-observation",
        ]
        assert detector._map_generation == 9
        assert detector._quality_counters["periodic_cleanup_applied"] == 1
        assert (
            detector._quality_counters[
                "periodic_cleanup_revalidated_applied_pairs"
            ]
            == 1
        )
        assert detector._quality_counters[
            "periodic_cleanup_discarded_stale"
        ] == 0
        recent = detector._periodic_cleanup_recent[-1]
        assert recent["outcome"] == "applied_revalidated"
        assert recent["stale_snapshot_ignored"] is True
        assert recent["revalidation"]["applied_pairs"] == 1
        assert "stale-shadow" not in {
            obj["id"] for obj in detector._map_objects
        }
    finally:
        release.set()
        detector._cleanup_executor.shutdown(wait=True)


def test_changed_participant_fails_live_revalidation_without_overwrite() -> None:
    detector = _minimal_detector(
        [_map_object("a"), _map_object("b", x_offset=0.005)]
    )
    started = threading.Event()
    release = threading.Event()
    _install_blocking_planner(
        detector,
        started,
        release,
        merge_plan=_plan(),
    )
    try:
        with detector._inference_lock:
            detector._schedule_periodic_cleanup_locked()
        assert started.wait(timeout=1.0)
        with detector._inference_lock:
            detector._map_objects[1] = _map_object("b", x_offset=1.0)
            detector._map_generation += 1
        release.set()
        _wait_for_callback(detector)

        assert [obj["id"] for obj in detector._map_objects] == ["a", "b"]
        assert detector._map_generation == 8
        recent = detector._periodic_cleanup_recent[-1]
        assert recent["outcome"] == "revalidated_noop"
        pair = recent["revalidation"]["pairs"][0]
        assert pair["reason"] in {"distance_gate", "spatial_gate"}
        assert pair["applied"] is False
        assert detector._cleanup_registry_projection_pending is False
        assert detector._cleanup_stale_streak == 0
        assert detector._cleanup_not_before_tick == 0
        assert "stale-shadow" not in {
            obj["id"] for obj in detector._map_objects
        }
    finally:
        release.set()
        detector._cleanup_executor.shutdown(wait=True)


def test_operator_label_added_during_planning_protects_participant() -> None:
    detector = _minimal_detector(
        [_map_object("a"), _map_object("b", x_offset=0.005)]
    )
    started = threading.Event()
    release = threading.Event()
    _install_blocking_planner(
        detector,
        started,
        release,
        merge_plan=_plan(),
    )
    try:
        with detector._inference_lock:
            detector._schedule_periodic_cleanup_locked()
        assert started.wait(timeout=1.0)
        with detector._inference_lock:
            detector._map_objects[1]["operator_label"] = "keep separate"
            detector._map_generation += 1
        release.set()
        _wait_for_callback(detector)

        assert [obj["id"] for obj in detector._map_objects] == ["a", "b"]
        recent = detector._periodic_cleanup_recent[-1]
        assert recent["outcome"] == "revalidated_noop"
        assert (
            recent["revalidation"]["pairs"][0]["reason"]
            == "operator_label_protected"
        )
        assert (
            detector._quality_counters[
                "periodic_cleanup_revalidated_skipped_pairs"
            ]
            == 1
        )
    finally:
        release.set()
        detector._cleanup_executor.shutdown(wait=True)


def test_operator_geometry_added_during_planning_protects_participant() -> None:
    detector = _minimal_detector(
        [_map_object("a"), _map_object("b", x_offset=0.005)]
    )
    detector._uuid_to_oid = {
        "a": "scene.object.chair_001",
        "b": "scene.object.chair_002",
    }
    started = threading.Event()
    release = threading.Event()
    _install_blocking_planner(
        detector,
        started,
        release,
        merge_plan=_plan(),
    )
    try:
        with detector._inference_lock:
            detector._schedule_periodic_cleanup_locked()
        assert started.wait(timeout=1.0)
        generation_before = detector._map_generation
        assert detector._update_object_geometry_override_locked(
            "scene.object.chair_002"
        )
        assert detector._map_generation == generation_before + 1
        release.set()
        _wait_for_callback(detector)

        assert [obj["id"] for obj in detector._map_objects] == ["a", "b"]
        recent = detector._periodic_cleanup_recent[-1]
        assert recent["outcome"] == "revalidated_noop"
        assert (
            recent["revalidation"]["pairs"][0]["reason"]
            == "operator_geometry_protected"
        )
        generation_before_clear = detector._map_generation
        assert detector._clear_object_geometry_override_locked(
            "scene.object.chair_002"
        )
        assert detector._map_generation == generation_before_clear + 1
    finally:
        release.set()
        detector._cleanup_executor.shutdown(wait=True)


def test_periodic_plan_rejects_distance_and_extent_before_merge() -> None:
    cases = (
        (
            [_map_object("a"), _map_object("b", x_offset=1.0)],
            "distance_gate",
        ),
        (
            [_map_object("a"), _map_object("b", scale=4.0)],
            "extent_gate",
        ),
    )
    for objects, reason in cases:
        detector = _minimal_detector(objects)
        detector._voxel_pcd_overlap_matrix = MethodType(
            lambda self, objects_a, objects_b=None, voxel_size=None: (
                np.asarray(((0.0, 1.0), (1.0, 0.0)))
            ),
            detector,
        )
        try:
            assert detector._run_periodic_cleanup_sync(
                run_denoise=False,
                run_merge=True,
                project_registry=False,
            )
            assert [obj["id"] for obj in detector._map_objects] == ["a", "b"]
            assert detector._periodic_merge_plan == []
            diagnostics = detector._merge_gate_diagnostics
            assert diagnostics["periodic_candidate_pairs"] == 1
            assert diagnostics["periodic_selected_pairs"] == 0
            assert diagnostics["periodic_physical_rejected_pairs"] == 1
            assert (
                diagnostics["periodic_physical_rejections"][0]["reason"]
                == reason
            )
        finally:
            detector._cleanup_executor.shutdown(wait=True)


def test_cleanup_backoff_still_skips_work_until_eligible_tick() -> None:
    detector = _minimal_detector()
    detector._cleanup_stale_streak = 3
    detector._cleanup_not_before_tick = 20
    detector._tick_idx = 10
    try:
        with detector._inference_lock:
            detector._schedule_periodic_cleanup_locked()

        assert detector._cleanup_future is None
        assert detector._quality_counters["periodic_cleanup_scheduled"] == 0
        assert (
            detector._quality_counters[
                "periodic_cleanup_skipped_backoff"
            ]
            == 1
        )
    finally:
        detector._cleanup_executor.shutdown(wait=True)
