# SPDX-License-Identifier: MulanPSL-2.0
"""Observation-aware Scene object lifecycle regression tests."""

import asyncio
import threading
from types import SimpleNamespace

import numpy as np

from scene_service.ingest.perception_concept_graphs import (
    ConceptGraphsDetector,
    _observed_map_object_uuids,
    _visible_missing_uuids,
)
from scene_service.state import ObjectRegistry


def _snapshot(uuid: str = "cg-1") -> dict:
    return {
        "uuid": uuid,
        "cls": "cup",
        "x": 1.0,
        "y": 2.0,
        "z": 0.8,
        "yaw": 0.0,
        "size_x": 0.1,
        "size_y": 0.1,
        "size_z": 0.2,
        "confidence": 0.9,
    }


def _detector(registry: ObjectRegistry) -> ConceptGraphsDetector:
    detector = ConceptGraphsDetector.__new__(ConceptGraphsDetector)
    detector._registry = registry
    detector._uuid_to_oid = {}
    detector._expired_uuids = set()
    detector._missing_uuids = set()
    detector._world_frame_fn = lambda: "map"
    detector._object_ttl_s = 30.0
    detector._visible_miss_threshold = 3
    detector._inference_lock = threading.Lock()
    detector._cg = None
    detector._map_objects = []
    detector.cfg = {"max_merge_dist_m": 1.5}
    return detector


def test_historical_projection_does_not_refresh_positive_observation() -> None:
    async def scenario() -> None:
        registry = ObjectRegistry(grace_period_s=0.1)
        detector = _detector(registry)

        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids={"cg-1"},
            visible_miss_uuids=set(),
            frame_seq=1,
            observed_at=100.0,
        )
        obj = next(iter(registry._objects.values()))
        oid = obj.object_id
        assert obj.last_seen == 100.0
        assert obj.observation_count == 1

        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids=set(),
            visible_miss_uuids=set(),
            frame_seq=2,
            observed_at=101.0,
        )
        obj = registry._objects[oid]
        assert obj.last_seen == 100.0
        assert obj.observation_count == 1
        assert not obj.missing

        # Observation-aware objects do not become missing merely because the
        # sensor/model stopped producing healthy frames.
        async with registry.lock():
            assert registry.mark_stale(200.0) == 0
        assert not registry._objects[oid].missing

    asyncio.run(scenario())


def test_only_repeated_healthy_visible_misses_mark_missing() -> None:
    async def scenario() -> None:
        registry = ObjectRegistry()
        detector = _detector(registry)
        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids={"cg-1"},
            visible_miss_uuids=set(),
            frame_seq=1,
            observed_at=100.0,
        )
        oid = next(iter(registry._objects))

        for frame_seq in (2, 3):
            await detector._apply_snapshot(
                [_snapshot()],
                observed_uuids=set(),
                visible_miss_uuids={"cg-1"},
                frame_seq=frame_seq,
                observed_at=100.0 + frame_seq,
            )
            assert not registry._objects[oid].missing

        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids=set(),
            visible_miss_uuids={"cg-1"},
            frame_seq=4,
            observed_at=104.0,
        )
        obj = registry._objects[oid]
        assert obj.missing
        assert detector._missing_uuids == {"cg-1"}
        assert obj.attributes["consecutive_visible_misses"] == 3
        assert obj.last_seen == 100.0

        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids={"cg-1"},
            visible_miss_uuids=set(),
            frame_seq=5,
            observed_at=105.0,
        )
        obj = registry._objects[oid]
        assert not obj.missing
        assert detector._missing_uuids == set()
        assert obj.object_id == oid
        assert obj.observation_count == 2
        assert obj.last_seen == 105.0
        assert obj.attributes["consecutive_visible_misses"] == 0

    asyncio.run(scenario())


def test_current_frame_membership_is_distinct_from_historical_map_membership() -> None:
    objects = [
        {"id": "old", "image_idx": [1, 2]},
        {"id": "matched", "image_idx": [1, 7]},
        {"id": "new", "image_idx": [7]},
        {"id": "", "image_idx": [7]},
    ]
    assert _observed_map_object_uuids(objects, frame_seq=7) == {"matched", "new"}


def test_depth_visibility_requires_clear_line_of_sight() -> None:
    obj = {
        "id": "cg-1",
        "pcd": SimpleNamespace(
            points=np.asarray(
                [
                    [-0.05, -0.05, 2.0],
                    [0.05, -0.05, 2.0],
                    [-0.05, 0.05, 2.0],
                    [0.05, 0.05, 2.0],
                ],
                dtype=np.float32,
            )
        ),
    }
    intrinsics = SimpleNamespace(fx=100.0, fy=100.0, cx=2.0, cy=2.0)
    camera_to_world = np.eye(4, dtype=np.float32)

    removed_depth = np.full((5, 5), 4.0, dtype=np.float32)
    assert _visible_missing_uuids(
        [obj],
        observed_uuids=set(),
        depth_m=removed_depth,
        intrinsics=intrinsics,
        camera_to_world=camera_to_world,
        depth_margin_m=0.2,
    ) == {"cg-1"}

    occluded_depth = np.full((5, 5), 1.0, dtype=np.float32)
    assert not _visible_missing_uuids(
        [obj],
        observed_uuids=set(),
        depth_m=occluded_depth,
        intrinsics=intrinsics,
        camera_to_world=camera_to_world,
        depth_margin_m=0.2,
    )

    present_depth = np.full((5, 5), 2.05, dtype=np.float32)
    assert not _visible_missing_uuids(
        [obj],
        observed_uuids=set(),
        depth_m=present_depth,
        intrinsics=intrinsics,
        camera_to_world=camera_to_world,
        depth_margin_m=0.2,
    )


def test_epoch_reset_drops_map_objects_and_uuid_bindings() -> None:
    async def scenario() -> None:
        detector = _detector(ObjectRegistry())
        detector._map_objects = [{"id": "cg-1"}]
        detector._uuid_to_oid["cg-1"] = "scene.object.cup_001"
        detector._tick_idx = 9
        await detector.reset_derived_state()
        assert detector._map_objects == []
        assert detector._uuid_to_oid == {}
        assert detector._tick_idx == 0

    asyncio.run(scenario())


def test_ttl_prunes_registry_binding_and_historical_map_object() -> None:
    async def scenario() -> None:
        registry = ObjectRegistry()
        detector = _detector(registry)
        detector._visible_miss_threshold = 1
        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids={"cg-1"},
            visible_miss_uuids=set(),
            frame_seq=1,
            observed_at=100.0,
        )
        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids=set(),
            visible_miss_uuids={"cg-1"},
            frame_seq=2,
            observed_at=101.0,
        )
        detector._map_objects = [{"id": "cg-1"}]
        await detector._apply_snapshot(
            [_snapshot()],
            observed_uuids=set(),
            visible_miss_uuids=set(),
            frame_seq=3,
            observed_at=131.0,
        )
        assert registry._objects == {}
        assert detector._uuid_to_oid == {}
        assert detector._expired_uuids == {"cg-1"}
        detector._purge_expired_map_objects_locked()
        assert detector._map_objects == []

    asyncio.run(scenario())
