# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for epoch-checked Scene object management."""

import asyncio
import json
from types import SimpleNamespace

import pytest

from scene_service.object_mutations import ObjectMutationCoordinator
from scene_service import mcp_tools
from scene_service.state import BBox3D, ObjectRegistry, Pose3D


class _Detector:
    def __init__(self):
        self.labels = {}
        self.geometry_overrides = set()
        self.deleted = []
        self.reset_count = 0

    async def update_object_label(self, object_id, label):
        self.labels[object_id] = label
        return True

    async def clear_object_label_override(self, object_id):
        self.labels.pop(object_id, None)
        return True

    async def update_object_geometry_override(self, object_id):
        self.geometry_overrides.add(object_id)
        return True

    async def clear_object_geometry_override(self, object_id):
        self.geometry_overrides.discard(object_id)
        return True

    async def delete_object(self, object_id):
        self.deleted.append(object_id)
        self.labels.pop(object_id, None)
        self.geometry_overrides.discard(object_id)
        return True

    async def reset_derived_state(self):
        self.reset_count += 1
        self.labels.clear()
        self.geometry_overrides.clear()


class _Graph:
    def __init__(self):
        self.clear_count = 0

    def clear_derived_state(self):
        self.clear_count += 1


class _Store:
    def __init__(self, *, persist_result=1):
        self.persist_result = persist_result
        self.persist_calls = []
        self.delete_object_calls = []
        self.delete_map_calls = []

    def persist(self, pairs, *, partition=None):
        self.persist_calls.append((pairs, partition))
        return self.persist_result

    def delete_object(self, object_id, *, partition=None):
        self.delete_object_calls.append((object_id, partition))
        return True

    def delete_map(self, partition):
        self.delete_map_calls.append(partition)
        return 1


class _Meta:
    def read(self, map_id):
        if map_id != "office":
            return None
        return SimpleNamespace(object_partition="office__s4")


def _registry():
    registry = ObjectRegistry()
    derived = registry.insert_object(
        "chair",
        Pose3D(1.0, 2.0, 0.5, frame_id="map"),
        BBox3D(0.5, 0.5, 1.0, frame_id="map"),
        0.8,
        10.0,
        source="concept_graphs",
    )
    derived.attributes.update(
        {
            "geometry_source": "rgbd_multi_view",
            "geometry_navigation_grade": True,
            "navigation_grade": True,
        }
    )
    robot = registry.insert_object(
        "robot",
        Pose3D(0.0, 0.0, 0.0, frame_id="map"),
        BBox3D(0.5, 0.5, 0.5, frame_id="map"),
        1.0,
        10.0,
        is_robot=True,
        source="self",
    )
    return registry, derived, robot


def _coordinator(*, persist_result=1):
    registry, derived, robot = _registry()
    detector = _Detector()
    graph = _Graph()
    store = _Store(persist_result=persist_result)
    coordinator = ObjectMutationCoordinator(
        registry=registry,
        detector=detector,
        scene_graph_store=graph,
        live_binding={"map_id": "office", "generation": 7},
        ops_lock=asyncio.Lock(),
        semantic_hold={"reason": None},
        object_store=store,
        map_meta=_Meta(),
    )
    return coordinator, registry, derived, robot, detector, graph, store


def test_epoch_mismatch_fails_before_mutation() -> None:
    coordinator, registry, derived, *_ = _coordinator()
    with pytest.raises(RuntimeError, match="map epoch changed"):
        asyncio.run(
            coordinator.delete_object(
                object_id=derived.object_id,
                expected_map_id="office",
                expected_generation=6,
                persist_to_snapshot=False,
            )
        )
    assert registry.get_object(derived.object_id) is derived


def test_update_label_is_sticky_and_persisted_to_current_snapshot() -> None:
    coordinator, registry, derived, _, detector, graph, store = _coordinator()
    updated, persisted, map_id, generation = asyncio.run(
        coordinator.update_label(
            object_id=derived.object_id,
            label="Coffee Mug",
            expected_map_id="office",
            expected_generation=7,
            persist_to_snapshot=True,
        )
    )
    assert updated is registry.get_object(derived.object_id)
    assert updated.cls == "coffee_mug"
    assert updated.attributes["operator_label"] == "coffee_mug"
    assert updated.attributes["operator_label_previous"]["label"] == "chair"
    assert detector.labels[derived.object_id] == "coffee_mug"
    assert updated.attributes["navigation_grade"] is True
    assert persisted is True
    assert (map_id, generation) == ("office", 7)
    assert store.persist_calls[0][1] == "office__s4"
    assert graph.clear_count == 1


def test_clear_label_override_restores_model_label_and_provenance() -> None:
    coordinator, registry, derived, _, detector, graph, _ = _coordinator()
    asyncio.run(
        coordinator.update_label(
            object_id=derived.object_id,
            label="coffee mug",
            expected_map_id="office",
            expected_generation=7,
            persist_to_snapshot=False,
        )
    )
    restored, persisted, _, _ = asyncio.run(
        coordinator.update_label(
            object_id=derived.object_id,
            label="",
            clear_override=True,
            expected_map_id="office",
            expected_generation=7,
            persist_to_snapshot=False,
        )
    )
    assert restored is registry.get_object(derived.object_id)
    assert restored.cls == "chair"
    assert restored.attributes.get("label_source") == "model"
    assert "operator_label" not in restored.attributes
    assert "operator_label_previous" not in restored.attributes
    assert derived.object_id not in detector.labels
    assert persisted is False
    assert graph.clear_count == 2


def test_failed_label_persistence_rolls_runtime_back() -> None:
    coordinator, registry, derived, _, detector, _, _ = _coordinator(
        persist_result=0
    )
    with pytest.raises(RuntimeError, match="rolled back"):
        asyncio.run(
            coordinator.update_label(
                object_id=derived.object_id,
                label="table",
                expected_map_id="office",
                expected_generation=7,
                persist_to_snapshot=True,
            )
        )
    current = registry.get_object(derived.object_id)
    assert current.cls == "chair"
    assert "operator_label" not in current.attributes
    assert derived.object_id not in detector.labels


def test_update_geometry_is_sticky_provenanced_and_non_navigation_grade() -> None:
    coordinator, registry, derived, _, detector, graph, store = _coordinator()
    updated, persisted, map_id, generation = asyncio.run(
        coordinator.update_geometry(
            object_id=derived.object_id,
            x=3.0,
            y=4.0,
            z=0.6,
            yaw=0.5,
            size_x=0.8,
            size_y=0.4,
            size_z=1.2,
            frame_id="map",
            expected_map_id="office",
            expected_generation=7,
            persist_to_snapshot=True,
        )
    )
    assert updated is registry.get_object(derived.object_id)
    assert (updated.pose.x, updated.pose.y, updated.pose.z) == (3.0, 4.0, 0.6)
    assert updated.bbox.size_x == pytest.approx(0.8)
    assert updated.attributes["operator_geometry"] is True
    assert updated.attributes["geometry_source"] == "operator_bbox"
    assert updated.attributes["geometry_point_count"] == 0
    assert updated.attributes["navigation_grade"] is False
    assert derived.object_id in detector.geometry_overrides
    assert store.persist_calls[0][1] == "office__s4"
    assert persisted is True
    assert (map_id, generation) == ("office", 7)
    assert graph.clear_count == 1


def test_failed_geometry_persistence_rolls_runtime_back() -> None:
    coordinator, registry, derived, _, detector, _, _ = _coordinator(
        persist_result=0
    )
    original_pose = derived.pose
    with pytest.raises(RuntimeError, match="runtime state was rolled back"):
        asyncio.run(
            coordinator.update_geometry(
                object_id=derived.object_id,
                x=3.0,
                y=4.0,
                z=0.6,
                yaw=0.5,
                size_x=0.8,
                size_y=0.4,
                size_z=1.2,
                frame_id="map",
                expected_map_id="office",
                expected_generation=7,
                persist_to_snapshot=True,
            )
        )
    current = registry.get_object(derived.object_id)
    assert current.pose == original_pose
    assert "operator_geometry" not in current.attributes
    assert derived.object_id not in detector.geometry_overrides


def test_update_geometry_rejects_frame_mismatch() -> None:
    coordinator, _, derived, *_ = _coordinator()
    with pytest.raises(ValueError, match="frame mismatch"):
        asyncio.run(
            coordinator.update_geometry(
                object_id=derived.object_id,
                x=3.0,
                y=4.0,
                z=0.6,
                yaw=0.5,
                size_x=0.8,
                size_y=0.4,
                size_z=1.2,
                frame_id="odom",
                expected_map_id="office",
                expected_generation=7,
                persist_to_snapshot=False,
            )
        )


def test_delete_removes_detector_registry_and_snapshot_row() -> None:
    coordinator, registry, derived, robot, detector, graph, store = _coordinator()
    deleted_id, persisted, _, _ = asyncio.run(
        coordinator.delete_object(
            object_id=derived.object_id,
            expected_map_id="office",
            expected_generation=7,
            persist_to_snapshot=True,
        )
    )
    assert deleted_id == derived.object_id
    assert registry.get_object(derived.object_id) is None
    assert registry.get_object(robot.object_id) is robot
    assert detector.deleted == [derived.object_id]
    assert store.delete_object_calls == [(derived.object_id, "office__s4")]
    assert persisted is True
    assert graph.clear_count == 1


def test_delete_rejects_robot_self_object() -> None:
    coordinator, registry, _, robot, *_ = _coordinator()
    with pytest.raises(ValueError, match="robot self-object"):
        asyncio.run(
            coordinator.delete_object(
                object_id=robot.object_id,
                expected_map_id="office",
                expected_generation=7,
                persist_to_snapshot=False,
            )
        )
    assert registry.get_object(robot.object_id) is robot


def test_delete_runtime_failure_restores_snapshot_row() -> None:
    coordinator, registry, derived, _, detector, _, store = _coordinator()

    async def fail_delete(_object_id):
        raise RuntimeError("detector delete failed")

    detector.delete_object = fail_delete
    with pytest.raises(RuntimeError, match="saved snapshot was rolled back"):
        asyncio.run(
            coordinator.delete_object(
                object_id=derived.object_id,
                expected_map_id="office",
                expected_generation=7,
                persist_to_snapshot=True,
            )
        )
    assert registry.get_object(derived.object_id) is derived
    assert store.delete_object_calls == [(derived.object_id, "office__s4")]
    assert len(store.persist_calls) == 1
    assert store.persist_calls[0][0][0][0].object_id == derived.object_id


def test_flush_preserves_robot_and_clears_committed_snapshot() -> None:
    coordinator, registry, _, robot, detector, graph, store = _coordinator()
    deleted_count, persisted, _, _ = asyncio.run(
        coordinator.flush_objects(
            expected_map_id="office",
            expected_generation=7,
            persist_to_snapshot=True,
        )
    )
    assert deleted_count == 1
    assert list(registry.all_objects()) == [robot]
    assert detector.reset_count == 1
    assert graph.clear_count == 1
    assert store.delete_map_calls == ["office__s4"]
    assert persisted is True


def test_flush_runtime_failure_restores_snapshot_rows() -> None:
    coordinator, registry, derived, robot, detector, _, store = _coordinator()

    async def fail_reset():
        raise RuntimeError("detector reset failed")

    detector.reset_derived_state = fail_reset
    with pytest.raises(RuntimeError, match="saved snapshot was rolled back"):
        asyncio.run(
            coordinator.flush_objects(
                expected_map_id="office",
                expected_generation=7,
                persist_to_snapshot=True,
            )
        )
    assert registry.get_object(derived.object_id) is derived
    assert registry.get_object(robot.object_id) is robot
    assert store.delete_map_calls == ["office__s4"]
    assert len(store.persist_calls) == 1


def test_persist_requires_a_committed_snapshot() -> None:
    coordinator, _, derived, *_ = _coordinator()
    coordinator.map_meta = None
    with pytest.raises(RuntimeError, match="save the map"):
        asyncio.run(
            coordinator.delete_object(
                object_id=derived.object_id,
                expected_map_id="office",
                expected_generation=7,
                persist_to_snapshot=True,
            )
        )


def test_mcp_list_epoch_and_label_update_round_trip() -> None:
    coordinator, registry, derived, *_ = _coordinator()
    mcp_tools.attach_state(registry=registry)
    mcp_tools.attach_annotation_store(None)
    mcp_tools.attach_object_mutations(coordinator)

    listed = asyncio.run(
        mcp_tools.list_objects(mcp_tools.ListObjects_Request())
    )
    assert listed.map_id == "office"
    assert listed.generation == 7

    updated = asyncio.run(
        mcp_tools.update_object_label(
            mcp_tools.UpdateObjectLabel_Request(
                object_id=derived.object_id,
                label="desk chair",
                expected_map_id=listed.map_id,
                expected_generation=listed.generation,
                persist_to_snapshot=False,
            )
        )
    )
    assert updated.object.id == derived.object_id
    assert updated.object.label == "desk_chair"
    assert updated.persisted is False


async def _web_request(app, method: str, path: str, body=None):
    raw = json.dumps(body).encode() if body is not None else b""
    scope = {
        "type": "http",
        "http_version": "1.1",
        "method": method,
        "scheme": "http",
        "path": path,
        "raw_path": path.encode(),
        "root_path": "",
        "query_string": b"",
        "client": ("test", 0),
        "server": ("test", 80),
        "headers": [
            (b"content-type", b"application/json"),
            (b"content-length", str(len(raw)).encode()),
        ],
    }
    sent = []
    delivered = False

    async def receive():
        nonlocal delivered
        if delivered:
            return {"type": "http.disconnect"}
        delivered = True
        return {"type": "http.request", "body": raw, "more_body": False}

    async def send(message):
        sent.append(message)

    await app(scope, receive, send)
    status = next(
        message["status"]
        for message in sent
        if message["type"] == "http.response.start"
    )
    payload = b"".join(
        message.get("body", b"")
        for message in sent
        if message["type"] == "http.response.body"
    )
    return status, json.loads(payload)


def test_web_object_correction_returns_typed_http_errors_and_updates() -> None:
    from scene_service import web

    coordinator, registry, derived, *_ = _coordinator()
    app = web.make_app(
        registry=registry,
        object_mutations=coordinator,
        map_binding={
            "map_id": "office",
            "generation": 7,
            "mode": "localization",
        },
    )

    async def exercise():
        status, out = await _web_request(
            app,
            "POST",
            f"/api/objects/{derived.object_id}/label",
            {
                "label": "office chair",
                "expected_map_id": "office",
                "expected_generation": 6,
            },
        )
        assert status == 409
        assert out["ok"] is False

        status, out = await _web_request(
            app,
            "DELETE",
            "/api/objects/scene.object.unknown_999",
            {
                "expected_map_id": "office",
                "expected_generation": 7,
            },
        )
        assert status == 404
        assert out["ok"] is False

        status, out = await _web_request(
            app,
            "POST",
            f"/api/objects/{derived.object_id}/geometry",
            {
                "x": 1.5,
                "y": 2.5,
                "z": 0.5,
                "yaw": 0.2,
                "size_x": 0.5,
                "size_y": 0.4,
                "size_z": 1.0,
                "frame_id": "map",
                "expected_map_id": "office",
                "expected_generation": 7,
                "persist_to_snapshot": False,
            },
        )
        assert status == 200 and out["ok"] is True
        assert out["object"]["geometry_source"] == "operator_bbox"
        assert out["object"]["navigation_grade"] is False

        status, out = await _web_request(app, "GET", "/api/objects3d")
        assert status == 200
        operator = next(
            item for item in out["objects"] if item["id"] == derived.object_id
        )
        assert operator["points"] == []
        assert operator["geometry_source"] == "operator_bbox"

    asyncio.run(exercise())
