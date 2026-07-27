# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for Scene label evidence and class-safe association."""

import asyncio

import pytest

from scene_service.ingest.perception_concept_graphs import (
    ConceptGraphsDetector,
    _label_evidence,
    _resolved_classes,
)
from scene_service.state.object_registry import BBox3D, ObjectRegistry, Pose3D


def _evidence(class_ids, confidences, *, current="chair"):
    return _label_evidence(
        {"class_id": class_ids, "conf": confidences, "class_name": current},
        ["chair", "table"],
        current_label=current,
        history_size=20,
        min_switch_observations=3,
        min_winner_share=0.65,
        switch_margin=0.20,
    )


def test_label_stays_provisional_until_repeated_evidence() -> None:
    first = _evidence([0], [0.90])
    assert first["label"] == "chair"
    assert first["provisional"] is True
    assert first["evidence_count"] == 1

    stable = _evidence([0, 0, 0], [0.80, 0.90, 0.85])
    assert stable["label"] == "chair"
    assert stable["provisional"] is False
    assert stable["confidence"] == pytest.approx(1.0)


def test_label_switch_requires_support_share_and_margin() -> None:
    weak_challenger = _evidence(
        [0, 0, 0, 1, 1],
        [0.90, 0.90, 0.90, 0.95, 0.95],
    )
    assert weak_challenger["label"] == "chair"

    strong_challenger = _evidence(
        [0, 0, 0, 1, 1, 1, 1, 1, 1, 1],
        [0.90, 0.90, 0.90] + [0.95] * 7,
    )
    assert strong_challenger["label"] == "table"
    assert strong_challenger["provisional"] is False
    assert strong_challenger["candidates"][0]["label"] == "table"


def test_configured_vocabulary_precedes_legacy_env(monkeypatch) -> None:
    monkeypatch.setenv("SCENE_OPEN_VOCAB_CLASSES", "legacy,override")
    assert _resolved_classes() == ["legacy", "override"]
    assert _resolved_classes(
        ["Chair", "chair", "Desk"],
        ["Mug", "desk"],
    ) == ["chair", "desk", "mug"]
    configured_addition = _resolved_classes(None, ["Mug"])
    assert "legacy" not in configured_addition
    assert "mug" in configured_addition


def _detector(**kwargs) -> ConceptGraphsDetector:
    async def _noop(_):
        return None

    kwargs.setdefault("allow_cross_class_merge", False)
    return ConceptGraphsDetector(
        rgb_fetcher_msg=lambda: None,
        depth_fetcher_msg=lambda: None,
        camera_info_fetcher=lambda: None,
        on_detections=_noop,
        registry=ObjectRegistry(),
        world_frame_fn=lambda: "map",
        classes=["chair", "table", "sofa", "couch"],
        **kwargs,
    )


def test_cross_class_association_is_opt_in() -> None:
    strict = _detector()
    assert strict._association_compatible("chair", "chair")
    assert not strict._association_compatible("chair", "table")

    grouped = _detector(confusable_class_groups=[["sofa", "couch"]])
    assert grouped._association_compatible("sofa", "couch")
    assert not grouped._association_compatible("chair", "table")

    legacy = _detector(allow_cross_class_merge=True)
    assert legacy._association_compatible("chair", "table")


def test_confusable_groups_must_reference_resolved_vocabulary() -> None:
    with pytest.raises(ValueError, match="outside the resolved vocabulary"):
        _detector(confusable_class_groups=[["sofa", "loveseat"]])


def test_snapshot_exposes_stable_label_evidence() -> None:
    detector = _detector()
    snapshot = {
        "uuid": "u1",
        "cls": "chair",
        "x": 1.0,
        "y": 2.0,
        "z": 0.5,
        "yaw": 0.0,
        "size_x": 0.5,
        "size_y": 0.5,
        "size_z": 1.0,
        "confidence": 0.9,
        "label_confidence": 0.8,
        "label_provisional": False,
        "label_evidence_count": 4,
        "label_candidates": [{"label": "chair", "share": 0.8}],
    }
    asyncio.run(
        detector._apply_snapshot(
            [snapshot],
            observed_uuids={"u1"},
            frame_seq=4,
        )
    )
    obj = next(iter(detector._registry.all_objects()))
    assert obj.cls == "chair"
    assert obj.attributes["label_confidence"] == pytest.approx(0.8)
    assert obj.attributes["label_provisional"] is False
    assert obj.attributes["label_evidence_count"] == 4
    assert obj.attributes["navigation_grade"] is True


def test_operator_label_override_survives_new_model_evidence() -> None:
    detector = _detector()
    detector._map_objects = [
        {
            "id": "u1",
            "class_name": "chair",
            "class_id": [0],
            "conf": [0.9],
        }
    ]
    detector._uuid_to_oid["u1"] = "scene.object.chair_001"
    asyncio.run(
        detector.update_object_label(
            "scene.object.chair_001",
            "table",
        )
    )
    detector._map_objects[0]["class_id"].extend([0, 0, 0, 0])
    detector._map_objects[0]["conf"].extend([0.99] * 4)
    detector._stabilize_map_labels()
    obj = detector._map_objects[0]
    assert obj["class_name"] == "table"
    assert obj["label_source"] == "operator"
    assert obj["label_provisional"] is False
    asyncio.run(
        detector.clear_object_label_override("scene.object.chair_001")
    )
    assert obj["class_name"] == "chair"
    assert obj["label_source"] == "model"


def test_operator_geometry_survives_new_detector_projection() -> None:
    detector = _detector()
    snapshot = {
        "uuid": "u1",
        "cls": "chair",
        "x": 1.0,
        "y": 2.0,
        "z": 0.5,
        "yaw": 0.0,
        "size_x": 0.5,
        "size_y": 0.5,
        "size_z": 1.0,
        "confidence": 0.9,
        "label_confidence": 0.9,
        "label_provisional": False,
        "label_evidence_count": 4,
        "label_candidates": [],
    }

    async def exercise():
        await detector._apply_snapshot(
            [snapshot],
            observed_uuids={"u1"},
            frame_seq=1,
        )
        obj = next(iter(detector._registry.all_objects()))
        await detector.update_object_geometry_override(obj.object_id)
        async with detector._registry.lock():
            detector._registry.update_object_geometry(
                obj.object_id,
                Pose3D(3.0, 4.0, 0.6, 0.5, "map"),
                BBox3D(0.8, 0.4, 1.2, 0.5, "map"),
            )
        changed = dict(snapshot, x=8.0, y=9.0, size_x=2.0)
        await detector._apply_snapshot(
            [changed],
            observed_uuids={"u1"},
            frame_seq=2,
        )
        return obj

    obj = asyncio.run(exercise())
    assert (obj.pose.x, obj.pose.y, obj.pose.z) == (3.0, 4.0, 0.6)
    assert obj.bbox.size_x == pytest.approx(0.8)
    assert obj.attributes["geometry_source"] == "operator_bbox"
    assert obj.attributes["navigation_grade"] is False
