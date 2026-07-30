# SPDX-License-Identifier: MulanPSL-2.0
from scene_service.state.data_assoc import Detection, associate
from scene_service.state.object_registry import BBox3D, ObjectRegistry, Pose3D


def _detection(*, pose_frame: str, bbox_frame: str) -> Detection:
    return Detection(
        cls="fixture",
        pose=Pose3D(x=1.0, y=2.0, z=0.0, frame_id=pose_frame),
        bbox=BBox3D(
            size_x=0.2,
            size_y=0.2,
            size_z=0.2,
            frame_id=bbox_frame,
        ),
        confidence=0.9,
    )


def test_association_rejects_unknown_or_mixed_detection_frames():
    registry = ObjectRegistry()
    matched, new = associate(
        registry,
        [
            _detection(pose_frame="", bbox_frame="fixture_world"),
            _detection(pose_frame="fixture_world", bbox_frame="other_world"),
        ],
    )
    assert matched == []
    assert new == []
    assert list(registry.all_objects()) == []


def test_association_never_matches_objects_across_frames():
    registry = ObjectRegistry()
    existing = _detection(pose_frame="world_a", bbox_frame="world_a")
    _, first_new = associate(registry, [existing], now=1.0)
    assert len(first_new) == 1

    matched, second_new = associate(
        registry,
        [_detection(pose_frame="world_b", bbox_frame="world_b")],
        now=2.0,
    )
    assert matched == []
    assert len(second_new) == 1
    assert len(list(registry.all_objects())) == 2
