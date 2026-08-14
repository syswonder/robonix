# SPDX-License-Identifier: MulanPSL-2.0
import time
from types import SimpleNamespace

import numpy as np

from scene_service.ingest.perception_concept_graphs import ConceptGraphsDetector
from scene_service.service import _camera_to_world_from_contracts
from scene_service.state import ObjectRegistry


def _vector(x=0.0, y=0.0, z=0.0):
    return SimpleNamespace(x=x, y=y, z=z)


def _quaternion():
    return SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)


def _pose(world_frame="fixture_world", child_frame="fixture_base"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=world_frame),
        child_frame_id=child_frame,
        pose=SimpleNamespace(
            pose=SimpleNamespace(
                position=_vector(1.0, 2.0, 0.0),
                orientation=_quaternion(),
            )
        ),
    )


def _extrinsics(parent="fixture_base", child="fixture_camera"):
    return SimpleNamespace(
        header=SimpleNamespace(frame_id=parent),
        child_frame_id=child,
        transform=SimpleNamespace(
            translation=_vector(0.1, 0.0, 0.5),
            rotation=_quaternion(),
        ),
    )


class _Hub:
    def __init__(self, *, pose=None, extrinsics=None, pose_age_s=0.0):
        now = time.time()
        self._slots = {
            "pose": (pose, now - pose_age_s if pose is not None else 0.0, 1),
            "camera_extrinsics": (
                extrinsics,
                now if extrinsics is not None else 0.0,
                1,
            ),
        }

    def has(self, kind):
        return kind in self._slots

    def latest(self, kind):
        return self._slots.get(kind, (None, 0.0, 0))


def _compose(hub):
    return _camera_to_world_from_contracts(
        hub,
        base_frame="fixture_base",
        camera_frame="fixture_camera",
        expected_world_frame="fixture_world",
        pose_max_age_s=2.0,
    )


def test_vlm_contract_composition_preserves_declared_world_frame():
    result = _compose(_Hub(pose=_pose(), extrinsics=_extrinsics()))
    assert result is not None
    matrix, world_frame = result
    assert world_frame == "fixture_world"
    np.testing.assert_allclose(matrix[:3, 3], [1.1, 2.0, 0.5])


def test_vlm_contract_composition_rejects_mismatched_or_stale_chain():
    assert _compose(_Hub(pose=_pose(), extrinsics=_extrinsics(parent="wrong"))) is None
    assert _compose(_Hub(pose=_pose("wrong"), extrinsics=_extrinsics())) is None
    assert _compose(_Hub(pose=_pose(), extrinsics=_extrinsics(), pose_age_s=3.0)) is None


def test_metric_detector_uses_the_same_frame_and_age_gates():
    async def _ignore(_detections):
        return None

    rgb = SimpleNamespace(header=SimpleNamespace(frame_id="fixture_camera"))
    detector = ConceptGraphsDetector(
        rgb_fetcher_msg=lambda: rgb,
        depth_fetcher_msg=lambda: None,
        camera_info_fetcher=lambda: None,
        on_detections=_ignore,
        registry=ObjectRegistry(),
        world_frame_fn=lambda: "fixture_world",
        robot_base_frame_fn=lambda: "fixture_base",
        hub=_Hub(pose=_pose(), extrinsics=_extrinsics()),
        pose_max_age_s=2.0,
    )
    matrix = detector._build_camera_to_map_transform()
    assert matrix is not None
    np.testing.assert_allclose(matrix[:3, 3], [1.1, 2.0, 0.5])

    detector._hub = _Hub(
        pose=_pose(),
        extrinsics=_extrinsics(child="wrong_camera"),
    )
    assert detector._build_camera_to_map_transform() is None
