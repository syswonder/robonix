# SPDX-License-Identifier: MulanPSL-2.0
import numpy as np

from scene_service.ingest.perception_vlm import VLMObjectDetector, _CamIntrinsics


async def _ignore(_detections):
    """Accept projected detections without side effects."""


def _detector(transform, frame="fixture_world"):
    """Construct the projection-only portion of the VLM detector."""
    return VLMObjectDetector(
        rgb_fetcher=lambda: None,
        camera_to_world_fn=lambda: (
            (transform, frame) if transform is not None else None
        ),
        on_detections=_ignore,
        intrinsics=_CamIntrinsics(
            width=640,
            height=480,
            fx=320.0,
            fy=320.0,
            cx=320.0,
            cy=240.0,
        ),
    )


def test_vlm_projection_preserves_contract_frame():
    detector = _detector(np.eye(4))
    detections = detector._project_to_world(
        [{
            "cls": "cup",
            "confidence": 0.8,
            "bbox_2d": [300, 220, 340, 260],
            "approximate_depth_m": 2.0,
        }]
    )
    assert len(detections) == 1
    assert detections[0].pose.frame_id == "fixture_world"
    assert detections[0].bbox.frame_id == "fixture_world"
    assert detections[0].pose.z == 2.0


def test_vlm_projection_fails_closed_without_transform_or_depth():
    raw = [{
        "cls": "cup",
        "confidence": 0.8,
        "bbox_2d": [300, 220, 340, 260],
    }]
    assert _detector(None)._project_to_world(raw) == []
    assert _detector(np.eye(4))._project_to_world(raw) == []


def test_vlm_projection_recovers_when_intrinsics_arrive_after_startup():
    holder = {"intrinsics": None}
    detector = VLMObjectDetector(
        rgb_fetcher=lambda: None,
        camera_to_world_fn=lambda: (np.eye(4), "fixture_world"),
        on_detections=_ignore,
        intrinsics_fn=lambda: holder["intrinsics"],
    )
    raw = [{
        "cls": "cup",
        "confidence": 0.8,
        "bbox_2d": [300, 220, 340, 260],
        "approximate_depth_m": 2.0,
    }]
    assert detector._project_to_world(raw) == []
    holder["intrinsics"] = _CamIntrinsics(
        width=640,
        height=480,
        fx=320.0,
        fy=320.0,
        cx=320.0,
        cy=240.0,
    )
    assert len(detector._project_to_world(raw)) == 1
