# SPDX-License-Identifier: MulanPSL-2.0
"""Regression tests for Scene VLM duplicate-call suppression (issue #207)."""

from __future__ import annotations

import asyncio
import io
import logging
import os
import sys
from unittest.mock import patch

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from scene_service.ingest.perception_vlm import VLMObjectDetector, _CamIntrinsics
from scene_service.scene_graph.image_relations import ImageRelationInferer
from scene_service.scene_graph.types import SceneGraphNode


def _scene_rgb() -> np.ndarray:
    """Build a deterministic textured frame with one prominent object."""
    yy, xx = np.indices((120, 160))
    image = np.stack(
        ((xx * 2) % 256, (yy * 2) % 256, (xx + yy) % 256),
        axis=-1,
    ).astype(np.uint8)
    image[35:85, 55:110] = (220, 40, 30)
    return image


def _jpeg(image: np.ndarray, quality: int = 90) -> bytes:
    """Encode one RGB fixture at a controllable JPEG quality."""
    output = io.BytesIO()
    Image.fromarray(image).save(output, format="JPEG", quality=quality)
    return output.getvalue()


async def _ignore(_detections) -> None:
    pass


def _run(coro):
    """Run a coroutine without removing the legacy suite's default loop."""
    try:
        return asyncio.run(coro)
    finally:
        asyncio.set_event_loop(asyncio.new_event_loop())


def _detector(frame_holder, **kwargs) -> VLMObjectDetector:
    """Construct a detector whose current JPEG can change during a test."""
    return VLMObjectDetector(
        rgb_fetcher=lambda: frame_holder["jpeg"],
        camera_to_world_fn=lambda: None,
        on_detections=_ignore,
        frame_change_threshold=0.04,
        **kwargs,
    )


def test_detection_cache_ignores_identical_compressed_and_noisy_frames(caplog):
    """Equivalent frames spend one model call despite encoding differences."""
    caplog.set_level(logging.DEBUG, logger="scene_service.ingest.perception_vlm")
    image = _scene_rgb()
    holder = {"jpeg": _jpeg(image, quality=95)}
    detector = _detector(holder)
    calls = []

    async def fake_call(jpeg_b64):
        calls.append(jpeg_b64)
        return []

    detector._call_vlm = fake_call

    async def exercise():
        """Feed exact, recompressed, and low-noise variants in sequence."""
        await detector._tick()
        await detector._tick()  # byte-identical cached JPEG
        holder["jpeg"] = _jpeg(image, quality=65)
        await detector._tick()  # same image, different JPEG compression
        noisy = np.clip(
            image.astype(np.int16)
            + np.random.default_rng(7).integers(-2, 3, image.shape),
            0,
            255,
        ).astype(np.uint8)
        holder["jpeg"] = _jpeg(noisy, quality=80)
        await detector._tick()  # sensor/compression-level noise

    _run(exercise())
    assert len(calls) == 1
    assert detector.inference_counts == {
        "processed": 1,
        "skipped": 3,
        "retried": 0,
        "failed": 0,
    }
    assert "processed=1 skipped=3 retried=0 failed=0" in caplog.text


def test_detection_cache_invalidates_on_meaningful_change_and_resolution():
    """A local object change and a resolution change both invalidate cache."""
    image = _scene_rgb()
    holder = {"jpeg": _jpeg(image)}
    detector = _detector(holder)
    calls = []

    async def fake_call(jpeg_b64):
        calls.append(jpeg_b64)
        return []

    detector._call_vlm = fake_call

    async def exercise():
        """Process the original frame followed by a changed scene."""
        await detector._tick()
        changed = image.copy()
        changed[48:72, 68:92] = (10, 240, 20)
        holder["jpeg"] = _jpeg(changed)
        await detector._tick()
        holder["jpeg"] = _jpeg(np.repeat(np.repeat(changed, 2, axis=0), 2, axis=1))
        await detector._tick()

    _run(exercise())
    assert len(calls) == 3
    assert detector.inference_counts["processed"] == 3


def test_detection_cache_reprojects_and_refreshes_objects_without_model_call():
    """Only newly delivered cache hits refresh registry observations."""
    holder = {"sample": (_jpeg(_scene_rgb()), 1.0)}
    published = []

    async def capture(detections):
        published.append(detections)

    detector = VLMObjectDetector(
        rgb_fetcher=lambda: holder["sample"],
        camera_to_world_fn=lambda: (np.eye(4), "map"),
        on_detections=capture,
        intrinsics=_CamIntrinsics(
            width=160,
            height=120,
            fx=100.0,
            fy=100.0,
            cx=80.0,
            cy=60.0,
        ),
    )
    calls = []

    async def fake_call(jpeg_b64):
        calls.append(jpeg_b64)
        return [
            {
                "cls": "cup",
                "confidence": 0.9,
                "bbox_2d": [70, 50, 90, 70],
                "approximate_depth_m": 2.0,
            }
        ]

    detector._call_vlm = fake_call

    async def exercise():
        """Repeat a frozen source timestamp, then deliver the same pixels anew."""
        await detector._tick()
        await detector._tick()
        holder["sample"] = (holder["sample"][0], 2.0)
        await detector._tick()

    _run(exercise())
    assert len(calls) == 1
    assert len(published) == 2
    assert published[0][0].pose.frame_id == "map"
    assert published[1][0].pose.frame_id == "map"


def test_detection_failures_use_endpoint_backoff_with_bounded_growth():
    """Retries back off across changed frames and stop growing at the cap."""
    image = _scene_rgb()
    holder = {"jpeg": _jpeg(image)}
    now = {"value": 0.0}
    detector = _detector(
        holder,
        failure_backoff_base_s=5.0,
        failure_backoff_max_s=20.0,
        clock=lambda: now["value"],
    )
    calls = []

    async def failed_call(jpeg_b64):
        calls.append(jpeg_b64)
        if len(calls) == 1:
            now["value"] = 30.0  # model timeout duration must not consume backoff
        return None

    detector._call_vlm = failed_call

    async def exercise():
        """Advance through base, doubled, and capped retry intervals."""
        await detector._tick()  # failure returns at t=30; retry due at t=35
        await detector._tick()  # suppressed at t=30
        changed = image.copy()
        changed[:, :80] = (255, 255, 255)
        holder["jpeg"] = _jpeg(changed)
        now["value"] = 34.9
        await detector._tick()  # changed input remains endpoint-backoff limited
        now["value"] = 35.0
        await detector._tick()  # retry; next interval doubles to 10 seconds
        now["value"] = 44.9
        await detector._tick()
        now["value"] = 45.0
        await detector._tick()  # retry; next interval reaches the 20-second cap
        now["value"] = 64.9
        await detector._tick()
        now["value"] = 65.0
        await detector._tick()  # retry; interval remains capped at 20 seconds
        now["value"] = 84.9
        await detector._tick()
        now["value"] = 85.0
        await detector._tick()

    _run(exercise())
    assert len(calls) == 5
    assert detector.inference_counts["skipped"] == 5
    assert detector.inference_counts["retried"] == 4
    assert detector.inference_counts["failed"] == 5


def test_detection_response_requires_a_detections_list():
    """Malformed model JSON fails while an explicit empty list succeeds."""
    detector = _detector({"jpeg": _jpeg(_scene_rgb())})
    detector.base_url = "https://example.invalid"
    detector.api_key = "test"
    response = {"content": '{"other": []}'}

    class _Response:
        status_code = 200
        text = ""

        def json(self):
            return {"choices": [{"message": response}]}

    class _Client:
        async def __aenter__(self):
            return self

        async def __aexit__(self, *_args):
            return None

        async def post(self, *_args, **_kwargs):
            return _Response()

    async def exercise():
        """Read malformed and valid-empty responses through the real parser."""
        assert await detector._call_vlm("jpeg") is None
        response["content"] = '{"detections": []}'
        assert await detector._call_vlm("jpeg") == []

    with patch(
        "scene_service.ingest.perception_vlm.httpx.AsyncClient",
        return_value=_Client(),
    ):
        _run(exercise())


class _K:
    fx = fy = 500.0
    cx = 80.0
    cy = 60.0
    width = 160
    height = 120


class _RelationClient:
    available = True

    def __init__(self, response=None) -> None:
        """Keep a deterministic response and count whole-scene model calls."""
        self.calls = 0
        self.response = (
            response
            if response is not None
            else {
                "edges": [
                    {
                        "source": 1,
                        "target": 2,
                        "relation": "on_top_of",
                        "confidence": 0.9,
                    }
                ]
            }
        )

    async def chat_json(self, *_args, **_kwargs):
        self.calls += 1
        return self.response


def _nodes() -> list[SceneGraphNode]:
    """Return two visible objects with stable ids, geometry, and captions."""
    return [
        SceneGraphNode(
            "scene.object.cup_001",
            "cup",
            (-0.15, 0.0, 2.0),
            (0.2, 0.2, 0.2),
            caption="red cup",
        ),
        SceneGraphNode(
            "scene.object.table_001",
            "table",
            (0.2, 0.0, 2.1),
            (0.6, 0.4, 0.2),
            caption="wooden table",
        ),
    ]


def _third_node() -> SceneGraphNode:
    """Return another visible object for visible-set invalidation tests."""
    return SceneGraphNode(
        "scene.object.book_001",
        "book",
        (0.0, -0.2, 2.2),
        (0.25, 0.15, 0.08),
        caption="blue book",
    )


def test_relation_cache_reuses_input_and_invalidates_scene_changes(caplog):
    """Reuse stable relation input and invalidate each meaningful component."""
    caplog.set_level(logging.DEBUG, logger="scene_service.scene_graph.image_relations")
    client = _RelationClient()
    inferer = ImageRelationInferer(client, frame_change_threshold=0.04)
    nodes = _nodes()
    frame = _scene_rgb()[..., ::-1].copy()  # relation bundle is BGR
    bundle = (frame, _K(), np.eye(4))

    async def exercise():
        """Vary order, noise, geometry, image, camera, then prompt input."""
        first = await inferer.infer(nodes, bundle)
        second = await inferer.infer(list(reversed(nodes)), bundle)
        assert first and second and second[0].method == "cached"

        noisy = np.clip(frame.astype(np.int16) + 1, 0, 255).astype(np.uint8)
        third = await inferer.infer(nodes, (noisy, _K(), np.eye(4)))
        assert third and third[0].method == "cached"

        nodes[0].bbox_center = (-0.35, 0.0, 2.0)
        await inferer.infer(nodes, bundle)  # visible geometry changed
        assert client.calls == 2

        changed = frame.copy()
        changed[10:100, 20:140] = (20, 240, 10)
        await inferer.infer(nodes, (changed, _K(), np.eye(4)))
        assert client.calls == 3

        moved_camera = np.eye(4)
        moved_camera[2, 3] = 0.1
        await inferer.infer(nodes, (changed, _K(), moved_camera))
        assert client.calls == 4

        nodes[0].caption = "blue cup"
        await inferer.infer(nodes, (changed, _K(), moved_camera))
        assert client.calls == 5

        await inferer.infer(nodes + [_third_node()], (changed, _K(), moved_camera))
        assert client.calls == 6

    with patch(
        "scene_service.scene_graph.image_relations.annotate_frame",
        return_value="ZmFrZS1qcGVn",
    ):
        _run(exercise())

    assert client.calls == 6
    assert inferer.inference_counts["processed"] == 6
    assert inferer.inference_counts["skipped"] == 2
    assert "processed=6 skipped=2 retried=0 failed=0" in caplog.text


def test_relation_cache_reuses_an_explicit_empty_result():
    """A valid empty edge list is successful and cacheable."""
    client = _RelationClient(response={"edges": []})
    inferer = ImageRelationInferer(client)
    bundle = (_scene_rgb()[..., ::-1].copy(), _K(), np.eye(4))

    async def exercise():
        """Run one valid-empty inference and then reuse it."""
        assert await inferer.infer(_nodes(), bundle) == []
        assert await inferer.infer(_nodes(), bundle) == []

    with patch(
        "scene_service.scene_graph.image_relations.annotate_frame",
        return_value="ZmFrZS1qcGVn",
    ):
        _run(exercise())

    assert client.calls == 1
    assert inferer.inference_counts["processed"] == 1
    assert inferer.inference_counts["skipped"] == 1


def test_relation_failure_backoff_is_global_and_bounded():
    """Relation retries survive input changes and stop growing at the cap."""
    now = {"value": 0.0}
    client = _RelationClient(response={})
    inferer = ImageRelationInferer(
        client,
        failure_backoff_base_s=10.0,
        failure_backoff_max_s=25.0,
        clock=lambda: now["value"],
    )
    bundle = (_scene_rgb()[..., ::-1].copy(), _K(), np.eye(4))

    async def exercise():
        """Advance through base, doubled, and capped relation retry intervals."""
        assert await inferer.infer(_nodes(), bundle) == []
        now["value"] = 2.0
        assert await inferer.infer(_nodes(), bundle) == []
        changed = bundle[0].copy()
        changed[10:100, 20:140] = (20, 240, 10)
        changed_bundle = (changed, _K(), np.eye(4))
        now["value"] = 10.0
        assert await inferer.infer(_nodes(), changed_bundle) == []
        now["value"] = 29.9
        assert await inferer.infer(_nodes(), changed_bundle) == []
        now["value"] = 30.0
        assert await inferer.infer(_nodes(), changed_bundle) == []
        now["value"] = 54.9
        assert await inferer.infer(_nodes(), changed_bundle) == []
        now["value"] = 55.0
        assert await inferer.infer(_nodes(), changed_bundle) == []
        now["value"] = 79.9
        assert await inferer.infer(_nodes(), changed_bundle) == []
        now["value"] = 80.0
        assert await inferer.infer(_nodes(), changed_bundle) == []

    with patch(
        "scene_service.scene_graph.image_relations.annotate_frame",
        return_value="ZmFrZS1qcGVn",
    ):
        _run(exercise())

    assert client.calls == 5
    assert inferer.inference_counts["retried"] == 4
    assert inferer.inference_counts["skipped"] == 4


def test_relation_malformed_response_retries_then_caches_empty_result():
    """Missing edges is a failure, but an explicit empty list can recover."""
    now = {"value": 0.0}
    client = _RelationClient(response={"other": []})
    inferer = ImageRelationInferer(
        client,
        failure_backoff_base_s=5.0,
        clock=lambda: now["value"],
    )
    bundle = (_scene_rgb()[..., ::-1].copy(), _K(), np.eye(4))

    async def exercise():
        """Reject malformed JSON, recover at deadline, then hit the cache."""
        assert await inferer.infer(_nodes(), bundle) == []
        now["value"] = 5.0
        client.response = {"edges": []}
        assert await inferer.infer(_nodes(), bundle) == []
        assert await inferer.infer(_nodes(), bundle) == []

    with patch(
        "scene_service.scene_graph.image_relations.annotate_frame",
        return_value="ZmFrZS1qcGVn",
    ):
        _run(exercise())

    assert client.calls == 2
    assert inferer.inference_counts == {
        "processed": 1,
        "skipped": 1,
        "retried": 1,
        "failed": 1,
    }


def test_relation_failure_does_not_fan_out_to_text_fallback():
    """A failed whole-scene call must not trigger many per-pair calls."""
    from scene_service.scene_graph.builder import SceneGraphBuilder

    class _Perception:
        def latest_frame_bundle(self):
            return (_scene_rgb()[..., ::-1].copy(), _K(), np.eye(4))

    class _FailedInferer:
        async def infer(self, _nodes_arg, _bundle):
            return []

    builder = object.__new__(SceneGraphBuilder)
    builder.perception = _Perception()
    builder.image_inferer = _FailedInferer()

    result = _run(builder._maybe_image_edges(_nodes()))
    assert result == [], "failed image pass must suppress per-pair text calls"
