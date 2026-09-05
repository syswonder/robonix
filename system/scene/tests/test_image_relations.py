# SPDX-License-Identifier: MulanPSL-2.0
"""Unit tests for the image-grounded VLM relation pass (VLM-primary).

Model-free: projection math, response parsing, frame annotation, and the
multimodal request body. The live VLM call is exercised only via a mocked
transport. Mirrors test_scene_graph.py's plain-`assert` + `__main__` style.
"""
import asyncio
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from scene_service.scene_graph.image_relations import (
    IMAGE_RELATION_VOCAB,
    ImageRelationInferer,
    ImageRelationResult,
    annotate_frame,
    build_image_relation_user_text,
    parse_image_relations,
    project_box,
    project_point,
)
from scene_service.scene_graph.types import SceneGraphNode


class _K:
    """Minimal intrinsics stub (duck-types _CamIntrinsics for projection)."""
    fx = fy = 500.0
    cx = 320.0
    cy = 240.0
    width = 640
    height = 480


def test_project_point():
    """world→pixel pinhole; identity transform ⇒ camera frame == map frame."""
    T = np.eye(4)
    K = _K()
    # Point straight ahead lands on the principal point.
    u, v, z = project_point(T, K, (0.0, 0.0, 2.0))
    assert abs(u - 320.0) < 1e-6 and abs(v - 240.0) < 1e-6 and abs(z - 2.0) < 1e-6
    # Offset in x shifts u by fx * x/z.
    u2, _, _ = project_point(T, K, (1.0, 0.0, 2.0))
    assert abs(u2 - (500.0 * 0.5 + 320.0)) < 1e-6
    # Behind the camera (z ≤ 0) is rejected.
    assert project_point(T, K, (0.0, 0.0, -1.0)) is None
    print("  [PASS] test_project_point")


def test_project_box():
    """A box in front projects to a clipped in-frame rect; behind/out → None."""
    T = np.eye(4)
    K = _K()
    rect = project_box(T, K, (0.0, 0.0, 2.0), (1.0, 1.0, 1.0), 0.0, 640, 480)
    assert rect is not None
    u0, v0, u1, v1 = rect
    assert 0 <= u0 < u1 <= 640 and 0 <= v0 < v1 <= 480
    # The box brackets the principal point (object centered ahead).
    assert u0 < 320 < u1 and v0 < 240 < v1
    # Behind the camera → None.
    assert project_box(T, K, (0.0, 0.0, -2.0), (1.0, 1.0, 1.0), 0.0, 640, 480) is None
    # Far off to the side → projects outside the frame → clipped to zero area.
    assert project_box(T, K, (100.0, 0.0, 2.0), (1.0, 1.0, 1.0), 0.0, 640, 480) is None
    print("  [PASS] test_project_box")


def test_parse_image_relations():
    """box#→object_id mapping + validation: drop unknown box, self, dup, vocab."""
    box_to_oid = {1: "scene.object.monitor_001", 2: "scene.object.table_001"}
    raw = {
        "edges": [
            {"source": 1, "target": 2, "relation": "on_top_of", "confidence": 0.9},
            {"source": 1, "target": 2, "relation": "on_top_of"},   # duplicate
            {"source": 1, "target": 1, "relation": "on_top_of"},   # self-edge
            {"source": 1, "target": 9, "relation": "on_top_of"},   # unknown box
            {"source": 2, "target": 1, "relation": "near"},        # out-of-vocab
            {"source": 2, "target": 1, "relation": "On Top Of"},   # normalized
        ]
    }
    edges = parse_image_relations(raw, box_to_oid)
    assert len(edges) == 2, f"expected 2 valid edges, got {len(edges)}"
    e0 = edges[0]
    assert e0.source_id == "scene.object.monitor_001"
    assert e0.target_id == "scene.object.table_001"
    assert e0.relation == "on_top_of" and e0.method == "llm"
    assert 0.0 <= e0.confidence <= 1.0
    # The normalized "On Top Of" survived as the 2→1 edge.
    assert edges[1].relation == "on_top_of"
    assert all(e.relation in IMAGE_RELATION_VOCAB for e in edges)
    # Empty / malformed input is safe.
    assert parse_image_relations({}, box_to_oid) == []
    assert parse_image_relations({"edges": [None, "x", {}]}, box_to_oid) == []
    print("  [PASS] test_parse_image_relations")


def test_build_user_text():
    """Legend text lists each box number and label + asks for JSON edges."""
    txt = build_image_relation_user_text([(1, "monitor"), (2, "wooden table")])
    assert "1: monitor" in txt and "2: wooden table" in txt
    assert "edges" in txt
    print("  [PASS] test_build_user_text")


def test_annotate_frame():
    """Annotated frame encodes to a non-empty base64 JPEG (needs cv2)."""
    try:
        import cv2  # noqa: F401
    except Exception as e:  # noqa: BLE001
        print(f"  [SKIP] test_annotate_frame (cv2 unavailable: {e})")
        return
    frame = np.zeros((480, 640, 3), dtype=np.uint8)
    b64 = annotate_frame(frame, [(1, (100, 100, 200, 200)), (2, (300, 200, 400, 300))])
    assert isinstance(b64, str) and len(b64) > 100
    import base64
    assert base64.b64decode(b64)[:2] == b"\xff\xd8", "JPEG SOI marker expected"
    print("  [PASS] test_annotate_frame")


def test_chat_json_image_body():
    """chat_json builds multimodal user content when images= is passed."""
    import httpx
    from scene_service.scene_graph.llm_client import SceneGraphLLMClient

    captured: dict = {}

    def handler(request):
        captured["body"] = json.loads(request.content)
        return httpx.Response(
            200, json={"choices": [{"message": {"content": '{"edges": []}'}}]}
        )

    orig = httpx.AsyncClient

    def patched(*a, **k):
        k.setdefault("transport", httpx.MockTransport(handler))
        return orig(*a, **k)

    run = asyncio.get_event_loop().run_until_complete
    httpx.AsyncClient = patched
    try:
        c = SceneGraphLLMClient(api_key="k", base_url="http://x", reasoning_effort="")
        # With an image: content is a list [text, image_url].
        run(c.chat_json("sys", "look", images=["QUJD"]))
        content = captured["body"]["messages"][1]["content"]
        assert isinstance(content, list)
        assert content[0]["type"] == "text" and content[0]["text"] == "look"
        assert content[1]["type"] == "image_url"
        assert content[1]["image_url"]["url"] == "data:image/jpeg;base64,QUJD"
        # Without images: content is the bare string (backward compatible).
        captured.clear()
        run(c.chat_json("sys", "plain"))
        assert captured["body"]["messages"][1]["content"] == "plain"
    finally:
        httpx.AsyncClient = orig
    print("  [PASS] test_chat_json_image_body")


def test_inferer_no_creds():
    """The inferer returns None (→ text fallback) when the LLM is unavailable."""
    from scene_service.scene_graph.llm_client import SceneGraphLLMClient

    client = SceneGraphLLMClient(api_key="", base_url="")
    inferer = ImageRelationInferer(client)
    nodes = [
        SceneGraphNode("scene.object.a_001", "a", (0.0, 0.0, 2.0), (0.2, 0.2, 0.2)),
        SceneGraphNode("scene.object.b_001", "b", (0.3, 0.0, 2.0), (0.2, 0.2, 0.2)),
    ]
    bundle = (np.zeros((480, 640, 3), dtype=np.uint8), _K(), np.eye(4))
    out = asyncio.get_event_loop().run_until_complete(inferer.infer(nodes, bundle))
    assert out is None, "unavailable LLM must yield None, not edges"
    print("  [PASS] test_inferer_no_creds")


_LLM_ENV = (
    "VLM_API_KEY", "VLM_BASE_URL", "VLM_MODEL", "VLM_REASONING_EFFORT",
    "OPENAI_API_KEY", "OPENAI_BASE_URL", "OPENAI_MODEL",
)


def test_builder_dispatch():
    """rebuild_once takes the image path when perception yields a bundle, and
    the text fallback otherwise."""
    import tempfile
    import time

    from scene_service.scene_graph.builder import SceneGraphBuilder, SceneGraphConfig
    from scene_service.scene_graph.captioner import NodeCaptioner
    from scene_service.scene_graph.llm_client import SceneGraphLLMClient
    from scene_service.scene_graph.relations import RelationInferer
    from scene_service.scene_graph.store import SceneGraphStore
    from scene_service.scene_graph.types import SceneGraphEdge
    from scene_service.state.object_registry import BBox3D, ObjectRegistry, Pose3D

    backup = {k: os.environ.pop(k, None) for k in _LLM_ENV}
    try:
        loop = asyncio.get_event_loop()
        with tempfile.TemporaryDirectory() as tmpdir:
            registry = ObjectRegistry()

            async def populate():
                async with registry.lock():
                    a = registry.insert_object(
                        "monitor", Pose3D(1.0, 0.0, 0.9), BBox3D(0.5, 0.2, 0.3),
                        confidence=0.9, now=time.time())
                    a.observation_count = 8
                    b = registry.insert_object(
                        "table", Pose3D(1.0, 0.0, 0.4), BBox3D(1.2, 0.8, 0.05),
                        confidence=0.9, now=time.time())
                    b.observation_count = 8

            loop.run_until_complete(populate())
            client = SceneGraphLLMClient(api_key="", base_url="")
            cfg = SceneGraphConfig()
            cfg.caption_enabled = False

            # Image path: a stub perception yields a bundle; a fake inferer
            # returns a known edge. That edge must land in the semantic slice.
            inj = [SceneGraphEdge(
                "scene.object.monitor_001", "scene.object.table_001",
                "on_top_of", 0.9, "llm")]

            class _Perc:
                def latest_frame_bundle(self):
                    return (np.zeros((480, 640, 3), dtype=np.uint8), _K(), np.eye(4))

            class _FakeInf:
                def __init__(self):
                    self.result = ImageRelationResult(tuple(inj), "processed")

                async def infer(self, nodes, bundle):
                    return self.result

            store = SceneGraphStore(cache_dir=tmpdir)
            builder = SceneGraphBuilder(
                registry=registry, captioner=NodeCaptioner(),
                relation_inferer=RelationInferer(client), store=store,
                config=cfg, perception=_Perc())
            fake_inferer = _FakeInf()
            builder.image_inferer = fake_inferer
            loop.run_until_complete(builder.rebuild_once())
            sem = store.get_semantic_edges()
            assert any(
                e.relation == "on_top_of"
                and e.source_id == "scene.object.monitor_001"
                for e in sem
            ), f"image-path edge expected in semantic slice, got {sem}"

            # Backoff made no observation, so repeated no-call rounds preserve
            # the established edge without consuming its stale-round budget.
            fake_inferer.result = ImageRelationResult((), "backoff")
            loop.run_until_complete(builder.rebuild_once())
            loop.run_until_complete(builder.rebuild_once())
            sem = store.get_semantic_edges()
            assert len(sem) == 1 and sem[0].stale_rounds == 0

            # A real attempted failure advances normal bounded hysteresis once.
            fake_inferer.result = ImageRelationResult((), "failed")
            loop.run_until_complete(builder.rebuild_once())
            sem = store.get_semantic_edges()
            assert len(sem) == 1 and sem[0].stale_rounds == 1

            # Text fallback: no perception → image pass returns None → text
            # path runs; with no LLM creds it yields no semantic edges.
            store2 = SceneGraphStore(cache_dir=tmpdir + "/2")
            builder2 = SceneGraphBuilder(
                registry=registry, captioner=NodeCaptioner(),
                relation_inferer=RelationInferer(client), store=store2,
                config=cfg, perception=None)
            loop.run_until_complete(builder2.rebuild_once())
            assert store2.get_semantic_edges() == [], (
                "text fallback with no creds should yield no semantic edges, "
                f"got {store2.get_semantic_edges()}")
    finally:
        for k, v in backup.items():
            if v is not None:
                os.environ[k] = v
    print("  [PASS] test_builder_dispatch")


def test_builder_transient_blip_preserves_graph():
    """A flicker where all objects momentarily go `missing` (<2 stable) must
    NOT wipe the semantic edges; an object that actually leaves the registry
    drops its edges."""
    import tempfile
    import time

    from scene_service.scene_graph.builder import SceneGraphBuilder, SceneGraphConfig
    from scene_service.scene_graph.captioner import NodeCaptioner
    from scene_service.scene_graph.llm_client import SceneGraphLLMClient
    from scene_service.scene_graph.relations import RelationInferer
    from scene_service.scene_graph.store import SceneGraphStore
    from scene_service.scene_graph.types import SceneGraphEdge
    from scene_service.state.object_registry import BBox3D, ObjectRegistry, Pose3D

    backup = {k: os.environ.pop(k, None) for k in _LLM_ENV}
    try:
        loop = asyncio.get_event_loop()
        with tempfile.TemporaryDirectory() as tmpdir:
            registry = ObjectRegistry()

            async def populate():
                async with registry.lock():
                    m = registry.insert_object(
                        "monitor", Pose3D(1.0, 0.0, 0.9), BBox3D(0.5, 0.2, 0.3),
                        confidence=0.9, now=time.time())
                    m.observation_count = 8
                    t = registry.insert_object(
                        "table", Pose3D(1.0, 0.0, 0.4), BBox3D(1.2, 0.8, 0.05),
                        confidence=0.9, now=time.time())
                    t.observation_count = 8

            loop.run_until_complete(populate())

            inj = [SceneGraphEdge(
                "scene.object.monitor_001", "scene.object.table_001",
                "on_top_of", 0.9, "llm")]

            class _Perc:
                def latest_frame_bundle(self):
                    return (np.zeros((480, 640, 3), dtype=np.uint8), _K(), np.eye(4))

            class _FakeInf:
                async def infer(self, nodes, bundle):
                    return ImageRelationResult(tuple(inj), "processed")

            cfg = SceneGraphConfig()
            cfg.caption_enabled = False
            store = SceneGraphStore(cache_dir=tmpdir)
            b = SceneGraphBuilder(
                registry=registry, captioner=NodeCaptioner(),
                relation_inferer=RelationInferer(
                    SceneGraphLLMClient(api_key="", base_url="")),
                store=store, config=cfg, perception=_Perc())
            b.image_inferer = _FakeInf()

            # Round 1: edge established with both objects stable.
            loop.run_until_complete(b.rebuild_once())
            assert len(store.get_semantic_edges()) == 1

            # Round 2: flicker — both objects `missing` → <2 stable nodes. The
            # transient-blip guard must preserve the edge, not wipe it.
            for o in registry._objects.values():
                o.missing = True
            loop.run_until_complete(b.rebuild_once())
            assert len(store.get_semantic_edges()) == 1, (
                "flicker (all missing) must preserve the edge, "
                f"got {store.get_semantic_edges()}")

            # Round 3: the table actually leaves the registry → its edge drops.
            del registry._objects["scene.object.table_001"]
            loop.run_until_complete(b.rebuild_once())
            assert store.get_semantic_edges() == [], (
                "an edge whose endpoint left the registry must be dropped")
    finally:
        for k, v in backup.items():
            if v is not None:
                os.environ[k] = v
    print("  [PASS] test_builder_transient_blip_preserves_graph")


if __name__ == "__main__":
    print("Running image-relation unit tests...\n")
    test_project_point()
    test_project_box()
    test_parse_image_relations()
    test_build_user_text()
    test_annotate_frame()
    test_chat_json_image_body()
    test_inferer_no_creds()
    test_builder_dispatch()
    test_builder_transient_blip_preserves_graph()
    print("\nAll tests passed!")
