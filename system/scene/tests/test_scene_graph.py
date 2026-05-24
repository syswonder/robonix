# SPDX-License-Identifier: MulanPSL-2.0
"""Unit tests for scene_graph core logic.

Runs without ROS2, Docker, atlas, or LLM API — pure Python.
"""
import asyncio
import json
import os
import sys
import tempfile
import time

# Ensure scene_service is importable without rclpy / codegen.
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))


# Env vars that SceneGraphLLMClient picks up via `arg or os.environ.get(...)`.
# Tests that assert "LLM unavailable" must clear them — otherwise on a dev
# machine where these are set, the constructor reads through `api_key=""`
# and actually hits the live LLM, returning e.g. "none" instead of the
# expected "unknown" / "llm_fail" path.
_LLM_ENV_VARS = (
    "VLM_API_KEY",
    "VLM_BASE_URL",
    "VLM_MODEL",
    "OPENAI_API_KEY",
    "OPENAI_BASE_URL",
    "OPENAI_MODEL",
)


def _pop_llm_env() -> dict:
    """Pop all LLM-related env vars and return them for restoration."""
    return {k: os.environ.pop(k, None) for k in _LLM_ENV_VARS}


def _restore_llm_env(backup: dict) -> None:
    for k, v in backup.items():
        if v is not None:
            os.environ[k] = v


def test_types():
    """SceneGraphNode / Edge / Snapshot constructors."""
    from scene_service.scene_graph.types import (
        RELATION_TYPES,
        INVERSE_RELATIONS,
        GeometryHint,
        SceneGraphEdge,
        SceneGraphNode,
        SceneGraphSnapshot,
    )

    node = SceneGraphNode(
        object_id="scene.object.cup_001",
        label="cup",
        bbox_center=(1.0, 2.0, 0.8),
        bbox_extent=(0.1, 0.1, 0.15),
    )
    assert node.caption is None
    assert node.observation_count == 0

    edge = SceneGraphEdge(
        source_id="scene.object.cup_001",
        target_id="scene.object.table_001",
        relation="on_top_of",
        confidence=0.9,
    )
    assert edge.method == "llm"

    snap = SceneGraphSnapshot(
        nodes={"cup_001": node},
        edges=[edge],
    )
    assert len(snap.nodes) == 1
    assert len(snap.edges) == 1

    # Inverse relations are consistent.
    for k, v in INVERSE_RELATIONS.items():
        assert k in RELATION_TYPES, f"{k} not in RELATION_TYPES"
        assert v in RELATION_TYPES, f"{v} not in RELATION_TYPES"

    hint = GeometryHint(distance=0.5, xy_overlap=0.3,
                        vertical_order="a_above_b", containment="none")
    assert hint.distance == 0.5
    print("  [PASS] test_types")


def test_prompts():
    """Prompt builder produces valid JSON."""
    from scene_service.scene_graph.prompts import (
        RELATION_SYSTEM_PROMPT,
        build_relation_user_prompt,
    )
    from scene_service.scene_graph.types import GeometryHint, SceneGraphNode

    a = SceneGraphNode("obj_1", "cup", (1.0, 0.5, 0.8), (0.1, 0.1, 0.15))
    b = SceneGraphNode("obj_2", "table", (1.0, 0.5, 0.4), (1.2, 0.8, 0.05))
    hint = GeometryHint(0.4, 0.6, "a_above_b", "none")

    msg = build_relation_user_prompt(a, b, hint)
    payload = json.loads(msg)

    assert "object_a" in payload
    assert "object_b" in payload
    assert "geometry_hint" in payload
    assert payload["object_a"]["label"] == "cup"
    assert payload["geometry_hint"]["vertical_order"] == "a_above_b"
    assert "relation" in RELATION_SYSTEM_PROMPT
    print("  [PASS] test_prompts")


def test_edge_candidates():
    """Edge candidate generation filters by distance / overlap / containment."""
    from scene_service.scene_graph.relations import (
        generate_edge_candidates,
        compute_geometry_hint,
    )
    from scene_service.scene_graph.types import SceneGraphNode

    cup = SceneGraphNode("cup", "cup", (1.0, 0.5, 0.8), (0.1, 0.1, 0.15))
    table = SceneGraphNode("table", "table", (1.0, 0.5, 0.4), (1.2, 0.8, 0.05))
    far_chair = SceneGraphNode("chair", "chair", (10.0, 10.0, 0.5), (0.5, 0.5, 0.8))

    # cup-table should be a candidate (distance < 2m).
    # cup-chair and table-chair should NOT (distance > 2m, no overlap).
    candidates = generate_edge_candidates([cup, table, far_chair])

    pair_ids = {(a.object_id, b.object_id) for a, b, _ in candidates}
    assert ("cup", "table") in pair_ids, f"cup-table should be candidate, got {pair_ids}"
    assert ("cup", "chair") not in pair_ids, "cup-chair should NOT be candidate"
    assert ("table", "chair") not in pair_ids, "table-chair should NOT be candidate"

    # Verify geometry hint.
    hint = compute_geometry_hint(cup, table)
    assert hint.vertical_order == "a_above_b"
    assert hint.distance < 1.0

    # Test max_candidates truncation.
    nodes = [
        SceneGraphNode(f"n{i}", "obj", (i * 0.1, 0, 0), (0.1, 0.1, 0.1))
        for i in range(20)
    ]
    cands = generate_edge_candidates(nodes, max_candidates=5)
    assert len(cands) <= 5, f"Expected <=5 candidates, got {len(cands)}"
    print("  [PASS] test_edge_candidates")


def test_captioner():
    """V1 captioner sets caption = label."""
    from scene_service.scene_graph.captioner import NodeCaptioner
    from scene_service.scene_graph.types import SceneGraphNode

    cap = NodeCaptioner()
    node = SceneGraphNode("obj_1", "cup", (0, 0, 0), (0.1, 0.1, 0.1))
    assert node.caption is None

    result = asyncio.get_event_loop().run_until_complete(cap.caption_node(node))
    assert result.caption == "cup"
    assert result.caption_updated_at is not None
    print("  [PASS] test_captioner")


def test_store_cache():
    """Store caches captions and relations, persists to JSON."""
    from scene_service.scene_graph.store import SceneGraphStore
    from scene_service.scene_graph.types import (
        SceneGraphEdge,
        SceneGraphNode,
        SceneGraphSnapshot,
    )

    with tempfile.TemporaryDirectory() as tmpdir:
        store = SceneGraphStore(cache_dir=tmpdir)

        node = SceneGraphNode(
            "obj_1", "cup", (1.0, 0.5, 0.8), (0.1, 0.1, 0.15),
            caption="a white cup", observation_count=10,
        )

        # Caption cache.
        assert store.get_cached_caption(node) is None
        store.put_cached_caption(node)
        assert store.get_cached_caption(node) == "a white cup"

        # Relation cache.
        node2 = SceneGraphNode(
            "obj_2", "table", (1.0, 0.5, 0.4), (1.2, 0.8, 0.05),
            caption="table", observation_count=20,
        )
        edge = SceneGraphEdge("obj_1", "obj_2", "on_top_of", 0.9, "llm", "cup is above table")
        assert store.get_cached_relation(node, node2) is None
        store.put_cached_relation(node, node2, edge)
        cached = store.get_cached_relation(node, node2)
        assert cached is not None
        assert cached.relation == "on_top_of"

        # Snapshot.
        snap = SceneGraphSnapshot(nodes={"obj_1": node}, edges=[edge])
        store.save_snapshot(snap)
        assert store.get_snapshot() is snap

        # Flush + reload from disk.
        store.flush_caches()
        assert os.path.exists(os.path.join(tmpdir, "captions.json"))
        assert os.path.exists(os.path.join(tmpdir, "relations.json"))

        store2 = SceneGraphStore(cache_dir=tmpdir)
        assert store2.get_cached_caption(node) == "a white cup"
        cached2 = store2.get_cached_relation(node, node2)
        assert cached2 is not None
        assert cached2.relation == "on_top_of"
    print("  [PASS] test_store_cache")


def test_llm_client_no_key():
    """LLM client with no API key returns empty dict and doesn't crash."""
    env_backup = _pop_llm_env()

    try:
        from scene_service.scene_graph.llm_client import SceneGraphLLMClient
        client = SceneGraphLLMClient(api_key="", base_url="")
        assert not client.available

        result = asyncio.get_event_loop().run_until_complete(
            client.chat_json("system", "user")
        )
        assert result == {}
    finally:
        _restore_llm_env(env_backup)
    print("  [PASS] test_llm_client_no_key")


def test_relation_inferer_no_llm():
    """RelationInferer returns unknown when LLM is unavailable."""
    env_backup = _pop_llm_env()
    try:
        from scene_service.scene_graph.llm_client import SceneGraphLLMClient
        from scene_service.scene_graph.relations import RelationInferer
        from scene_service.scene_graph.types import GeometryHint, SceneGraphNode

        client = SceneGraphLLMClient(api_key="", base_url="")
        inferer = RelationInferer(client)

        a = SceneGraphNode("obj_1", "cup", (1.0, 0.5, 0.8), (0.1, 0.1, 0.15), caption="cup")
        b = SceneGraphNode("obj_2", "table", (1.0, 0.5, 0.4), (1.2, 0.8, 0.05), caption="table")
        hint = GeometryHint(0.4, 0.6, "a_above_b", "none")

        edge = asyncio.get_event_loop().run_until_complete(
            inferer.infer_relation(a, b, hint)
        )
        assert edge.relation == "unknown"
        assert edge.method == "llm_fail"
        assert edge.source_id == "obj_1"
        assert edge.target_id == "obj_2"
    finally:
        _restore_llm_env(env_backup)
    print("  [PASS] test_relation_inferer_no_llm")


def test_builder_rebuild_no_objects():
    """Builder with empty registry produces empty snapshot."""
    from scene_service.scene_graph.builder import SceneGraphBuilder, SceneGraphConfig
    from scene_service.scene_graph.captioner import NodeCaptioner
    from scene_service.scene_graph.llm_client import SceneGraphLLMClient
    from scene_service.scene_graph.relations import RelationInferer
    from scene_service.scene_graph.store import SceneGraphStore
    from scene_service.state.object_registry import ObjectRegistry

    with tempfile.TemporaryDirectory() as tmpdir:
        registry = ObjectRegistry()
        store = SceneGraphStore(cache_dir=tmpdir)
        client = SceneGraphLLMClient(api_key="", base_url="")
        captioner = NodeCaptioner()
        inferer = RelationInferer(client)
        config = SceneGraphConfig()

        builder = SceneGraphBuilder(
            registry=registry,
            captioner=captioner,
            relation_inferer=inferer,
            store=store,
            config=config,
        )

        snap = asyncio.get_event_loop().run_until_complete(builder.rebuild_once())
        assert len(snap.nodes) == 0
        assert len(snap.edges) == 0
        assert snap.updated_at > 0
    print("  [PASS] test_builder_rebuild_no_objects")


def test_builder_rebuild_with_objects():
    """Builder with stable objects produces nodes and attempts edges."""
    env_backup = _pop_llm_env()
    try:
        _run_builder_rebuild_with_objects_body()
    finally:
        _restore_llm_env(env_backup)
    print("  [PASS] test_builder_rebuild_with_objects")


def _run_builder_rebuild_with_objects_body():
    from scene_service.scene_graph.builder import SceneGraphBuilder, SceneGraphConfig
    from scene_service.scene_graph.captioner import NodeCaptioner
    from scene_service.scene_graph.llm_client import SceneGraphLLMClient
    from scene_service.scene_graph.relations import RelationInferer
    from scene_service.scene_graph.store import SceneGraphStore
    from scene_service.state.object_registry import BBox3D, ObjectRegistry, Pose3D

    with tempfile.TemporaryDirectory() as tmpdir:
        registry = ObjectRegistry()

        loop = asyncio.get_event_loop()

        async def populate():
            async with registry.lock():
                cup = registry.insert_object(
                    "cup",
                    Pose3D(1.0, 0.5, 0.8),
                    BBox3D(0.1, 0.1, 0.15),
                    confidence=0.9,
                    now=time.time(),
                )
                # Make it stable: bump observation_count.
                cup.observation_count = 5

                table = registry.insert_object(
                    "table",
                    Pose3D(1.0, 0.5, 0.4),
                    BBox3D(1.2, 0.8, 0.05),
                    confidence=0.95,
                    now=time.time(),
                )
                table.observation_count = 10

                # Far away chair — should not generate edge with cup/table.
                chair = registry.insert_object(
                    "chair",
                    Pose3D(10.0, 10.0, 0.5),
                    BBox3D(0.5, 0.5, 0.8),
                    confidence=0.8,
                    now=time.time(),
                )
                chair.observation_count = 3

        loop.run_until_complete(populate())

        store = SceneGraphStore(cache_dir=tmpdir)
        client = SceneGraphLLMClient(api_key="", base_url="")
        captioner = NodeCaptioner()
        inferer = RelationInferer(client)
        config = SceneGraphConfig()

        builder = SceneGraphBuilder(
            registry=registry,
            captioner=captioner,
            relation_inferer=inferer,
            store=store,
            config=config,
        )

        snap = loop.run_until_complete(builder.rebuild_once())

        # Should have 3 nodes (all have obs >= 2).
        assert len(snap.nodes) == 3, f"Expected 3 nodes, got {len(snap.nodes)}"

        # All nodes should have caption = label (v1 captioner).
        for node in snap.nodes.values():
            assert node.caption == node.label, f"{node.object_id}: caption should be label"

        # cup-table should be a candidate edge (nearby).
        # LLM is unavailable → relation=unknown → edge filtered out (not in edges list).
        # But the relation cache should have an entry.
        # Note: candidate order depends on sort (observation_count desc),
        # so the pair could be (table, cup) or (cup, table).
        cup_node = [n for n in snap.nodes.values() if n.label == "cup"][0]
        table_node = [n for n in snap.nodes.values() if n.label == "table"][0]
        cached = (
            store.get_cached_relation(cup_node, table_node)
            or store.get_cached_relation(table_node, cup_node)
        )
        assert cached is not None, "cup-table relation should be cached (either direction)"
        assert cached.relation == "unknown", "Without LLM, relation should be unknown"

        # Edges list should be empty (unknown relations are filtered out).
        assert len(snap.edges) == 0, f"Expected 0 visible edges (no LLM), got {len(snap.edges)}"

        # Verify snapshot is persisted.
        assert store.get_snapshot() is snap


def test_geometry_containment():
    """Containment detection works correctly."""
    from scene_service.scene_graph.relations import compute_geometry_hint
    from scene_service.scene_graph.types import SceneGraphNode

    small = SceneGraphNode("small", "ball", (1.0, 1.0, 0.5), (0.1, 0.1, 0.1))
    big = SceneGraphNode("big", "box", (1.0, 1.0, 0.5), (1.0, 1.0, 1.0))

    hint = compute_geometry_hint(small, big)
    assert hint.containment == "a_inside_b", f"Expected a_inside_b, got {hint.containment}"

    hint2 = compute_geometry_hint(big, small)
    assert hint2.containment == "b_inside_a", f"Expected b_inside_a, got {hint2.containment}"

    # Two separate objects.
    far = SceneGraphNode("far", "obj", (5.0, 5.0, 0.5), (0.1, 0.1, 0.1))
    hint3 = compute_geometry_hint(small, far)
    assert hint3.containment == "none"
    print("  [PASS] test_geometry_containment")


if __name__ == "__main__":
    print("Running scene_graph unit tests...\n")
    test_types()
    test_prompts()
    test_edge_candidates()
    test_captioner()
    test_store_cache()
    test_llm_client_no_key()
    test_relation_inferer_no_llm()
    test_builder_rebuild_no_objects()
    test_builder_rebuild_with_objects()
    test_geometry_containment()
    print("\nAll tests passed!")
