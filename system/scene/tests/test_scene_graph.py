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
    "VLM_REASONING_EFFORT",
    "OPENAI_API_KEY",
    "OPENAI_BASE_URL",
    "OPENAI_MODEL",
)


def _ensure_loop() -> asyncio.AbstractEventLoop:
    """Current event loop, creating (and installing) one when the thread has
    none. Bare `asyncio.get_event_loop()` is unreliable across the suite:
    running the milvus-lite tests (test_persistence) first leaves the main
    thread without a current loop, and the deprecated implicit-creation path
    then raises "There is no current event loop"."""
    try:
        return asyncio.get_event_loop()
    except RuntimeError:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        return loop


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

    result = _ensure_loop().run_until_complete(cap.caption_node(node))
    assert result.caption == "cup"
    assert result.caption_updated_at is not None
    print("  [PASS] test_captioner")


def test_store_cache():
    """Store caches captions and relations, persists to JSON."""
    from scene_service.scene_graph.store import SceneGraphStore
    from scene_service.scene_graph.types import (
        SceneGraphEdge,
        SceneGraphNode,
    )

    from scene_service.scene_graph.relations import compute_geometry_hint

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
        hint = compute_geometry_hint(node, node2)
        edge = SceneGraphEdge("obj_1", "obj_2", "on_top_of", 0.9, "llm", "cup is above table")
        assert store.get_cached_relation(node, node2, hint) is None
        store.put_cached_relation(node, node2, hint, edge)
        cached = store.get_cached_relation(node, node2, hint)
        assert cached is not None
        assert cached.relation == "on_top_of"

        # Live slices compose into a snapshot.
        store.set_geometric([node], [edge])
        snap = store.get_snapshot()
        assert snap is not None and "obj_1" in snap.nodes
        assert any(e.relation == "on_top_of" for e in snap.edges)

        # Flush + reload from disk.
        store.flush_caches()
        assert os.path.exists(os.path.join(tmpdir, "captions.json"))
        assert os.path.exists(os.path.join(tmpdir, "relations.json"))

        store2 = SceneGraphStore(cache_dir=tmpdir)
        assert store2.get_cached_caption(node) == "a white cup"
        cached2 = store2.get_cached_relation(node, node2, hint)
        assert cached2 is not None
        assert cached2.relation == "on_top_of"

        # A map-epoch reset clears both live slices and persisted derived
        # caches, so old object ids and edges cannot reappear.
        store2.set_geometric([node], [edge])
        store2.clear_derived_state()
        assert store2.get_snapshot() is None
        assert store2.get_cached_caption(node) is None
        assert store2.get_cached_relation(node, node2, hint) is None
        store3 = SceneGraphStore(cache_dir=tmpdir)
        assert store3.get_cached_caption(node) is None
        assert store3.get_cached_relation(node, node2, hint) is None
    print("  [PASS] test_store_cache")


def test_store_cache_map_id_partition():
    """Two stores with different map_ids on the same base dir keep isolated
    caches: each writes to its own <map_id> subdir and cannot read the other's
    cached captions."""
    from scene_service.scene_graph.store import SceneGraphStore
    from scene_service.scene_graph.types import SceneGraphNode

    node = SceneGraphNode(
        "obj_1", "cup", (1.0, 0.5, 0.8), (0.1, 0.1, 0.15),
        caption="a white cup", observation_count=10,
    )

    with tempfile.TemporaryDirectory() as base:
        kitchen = SceneGraphStore(cache_dir=base, map_id="kitchen")
        kitchen.put_cached_caption(node)
        kitchen.flush_caches()

        # Same base dir, different map_id → nested in its own subdir, blind to
        # kitchen's cache.
        office = SceneGraphStore(cache_dir=base, map_id="office")
        assert office.get_cached_caption(node) is None

        assert os.path.exists(os.path.join(base, "kitchen", "captions.json"))
        assert not os.path.exists(os.path.join(base, "office", "captions.json"))

        # A fresh store reopened on the same map_id sees its own cache back.
        kitchen2 = SceneGraphStore(cache_dir=base, map_id="kitchen")
        assert kitchen2.get_cached_caption(node) == "a white cup"
    print("  [PASS] test_store_cache_map_id_partition")


def test_llm_client_no_key():
    """LLM client with no API key returns empty dict and doesn't crash."""
    env_backup = _pop_llm_env()

    try:
        from scene_service.scene_graph.llm_client import SceneGraphLLMClient
        client = SceneGraphLLMClient(api_key="", base_url="")
        assert not client.available

        result = _ensure_loop().run_until_complete(
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

        edge = _ensure_loop().run_until_complete(
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

        snap = _ensure_loop().run_until_complete(builder.rebuild_once())
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

        loop = _ensure_loop()

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
        from scene_service.scene_graph.relations import compute_geometry_hint
        cup_node = [n for n in snap.nodes.values() if n.label == "cup"][0]
        table_node = [n for n in snap.nodes.values() if n.label == "table"][0]
        cached = (
            store.get_cached_relation(
                cup_node, table_node, compute_geometry_hint(cup_node, table_node))
            or store.get_cached_relation(
                table_node, cup_node, compute_geometry_hint(table_node, cup_node))
        )
        assert cached is not None, "cup-table relation should be cached (either direction)"
        assert cached.relation == "unknown", "Without LLM, relation should be unknown"

        # Edges list should be empty (unknown relations are filtered out).
        assert len(snap.edges) == 0, f"Expected 0 visible edges (no LLM), got {len(snap.edges)}"

        # The builder published its (empty) semantic slice. Nodes + the
        # geometric slice are owned by the fast loop (not run here), so the
        # store's composed snapshot has no nodes — get_snapshot() is None.
        assert store.get_semantic_edges() == snap.edges
        assert store.get_snapshot() is None


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


def test_geometric_relation():
    """Deterministic geometric predicates: contact, containment, near."""
    from scene_service.scene_graph.geometry import geometric_relation
    from scene_service.scene_graph.types import SceneGraphNode

    # cup resting on table: cup bottom (0.5 - 0.075 = 0.425) meets table
    # top (0.4 + 0.025 = 0.425), with XY overlap.
    cup = SceneGraphNode("cup", "cup", (1.0, 0.5, 0.5), (0.1, 0.1, 0.15))
    table = SceneGraphNode("table", "table", (1.0, 0.5, 0.4), (1.2, 0.8, 0.05))
    assert geometric_relation(cup, table) == ("on_top_of", 0.95)
    assert geometric_relation(table, cup) == ("under", 0.95)

    # ball whose center sits inside the box, but the box center is NOT
    # inside the ball (offset so containment is one-directional).
    ball = SceneGraphNode("ball", "ball", (1.4, 1.0, 0.5), (0.1, 0.1, 0.1))
    box = SceneGraphNode("box", "box", (1.0, 1.0, 0.5), (1.0, 1.0, 1.0))
    assert geometric_relation(ball, box) == ("inside", 0.95)
    assert geometric_relation(box, ball) == ("contains", 0.95)

    # two cups 0.5 m apart on the floor — only `near`.
    c1 = SceneGraphNode("c1", "cup", (0.0, 0.0, 0.5), (0.1, 0.1, 0.1))
    c2 = SceneGraphNode("c2", "cup", (0.5, 0.0, 0.5), (0.1, 0.1, 0.1))
    rel = geometric_relation(c1, c2)
    assert rel is not None and rel[0] == "near"

    # far apart — geometry decides nothing.
    far = SceneGraphNode("far", "cup", (3.0, 0.0, 0.5), (0.1, 0.1, 0.1))
    assert geometric_relation(c1, far) is None
    print("  [PASS] test_geometric_relation")


def test_compute_geometric_edges():
    """compute_geometric_edges emits contact/containment edges only."""
    from scene_service.scene_graph.geometry import compute_geometric_edges
    from scene_service.scene_graph.types import SceneGraphNode

    cup = SceneGraphNode("cup", "cup", (1.0, 0.5, 0.5), (0.1, 0.1, 0.15))
    table = SceneGraphNode("table", "table", (1.0, 0.5, 0.4), (1.2, 0.8, 0.05))
    ball = SceneGraphNode("ball", "ball", (1.4, 1.0, 0.5), (0.1, 0.1, 0.1))
    box = SceneGraphNode("box", "box", (1.0, 1.0, 0.5), (1.0, 1.0, 1.0))
    far = SceneGraphNode("far", "cup", (5.0, 5.0, 0.5), (0.1, 0.1, 0.1))

    edges = compute_geometric_edges([cup, table, ball, box, far])
    rels = {(e.source_id, e.target_id, e.relation) for e in edges}

    assert ("cup", "table", "on_top_of") in rels
    assert any(e.relation in ("inside", "contains")
               for e in edges if {e.source_id, e.target_id} == {"ball", "box"})
    # `near` is never emitted as an edge.
    assert all(e.relation != "near" for e in edges)
    # every geometric edge is tagged and explained.
    assert all(e.method == "geometric" and e.reason for e in edges)
    # the isolated object touches nothing → no edge references it.
    assert all("far" not in (e.source_id, e.target_id) for e in edges)
    print("  [PASS] test_compute_geometric_edges")


def test_geometry_signature_stable():
    """Signature is stable under EMA-level jitter but flips on real moves."""
    from scene_service.scene_graph.geometry import geometry_signature
    from scene_service.scene_graph.types import GeometryHint

    base = GeometryHint(distance=0.5, xy_overlap=0.6,
                        vertical_order="a_above_b", containment="none")
    sig = geometry_signature(base)

    # ±2 cm / ±0.02 jitter around bucket centers — must not flip.
    for dd, do in ((0.02, 0.02), (-0.02, -0.02), (0.02, -0.02)):
        jittered = GeometryHint(distance=0.5 + dd, xy_overlap=0.6 + do,
                                vertical_order="a_above_b", containment="none")
        assert geometry_signature(jittered) == sig, (
            f"jitter flipped signature: {geometry_signature(jittered)} != {sig}")

    # Object lifted off / moved: vertical order changes → signature flips.
    moved = GeometryHint(distance=0.5, xy_overlap=0.6,
                         vertical_order="same_level", containment="none")
    assert geometry_signature(moved) != sig
    print("  [PASS] test_geometry_signature_stable")


def test_relation_cache_invalidation():
    """A cached edge invalidates when the geometry signature changes."""
    from scene_service.scene_graph.store import SceneGraphStore
    from scene_service.scene_graph.types import (
        GeometryHint, SceneGraphEdge, SceneGraphNode,
    )

    with tempfile.TemporaryDirectory() as tmpdir:
        store = SceneGraphStore(cache_dir=tmpdir)
        a = SceneGraphNode("obj_1", "cup", (1.0, 0.5, 0.5), (0.1, 0.1, 0.15))
        b = SceneGraphNode("obj_2", "table", (1.0, 0.5, 0.4), (1.2, 0.8, 0.05))
        hint = GeometryHint(0.1, 0.6, "a_above_b", "none")
        edge = SceneGraphEdge("obj_1", "obj_2", "on_top_of", 0.95, "geometric")
        store.put_cached_relation(a, b, hint, edge)

        # Same geometry → hit.
        assert store.get_cached_relation(a, b, hint) is not None

        # Object moved off: vertical order + distance changed → miss.
        moved = GeometryHint(0.8, 0.0, "same_level", "none")
        assert store.get_cached_relation(a, b, moved) is None
    print("  [PASS] test_relation_cache_invalidation")


def test_store_compose():
    """get_snapshot composes geometric + semantic slices, dedups, and is
    None before the geometric loop has populated anything."""
    from scene_service.scene_graph.store import SceneGraphStore
    from scene_service.scene_graph.types import SceneGraphEdge, SceneGraphNode

    with tempfile.TemporaryDirectory() as tmpdir:
        store = SceneGraphStore(cache_dir=tmpdir)
        assert store.get_snapshot() is None

        a = SceneGraphNode("a", "cup", (0.0, 0.0, 0.5), (0.1, 0.1, 0.1))
        b = SceneGraphNode("b", "table", (0.0, 0.0, 0.4), (1.0, 1.0, 0.05))
        store.set_geometric([a, b], [SceneGraphEdge("a", "b", "on_top_of", 0.95, "geometric")])
        store.set_semantic_edges([SceneGraphEdge("a", "b", "attached_to", 0.8, "llm")])

        snap = store.get_snapshot()
        assert snap is not None and set(snap.nodes) == {"a", "b"}
        # Distinct relations on the same pair coexist (different axes).
        assert {e.relation for e in snap.edges} == {"on_top_of", "attached_to"}

        # A semantic edge duplicating a geometric relation is deduped;
        # the geometric one wins.
        store.set_semantic_edges([SceneGraphEdge("a", "b", "on_top_of", 0.5, "llm")])
        ontop = [e for e in store.get_snapshot().edges if e.relation == "on_top_of"]
        assert len(ontop) == 1 and ontop[0].method == "geometric"
    print("  [PASS] test_store_compose")


def test_geometric_loop_debounce():
    """Debounce band: 2 present ticks to emit, 1 grace tick on dropout."""
    from scene_service.scene_graph.geometric_loop import GeometricRelationLoop
    from scene_service.scene_graph.types import SceneGraphEdge

    loop = GeometricRelationLoop(registry=None, store=None)
    e = SceneGraphEdge("a", "b", "on_top_of", 0.95, "geometric")

    assert loop._debounce([e]) == []                       # score 1 → hold
    assert [x.relation for x in loop._debounce([e])] == ["on_top_of"]  # score 2 → emit
    assert len(loop._debounce([e])) == 1                   # score 3 (cap) → emit
    assert len(loop._debounce([])) == 1                    # score 2 → grace emit
    assert loop._debounce([]) == []                        # score 1 → hold, no emit
    assert loop._debounce([]) == []                        # score 0 → dropped
    print("  [PASS] test_geometric_loop_debounce")


def test_infer_semantic_relation():
    """Semantic-only inference keeps semantic answers, collapses spatial to
    none, and surfaces transport failure as a retryable llm_fail."""
    import asyncio

    from scene_service.scene_graph.relations import RelationInferer
    from scene_service.scene_graph.types import GeometryHint, SceneGraphNode

    class _FakeClient:
        def __init__(self, reply):
            self._reply = reply

        async def chat_json(self, **_kw):
            return self._reply

    a = SceneGraphNode("a", "handle", (0.0, 0.0, 0.5), (0.1, 0.1, 0.1))
    b = SceneGraphNode("b", "door", (0.0, 0.0, 0.5), (1.0, 0.1, 2.0))
    hint = GeometryHint(0.0, 0.5, "same_level", "a_inside_b")
    run = _ensure_loop().run_until_complete

    # A spatial answer is rejected — geometry owns the spatial slot → none.
    inf = RelationInferer(_FakeClient({"relation": "on_top_of", "confidence": 0.9}))
    assert run(inf.infer_semantic_relation(a, b, hint, "contains")).relation == "none"

    # A genuine semantic answer is kept.
    inf2 = RelationInferer(_FakeClient(
        {"relation": "part_of", "confidence": 0.8, "reason": "handle of door"}))
    edge2 = run(inf2.infer_semantic_relation(a, b, hint, "contains"))
    assert edge2.relation == "part_of" and edge2.method == "llm"

    # Transport failure → retryable llm_fail, not a false "none".
    inf3 = RelationInferer(_FakeClient(None))
    edge3 = run(inf3.infer_semantic_relation(a, b, hint, "contains"))
    assert edge3.relation == "unknown" and edge3.method == "llm_fail"
    print("  [PASS] test_infer_semantic_relation")


def test_normalize_relation():
    """LLM relation strings canonicalize to the on-wire vocabulary tokens."""
    from scene_service.scene_graph.relations import _normalize_relation

    assert _normalize_relation("on top of") == "on_top_of"
    assert _normalize_relation("On-Top-Of") == "on_top_of"
    assert _normalize_relation("  ATTACHED TO ") == "attached_to"
    assert _normalize_relation("inside") == "inside"
    # Off-vocabulary stays off-vocabulary (the caller's membership check, not
    # this helper, decides validity) — no fuzzy coercion to a valid token.
    assert _normalize_relation("floating above") == "floating_above"
    print("  [PASS] test_normalize_relation")


def test_llm_client_reasoning_effort_default():
    """reasoning_effort is opt-in: unset/blank env → "" (field omitted, so
    strict endpoints that 400 on unknown params are unaffected); a set env
    value is used; a constructor arg overrides the env."""
    env_backup = _pop_llm_env()
    try:
        from scene_service.scene_graph.llm_client import SceneGraphLLMClient

        # env unset → "" (field omitted from requests)
        assert SceneGraphLLMClient().reasoning_effort == ""
        # env empty / whitespace → still "" (opt-in stays off)
        os.environ["VLM_REASONING_EFFORT"] = ""
        assert SceneGraphLLMClient().reasoning_effort == ""
        os.environ["VLM_REASONING_EFFORT"] = "   "
        assert SceneGraphLLMClient().reasoning_effort == ""
        # explicit env value wins
        os.environ["VLM_REASONING_EFFORT"] = "high"
        assert SceneGraphLLMClient().reasoning_effort == "high"
        # constructor arg overrides the env
        assert SceneGraphLLMClient(reasoning_effort="minimal").reasoning_effort == "minimal"
        assert SceneGraphLLMClient(reasoning_effort="").reasoning_effort == ""
    finally:
        _restore_llm_env(env_backup)
    print("  [PASS] test_llm_client_reasoning_effort_default")


def test_llm_client_request_body():
    """The request body carries reasoning_effort when set and omits it when
    disabled; max_tokens is never sent (no hidden cap)."""
    import httpx

    from scene_service.scene_graph.llm_client import SceneGraphLLMClient

    captured: dict = {}

    def handler(request: "httpx.Request") -> "httpx.Response":
        captured["body"] = json.loads(request.content)
        return httpx.Response(
            200, json={"choices": [{"message": {"content": '{"relation": "none"}'}}]}
        )

    # The client builds its own AsyncClient internally; wrap the constructor so
    # ours injects a MockTransport without changing production code.
    orig = httpx.AsyncClient

    def patched(*a, **k):
        k.setdefault("transport", httpx.MockTransport(handler))
        return orig(*a, **k)

    run = _ensure_loop().run_until_complete
    httpx.AsyncClient = patched
    try:
        c = SceneGraphLLMClient(
            api_key="k", base_url="http://x", reasoning_effort="minimal"
        )
        run(c.chat_json("sys", "user"))
        assert captured["body"]["reasoning_effort"] == "minimal"
        assert "max_tokens" not in captured["body"]

        captured.clear()
        c2 = SceneGraphLLMClient(api_key="k", base_url="http://x", reasoning_effort="")
        run(c2.chat_json("sys", "user"))
        assert "reasoning_effort" not in captured["body"]
    finally:
        httpx.AsyncClient = orig
    print("  [PASS] test_llm_client_request_body")


def test_prune_relations():
    """prune_relations drops edges whose endpoints left the registry."""
    from scene_service.scene_graph.store import SceneGraphStore
    from scene_service.scene_graph.types import (
        GeometryHint, SceneGraphEdge, SceneGraphNode,
    )

    with tempfile.TemporaryDirectory() as tmpdir:
        store = SceneGraphStore(cache_dir=tmpdir)
        hint = GeometryHint(0.5, 0.0, "same_level", "none")
        a = SceneGraphNode("a", "cup", (0.0, 0.0, 0.5), (0.1, 0.1, 0.1))
        b = SceneGraphNode("b", "cup", (0.5, 0.0, 0.5), (0.1, 0.1, 0.1))
        c = SceneGraphNode("c", "cup", (1.0, 0.0, 0.5), (0.1, 0.1, 0.1))
        store.put_cached_relation(a, b, hint, SceneGraphEdge("a", "b", "near", 0.7))
        store.put_cached_relation(b, c, hint, SceneGraphEdge("b", "c", "near", 0.7))

        # `c` gone: the b-c edge must be evicted, a-b kept.
        evicted = store.prune_relations({"a", "b"})
        assert evicted == 1, f"expected 1 eviction, got {evicted}"
        assert store.get_cached_relation(a, b, hint) is not None
        assert store.get_cached_relation(b, c, hint) is None
    print("  [PASS] test_prune_relations")


if __name__ == "__main__":
    print("Running scene_graph unit tests...\n")
    test_types()
    test_prompts()
    test_edge_candidates()
    test_captioner()
    test_store_cache()
    test_store_cache_map_id_partition()
    test_llm_client_no_key()
    test_relation_inferer_no_llm()
    test_builder_rebuild_no_objects()
    test_builder_rebuild_with_objects()
    test_geometry_containment()
    test_geometric_relation()
    test_compute_geometric_edges()
    test_geometry_signature_stable()
    test_relation_cache_invalidation()
    test_store_compose()
    test_geometric_loop_debounce()
    test_infer_semantic_relation()
    test_normalize_relation()
    test_llm_client_reasoning_effort_default()
    test_llm_client_request_body()
    test_prune_relations()
    print("\nAll tests passed!")
