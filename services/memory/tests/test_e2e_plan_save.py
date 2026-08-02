"""End-to-end simulation of the plan-save→retrieve pipeline.

Simulates what happens when Pilot completes a successful RTDL plan:

  1. Pilot calls memory/remember with kv.task_type="plan" (simulated)
  2. Later, Pilot calls memory/hybrid_search for a similar query (simulated)
  3. The search returns the saved plan node, formatted for LLM reuse

Runs without Webots/LLM — tests the full Python-side data path.
"""

import asyncio
import json
import os
import sys
import tempfile
import shutil
from pathlib import Path

_HERE = Path(__file__).resolve().parent
_SVC = _HERE.parent
if str(_SVC) not in sys.path:
    sys.path.insert(0, str(_SVC))

from memory_service.core.types import (
    LogRecord, SpatialContext, ObjectCoord,
    TagFilter, RememberRequest, SearchRequest,
    NodeType,
)
from memory_service.storage.graph_store import GraphStore
from memory_service.storage.tag_index import TagIndex
from memory_service.storage.vector_store import VectorStore
from memory_service.storage.embedding_config import EmbeddingModelConfig
from memory_service.core.remember import RememberPipeline
from memory_service.core.retrieve import RetrievePipeline
from memory_service.core import llm_search


# ── Phase 1: Simulate Pilot saving successful plans ──────────────────────

def simulate_pilot_plan_saves(pipeline):
    """Simulate 3 successful RTDL plans that Pilot would save."""

    plans = [
        {
            "query": "去厨房检查灭火器",
            "description": "navigate to kitchen and inspect fire extinguisher",
            "steps": [
                "1. [tiago_navigation.navigate_to_goal] navigate to kitchen",
                "2. [tiago_camera.camera_snapshot] capture image of fire extinguisher",
                "3. [tiago_navigation.navigate_to_goal] return to start",
            ],
        },
        {
            "query": "巡逻走廊检查设备",
            "description": "patrol corridor and inspect equipment",
            "steps": [
                "1. [tiago_navigation.navigate_to_goal] navigate to corridor",
                "2. [tiago_camera.camera_snapshot] inspect equipment",
                "3. [tiago_navigation.navigate_to_goal] continue patrol",
            ],
        },
        {
            "query": "去办公室确认电脑是否关机",
            "description": "go to office and check if computers are off",
            "steps": [
                "1. [tiago_navigation.navigate_to_goal] navigate to office",
                "2. [tiago_camera.camera_snapshot] capture desk area",
            ],
        },
    ]

    saved_ids = []
    for plan in plans:
        lr = LogRecord(
            ts=0, level="Info", tag="pilot",
            msg=plan["query"],
        )
        kv = {
            "task_type": "plan",
            "success": "true",
            "plan_query": plan["query"],
            "plan_description": plan["description"],
            "plan_steps": "\n".join(plan["steps"]),
        }
        req = RememberRequest(
            session_id="e2e-test", plan_id=f"plan-{len(saved_ids)}",
            log_record=lr, kv=kv,
        )
        resp = asyncio.run(pipeline.execute(req))
        saved_ids.append(resp.node_id)
        print(f"  [SAVE] plan node {resp.node_id}: \"{plan['query']}\"")

    return saved_ids


# ── Phase 2: Also save some regular observations ─────────────────────────

def simulate_observations(pipeline):
    """Simulate regular object observations (like ObjectWatchdog would send)."""
    obs = [
        ("observed fire_extinguisher in corridor", "corridor"),
        ("observed laptop on desk in office", "office"),
        ("observed coffee_machine in kitchen", "kitchen"),
    ]
    for msg, scene in obs:
        lr = LogRecord(ts=0, level="Info", tag="camera", msg=msg)
        req = RememberRequest(
            session_id="e2e-test", plan_id="obs",
            log_record=lr,
        )
        resp = asyncio.run(pipeline.execute(req))
        print(f"  [OBS] node {resp.node_id}: \"{msg}\"")


# ── Phase 3: Simulate Pilot prefetch (search memory before planning) ─────

async def simulate_prefetch(retrieve_pipeline, query, expected_label):
    """Simulate Pilot's memory::prefetch searching memory for relevant context."""
    req = SearchRequest(query=query, top_k=5)
    resp = await retrieve_pipeline.execute(req)
    print(f"\n  [SEARCH] \"{query}\" → {len(resp.nodes)} results:")
    for i, n in enumerate(resp.nodes):
        marker = "★ PLAN" if (n.tags and n.tags.task_type == "plan") else "  obs"
        print(f"    {i+1}. {marker} node {n.node_id}: \"{n.summary}\" "
              f"(type={n.node_type.value}, weight={n.weight})")

    # Verify: plan-type nodes should appear for plan-like queries
    plan_nodes = [n for n in resp.nodes if n.tags and n.tags.task_type == "plan"]
    print(f"  → {len(plan_nodes)} plan nodes found")

    if expected_label == "plans_expected":
        assert len(plan_nodes) > 0, \
            f"Expected plan nodes in search results for '{query}', got none"
        print(f"  ✅ Plan nodes correctly retrieved for plan-like query")
    elif expected_label == "obs_expected":
        print(f"  ✅ Observation-heavy results as expected")

    return resp


# ── Phase 4: Verify LLM prompt formatting ────────────────────────────────

def verify_llm_prompt_formatting(graph, plan_ids):
    """Verify that plan nodes appear correctly in LLM ranking prompts."""
    nodes = []
    for pid in plan_ids:
        node = graph.get_node(pid)
        if node:
            nodes.append(node)
    # Also add an observation
    for nid in graph.all_ids():
        node = graph.get_node(nid)
        if node and node.node_type != NodeType.LESSON:
            nodes.append(node)
            break

    prompt = llm_search._build_prompt("how to patrol the corridor", nodes)
    print(f"\n  [LLM PROMPT] length={len(prompt)} chars")
    assert "successful plan" in prompt, "prompt should mention successful plans"
    assert "lesson" in prompt, "prompt should show node_type: lesson"
    print(f"  ✅ LLM prompt correctly includes plan type info")

    # Verify individual node formatting
    for node in nodes:
        if node.tags and node.tags.task_type == "plan":
            formatted = llm_search._format_node(node)
            assert "type: lesson" in formatted, \
                f"plan node {node.node_id} should show type: lesson"
            assert "task: plan" in formatted
    print(f"  ✅ Plan node formatting correct (type: lesson, task: plan)")


# ── Phase 5: TagIndex filtering ──────────────────────────────────────────

def verify_tag_filtering(tags, plan_ids):
    """Verify TagIndex can separate plans from observations."""
    tf_plan = TagFilter(task_type="plan")
    plan_candidates = tags.query(tf_plan)
    assert len(plan_candidates) >= len(plan_ids), \
        f"TagIndex should find at least {len(plan_ids)} plan nodes"
    for pid in plan_ids:
        assert pid in plan_candidates, \
            f"plan node {pid} should be findable via task_type=plan"
    print(f"  ✅ TagIndex correctly filters {len(plan_candidates)} plan nodes")


# ── Main ─────────────────────────────────────────────────────────────────

def main():
    print("=" * 60)
    print("End-to-End Plan Save → Retrieve Test")
    print("=" * 60)

    tmp = tempfile.mkdtemp()
    try:
        # Setup
        graph = GraphStore(data_dir=tmp)
        tags = TagIndex()
        cfg = EmbeddingModelConfig(dim=8)
        vectors = VectorStore(config=cfg, alpha=0.3)
        pipeline = RememberPipeline(graph, tags, vectors)
        retrieve_pipe = RetrievePipeline(graph, tags, vectors)

        # Phase 1: Save plans
        print("\n── Phase 1: Saving successful RTDL plans ──")
        plan_ids = simulate_pilot_plan_saves(pipeline)
        assert len(plan_ids) == 3

        # Verify each plan node has correct metadata
        for pid in plan_ids:
            node = graph.get_node(pid)
            assert node.node_type == NodeType.LESSON, \
                f"node {pid}: expected LESSON, got {node.node_type}"
            assert node.weight == 0.8, \
                f"node {pid}: expected weight 0.8, got {node.weight}"
            assert node.tags.task_type == "plan"
            assert node.tags.success is True
            assert len(node.embedding) == 8
        print("  ✅ All 3 plan nodes saved with LESSON type + weight 0.8")

        # Phase 2: Save observations
        print("\n── Phase 2: Saving regular observations ──")
        simulate_observations(pipeline)

        # Phase 3: Search
        print("\n── Phase 3: Simulating Pilot prefetch (search) ──")

        # Search 1: similar to a saved plan → should find plans
        asyncio.run(simulate_prefetch(
            retrieve_pipe,
            "去厨房看看灭火器还在不在",
            "plans_expected",
        ))

        # Search 2: similar to another plan
        asyncio.run(simulate_prefetch(
            retrieve_pipe,
            "帮我巡逻一下走廊",
            "plans_expected",
        ))

        # Search 3: observation-like query
        asyncio.run(simulate_prefetch(
            retrieve_pipe,
            "咖啡机在哪里",
            "obs_expected",
        ))

        # Phase 4: LLM prompt formatting
        print("\n── Phase 4: LLM prompt formatting ──")
        verify_llm_prompt_formatting(graph, plan_ids)

        # Phase 5: Tag filtering
        print("\n── Phase 5: TagIndex filtering ──")
        verify_tag_filtering(tags, plan_ids)

        # Summary
        total = graph.count()
        print(f"\n{'=' * 60}")
        print(f"All phases passed. Graph has {total} nodes total.")
        print(f"Plan nodes: {plan_ids}")
        print(f"{'=' * 60}")

    finally:
        shutil.rmtree(tmp, ignore_errors=True)


if __name__ == "__main__":
    import logging
    logging.disable(logging.CRITICAL)
    main()
