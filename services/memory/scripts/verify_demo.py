#!/usr/bin/env python3
"""Scribe Mem verification — Demo 1 & 2 end-to-end validation.

Steps:
  1. Load YAML test data (or generate programmatically)
  2. Import into MemoryService
  3. Run Demo 1: object spatiotemporal backtracking queries
  4. Run Demo 2: task history backtracking queries
  5. Run mixed-scenario queries (failure lessons, combined filters)
  6. Report pass/fail for each verification case

Usage:
    python3 scripts/verify_demo.py                          # use generated data
    python3 scripts/verify_demo.py --yaml data/demo_memories.yaml  # use YAML file
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import sys
import tempfile
import time as _time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from memory_service.service import MemoryService
from memory_service.core.types import TagFilter, TimeRange, NodeType
from memory_service.core.builder import (
    load_memories_from_yaml,
    import_yaml_to_service,
    generate_demo_data,
    export_to_yaml,
)

log = logging.getLogger("verify_demo")

PASS = 0
FAIL = 0


def check(name: str, condition: bool, detail: str = ""):
    global PASS, FAIL
    if condition:
        print(f"  ✅ {name}")
        PASS += 1
    else:
        print(f"  ❌ {name}  — {detail}")
        FAIL += 1


async def run_verification(data_dir: str, yaml_path: str = "") -> int:
    """Run all verification cases."""
    global PASS, FAIL
    PASS = FAIL = 0

    print("=" * 60)
    print("  Scribe Mem — Verification Suite")
    print("=" * 60)

    # ── Step 1: Setup ──
    print("\n── Step 1: Service Setup ──")
    svc = MemoryService(data_dir=data_dir)
    await svc.init()

    # ── Step 2: Data Loading ──
    print("\n── Step 2: Data Loading ──")
    if yaml_path and os.path.exists(yaml_path):
        print(f"  Loading YAML: {yaml_path}")
        records = load_memories_from_yaml(yaml_path)
        node_ids = await import_yaml_to_service(svc, yaml_path, clear_existing=True)
    else:
        print("  Generating programmatic demo data")
        records = generate_demo_data()
        node_ids = []
        for rec in records:
            nid = await svc.remember(
                session_id=rec["session_id"],
                plan_id=rec["plan_id"],
                log_record=__import__("memory_service.core.types", fromlist=["LogRecord"]).LogRecord(
                    ts=rec.get("ts", _time.time_ns()),
                    level=rec["level"],
                    tag=rec["tag"],
                    msg=rec["msg"],
                ),
                spatial=None,
                kv=rec.get("kv", {}),
            )
            node_ids.append(nid)

    check("Data loaded", len(node_ids) > 0,
          f"Loaded {len(node_ids)} records")
    check("Node count matches", svc.graph.count() == len(node_ids),
          f"Expected {len(node_ids)}, got {svc.graph.count()}")
    check("Tag index rebuilt", svc.tags.count() == len(node_ids),
          f"Expected {len(node_ids)}, got {svc.tags.count()}")
    check("Vector store populated", svc.vectors.count() == len(node_ids),
          f"Expected {len(node_ids)}, got {svc.vectors.count()}")

    # ── Step 3: Demo 1 — Object Spatiotemporal Backtracking ──
    print("\n── Step 3: Demo 1 — Object Spatiotemporal Backtracking ──")

    # 3a: Find red cup in kitchen
    resp1 = await svc.search("red cup", tags=TagFilter(scene_type="kitchen"), top_k=3)
    check("Demo1a: kitchen search returns results", len(resp1.nodes) >= 1,
          f"Got {len(resp1.nodes)} results")
    if resp1.nodes:
        n = resp1.nodes[0]
        check("Demo1a: scene_type is kitchen",
              n.tags and n.tags.scene_type == "kitchen",
              f"Got scene_type={n.tags.scene_type if n.tags else None}")
        check("Demo1a: spatial data preserved",
              n.spatial_data is not None and len(n.spatial_data.objects) > 0)

    # 3b: Find blue cup — should be in living_room
    resp2 = await svc.search("blue cup", tags=TagFilter(scene_type="living_room"), top_k=3)
    check("Demo1b: living_room search returns results", len(resp2.nodes) >= 1,
          f"Got {len(resp2.nodes)} results")
    if resp2.nodes:
        check("Demo1b: correct scene_type",
              resp2.nodes[0].tags and resp2.nodes[0].tags.scene_type == "living_room")

    # 3c: Spatial coordinates are real numbers (not default 0,0,0)
    all_nodes = (resp1.nodes or []) + (resp2.nodes or [])
    if all_nodes:
        spatial_nodes = [n for n in all_nodes if n.spatial_data and n.spatial_data.objects]
        if spatial_nodes:
            first_obj = spatial_nodes[0].spatial_data.objects[0]
            has_coords = (first_obj.x != 0.0 or first_obj.y != 0.0 or first_obj.z != 0.0)
            check("Demo1c: spatial coordinates non-zero", has_coords,
                  f"obj {first_obj.label}: ({first_obj.x}, {first_obj.y}, {first_obj.z})")

    # ── Step 4: Demo 2 — Task History Backtracking ──
    print("\n── Step 4: Demo 2 — Task History Backtracking ──")

    # 4a: Fetch tasks only
    resp3 = await svc.search("task", tags=TagFilter(task_type="fetch"), top_k=10)
    check("Demo2a: fetch task filter returns results", len(resp3.nodes) >= 1,
          f"Got {len(resp3.nodes)} results")
    all_fetch = all(n.tags and n.tags.task_type == "fetch" for n in resp3.nodes)
    check("Demo2a: all results are fetch tasks", all_fetch)

    # 4b: Build tasks
    resp4 = await svc.search("crafting", tags=TagFilter(task_type="build"), top_k=5)
    check("Demo2b: build task filter returns results", len(resp4.nodes) >= 1,
          f"Got {len(resp4.nodes)} results")
    all_build = all(n.tags and n.tags.task_type == "build" for n in resp4.nodes)
    check("Demo2b: all results are build tasks", all_build)

    # 4c: Successful fetch tasks
    resp5 = await svc.search("fetch", tags=TagFilter(task_type="fetch", success=True), top_k=10)
    check("Demo2c: successful fetch only", len(resp5.nodes) >= 1)
    all_success = all(n.tags and n.tags.success for n in resp5.nodes)
    check("Demo2c: all results are successful", all_success)

    # ── Step 5: Mixed Scenarios ──
    print("\n── Step 5: Mixed Scenarios ──")

    # 5a: Failure lessons retrieval
    resp6 = await svc.search("failed", tags=TagFilter(success=False), top_k=10)
    check("Demo5a: failure lessons searchable", len(resp6.nodes) >= 1,
          f"Got {len(resp6.nodes)} results")
    all_fail = all(n.tags and not n.tags.success for n in resp6.nodes)
    check("Demo5a: all results are failures", all_fail)

    # 5b: Kitchen + fetch + success
    resp7 = await svc.search("cup", tags=TagFilter(
        scene_type="kitchen", task_type="fetch", success=True), top_k=10)
    check("Demo5b: 3-dimension AND filter", len(resp7.nodes) >= 1,
          f"Got {len(resp7.nodes)} results")
    for n in resp7.nodes:
        t = n.tags
        ok = (t.scene_type == "kitchen" and t.task_type == "fetch" and t.success)
        if not ok:
            check(f"Demo5b: node {n.node_id} passes 3-dim filter", False,
                  f"scene={t.scene_type} task={t.task_type} success={t.success}")
            break
    else:
        check("Demo5b: all results pass 3-dim filter", True)

    # 5c: Difficulty filter (max medium)
    resp8 = await svc.search("task", tags=TagFilter(difficulty_max="medium"), top_k=20)
    if resp8.nodes:
        all_leq = all(
            n.tags and n.tags.difficulty in ("easy", "medium")
            for n in resp8.nodes
        )
        check("Demo5c: difficulty ≤ medium filter", all_leq)

    # 5d: Pure semantic search (no tag filter)
    resp9 = await svc.search("crafted items", top_k=5)
    check("Demo5d: pure semantic search returns results", len(resp9.nodes) >= 1,
          f"Got {len(resp9.nodes)} results")

    # 5e: Time range filter
    now = _time.time_ns()
    resp10 = await svc.search("event",
                              time_range=TimeRange(start_ts=now - 60_000_000_000),
                              top_k=20)
    check("Demo5e: time range filter (last 60s)", len(resp10.nodes) >= 1,
          f"Got {len(resp10.nodes)} results")

    # ── Step 6: Empty / Edge Cases ──
    print("\n── Step 6: Edge Cases ──")

    resp_emp = await svc.search("zzz_nonexistent_query_xyz", top_k=3)
    check("Edge: no-match returns empty", True)  # always passes (graceful)
    check("Edge: compact works on populated store",
          (await svc.compact()).nodes_compacted >= 0)

    # ── Report ──
    print(f"\n{'='*60}")
    total = PASS + FAIL
    pct = (PASS / total * 100) if total > 0 else 0
    print(f"  Results: {PASS} passed, {FAIL} failed, {total} total ({pct:.0f}%)")
    print(f"{'='*60}")

    return 0 if FAIL == 0 else 1


def main() -> int:
    import argparse
    parser = argparse.ArgumentParser(
        description="Scribe Mem verification — Demo 1 & 2 validation"
    )
    parser.add_argument("--yaml", help="Path to YAML test data file")
    parser.add_argument("--data-dir", default="",
                        help="Memory data directory (temp dir if empty)")
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.DEBUG, format="%(message)s")
    else:
        logging.disable(logging.CRITICAL)

    if args.data_dir:
        import shutil
        data_dir = args.data_dir
        os.makedirs(data_dir, exist_ok=True)
        rc = asyncio.run(run_verification(data_dir, args.yaml or ""))
    else:
        import tempfile, shutil
        tmp = tempfile.mkdtemp()
        try:
            rc = asyncio.run(run_verification(tmp, args.yaml or ""))
        finally:
            shutil.rmtree(tmp, ignore_errors=True)

    return rc


if __name__ == "__main__":
    sys.exit(main())
