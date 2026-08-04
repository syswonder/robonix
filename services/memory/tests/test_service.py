"""End-to-end tests for MemoryService — Demo 1 & 2 validation.

Demo 1: Object Spatiotemporal Backtracking — "where was the cup yesterday?"
    Write memory with spatial coordinates → search by scene_type → find it.

Demo 2: Task History Backtracking — "what did I ask you to do before?"
    Write memory with task_type → search by task_type → find it.
"""

import sys, os, tempfile, shutil, asyncio, logging
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from memory_service.core.types import (
    LogRecord, SpatialContext, ObjectCoord, TagFilter, NodeType,
)
from memory_service.service import MemoryService


class TestMemoryService:
    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.svc = MemoryService(data_dir=self.tmp)
        asyncio.run(self.svc.init())

    def teardown_method(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    # ── End-to-end: remember → search round-trip ──────────────────────

    def test_remember_search_roundtrip(self):
        """Write one node, search for it, verify it comes back."""
        nid = asyncio.run(self.svc.remember_from_log(
            session_id="sess-1", plan_id="plan-1",
            level="Info", tag="exec",
            msg="robot grasped the red cup from the kitchen counter",
            objects=[("scene.obj.cup_001", "red cup", 1.0, 2.0, 0.8)],
            spatial_origin="fixture_frame",
        ))
        assert nid >= 0

        resp = asyncio.run(self.svc.search("grasp red cup", top_k=3))
        assert len(resp.nodes) >= 1
        found = any(n.node_id == nid for n in resp.nodes)
        assert found, f"Node {nid} not found in search results"

    # ── Demo 1: Object Spatiotemporal Backtracking ─────────────────────

    def test_demo1_object_spatiotemporal(self):
        """Demo 1: "Where was the cup yesterday?" — write with spatial coords → filter by scene."""
        # Day 1: place cup in kitchen
        asyncio.run(self.svc.remember_from_log(
            session_id="sess-day1", plan_id="plan-1",
            level="Info", tag="exec",
            msg="placed red cup on the kitchen counter",
            objects=[("scene.obj.cup_001", "red cup", 1.0, 2.0, 0.8)],
            spatial_origin="fixture_frame",
        ))

        # Day 1: also placed cup in living room (decoy)
        asyncio.run(self.svc.remember_from_log(
            session_id="sess-day1", plan_id="plan-2",
            level="Info", tag="exec",
            msg="placed blue cup on the living room coffee table",
            objects=[("scene.obj.cup_002", "blue cup", 3.0, 4.0, 0.5)],
            spatial_origin="fixture_frame",
        ))

        # Day 2: ask "where was the red cup yesterday?"
        # Search: scene_type=kitchen + semantic "cup"
        resp = asyncio.run(self.svc.search(
            "red cup",
            tags=TagFilter(scene_type="kitchen"),
            top_k=3,
        ))

        # Should find the kitchen memory, not the living room one
        assert len(resp.nodes) >= 1
        kitchen_hits = [n for n in resp.nodes
                        if n.tags and n.tags.scene_type == "kitchen"]
        assert len(kitchen_hits) >= 1, "Should find kitchen memory"

        # Verify spatial data is preserved
        kitchen_node = kitchen_hits[0]
        if kitchen_node.spatial_data:
            labels = [o.label for o in kitchen_node.spatial_data.objects]
            assert "red cup" in labels, f"Expected 'red cup' in objects, got {labels}"

    # ── Demo 2: Task History Backtracking ──────────────────────────────

    def test_demo2_task_history(self):
        """Demo 2: "What did I ask you to do before?" — write with task_type → filter by task."""
        # Earlier tasks
        asyncio.run(self.svc.remember_from_log(
            session_id="sess-1", plan_id="plan-fetch",
            level="Info", tag="pilot",
            msg="executed fetch task: brought the red cup to user",
        ))
        asyncio.run(self.svc.remember_from_log(
            session_id="sess-1", plan_id="plan-build",
            level="Info", tag="pilot",
            msg="executed build task: crafted a wooden plank",
        ))
        asyncio.run(self.svc.remember_from_log(
            session_id="sess-1", plan_id="plan-explore",
            level="Info", tag="pilot",
            msg="executed explore task: scanned the living room",
        ))

        # Query: "what fetch tasks did I do?"
        resp = asyncio.run(self.svc.search(
            "fetch task",
            tags=TagFilter(task_type="fetch"),
            top_k=5,
        ))

        assert len(resp.nodes) >= 1
        # Should only return fetch tasks
        for node in resp.nodes:
            assert node.tags is not None
            assert node.tags.task_type == "fetch", \
                f"Expected task_type='fetch', got '{node.tags.task_type}'"

    # ── Mixed: tag + time ──────────────────────────────────────────────

    def test_tag_and_time_combined(self):
        """Combined spatial + temporal filtering."""
        import time as _time

        past = _time.time_ns() - 10_000_000_000  # 10s ago
        now = _time.time_ns()

        from memory_service.core.types import TimeRange, LogRecord, RememberRequest

        # Old kitchen memory
        lr_old = LogRecord(ts=past, level="Info", tag="exec",
                          msg="grasped cup in the kitchen yesterday")
        asyncio.run(self.svc.remember(
            session_id="s1", plan_id="p1", log_record=lr_old,
            spatial=SpatialContext(
                objects=[ObjectCoord("o1", "cup", 1.0, 2.0, 0.5)],
                origin="fixture_frame",
            ),
        ))

        # Recent kitchen memory
        lr_recent = LogRecord(ts=now, level="Info", tag="exec",
                             msg="grasped cup in the kitchen today")
        asyncio.run(self.svc.remember(
            session_id="s1", plan_id="p2", log_record=lr_recent,
            spatial=SpatialContext(
                objects=[ObjectCoord("o2", "cup", 1.1, 2.1, 0.5)],
                origin="fixture_frame",
            ),
        ))

        # Search: kitchen + recent (last 5 seconds)
        resp = asyncio.run(self.svc.search(
            "cup in kitchen",
            tags=TagFilter(scene_type="kitchen"),
            time_range=TimeRange(start_ts=now - 5_000_000_000),
            top_k=5,
        ))

        # Should find only the recent kitchen memory
        assert len(resp.nodes) >= 1
        for node in resp.nodes:
            assert node.tags.scene_type == "kitchen"
            # All should be recent
            assert node.timestamp >= now - 5_000_000_000

    # ── Compact integration ────────────────────────────────────────────

    def test_service_compact(self):
        """Compact via service API — respects default threshold (50)."""
        for i in range(10):
            asyncio.run(self.svc.remember_from_log(
                session_id="s1", plan_id="p1",
                level="Info", tag="t", msg=f"event {i}",
            ))
        before = self.svc.graph.count()
        resp = asyncio.run(self.svc.compact())
        after = self.svc.graph.count()
        # Node count stays the same (promoted, not deleted)
        assert after == before
        # With 10 nodes < threshold 50, no compaction happens
        # (verified by nodes_compacted being 0)
        assert resp.nodes_compacted == 0
        assert "No compaction needed" in resp.summary

    # ── Cross-boot persistence ──────────────────────────────────────

    def test_cross_reboot_persistence(self):
        """Simulate rbnx boot restart: write data, destroy, recreate, verify.

        Cycle:
          1. Boot 1: create service → write 3 nodes → search OK
          2. Shutdown: discard service (indices lost from memory)
          3. Boot 2: create NEW service from same data_dir → init rebuild
          4. Verify: search finds all 3 nodes from Boot 1
        """
        import gc

        # ── Boot 1: write data ──
        svc1 = MemoryService(data_dir=self.tmp)
        asyncio.run(svc1.init())

        ids = []
        ids.append(asyncio.run(svc1.remember_from_log(
            session_id="sess-reboot", plan_id="plan-1",
            level="Info", tag="exec",
            msg="grasped red cup in the kitchen",
            objects=[("scene.obj.cup_001", "red cup", 1.0, 2.0, 0.8)],
            spatial_origin="fixture_frame",
        )))
        ids.append(asyncio.run(svc1.remember_from_log(
            session_id="sess-reboot", plan_id="plan-2",
            level="Error", tag="exec",
            msg="failed to grasp slippery glass on the kitchen sink",
            objects=[("scene.obj.glass_001", "glass", 1.2, 2.3, 1.0)],
            spatial_origin="fixture_frame",
        )))
        ids.append(asyncio.run(svc1.remember_from_log(
            session_id="sess-reboot", plan_id="plan-3",
            level="Info", tag="pilot",
            msg="crafted stone axe in the workshop",
            objects=[("scene.obj.axe_001", "stone axe", 7.1, 8.1, 1.0)],
            spatial_origin="fixture_frame",
        )))

        # Verify Boot 1 search works
        resp1 = asyncio.run(svc1.search("cup kitchen", top_k=5))
        assert len(resp1.nodes) >= 2, f"Boot 1: expected >=2 results, got {len(resp1.nodes)}"

        # ── Shutdown: destroy all references ──
        del svc1
        gc.collect()

        # ── Boot 2: recreate from same data_dir ──
        svc2 = MemoryService(data_dir=self.tmp)
        asyncio.run(svc2.init())

        # Verify node count
        assert svc2.graph.count() == 3, \
            f"Boot 2: expected 3 nodes, got {svc2.graph.count()}"
        assert svc2.tags.count() == 3, \
            f"Boot 2: expected 3 tags indexed, got {svc2.tags.count()}"
        if svc2.vectors.is_semantic or svc2.vectors._embedding_enabled:
            assert svc2.vectors.count() == 3, \
                f"Boot 2: expected 3 vectors indexed, got {svc2.vectors.count()}"

        # Verify search works on rebuilt indices
        resp2 = asyncio.run(svc2.search("cup kitchen", top_k=5))
        assert len(resp2.nodes) >= 2, \
            f"Boot 2: expected >=2 results, got {len(resp2.nodes)}"

        # Verify specific nodes are findable
        for nid in ids:
            assert nid in svc2.graph.all_ids(), \
                f"Node {nid} missing from graph after reboot"

        # Verify tag filter still works on reloaded data
        resp3 = asyncio.run(svc2.search(
            "grasp",
            tags=TagFilter(scene_type="kitchen", success=True),
            top_k=5,
        ))
        assert len(resp3.nodes) >= 1
        for n in resp3.nodes:
            assert n.tags.scene_type == "kitchen"
            assert n.tags.success is True

        # Verify failure filter works
        resp4 = asyncio.run(svc2.search(
            "failed",
            tags=TagFilter(success=False),
            top_k=5,
        ))
        assert len(resp4.nodes) >= 1
        for n in resp4.nodes:
            assert n.tags.success is False


# ── Runner ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import traceback
    logging.disable(logging.CRITICAL)

    tests = [TestMemoryService()]
    passed = failed = 0
    for obj in tests:
        cls_name = type(obj).__name__
        for name in dir(obj):
            if name.startswith("test_"):
                try:
                    if hasattr(obj, "setup_method"):
                        obj.setup_method()
                    getattr(obj, name)()
                    if hasattr(obj, "teardown_method"):
                        obj.teardown_method()
                    print(f"  PASS {cls_name}.{name}")
                    passed += 1
                except Exception:
                    print(f"  FAIL {cls_name}.{name}")
                    traceback.print_exc()
                    failed += 1
                    if hasattr(obj, "teardown_method"):
                        obj.teardown_method()
    print(f"\n{passed} passed, {failed} failed")
    sys.exit(1 if failed else 0)
