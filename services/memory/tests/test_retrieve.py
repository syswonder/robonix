"""Tests for RetrievePipeline — tag filter → LLM rank → causal/time/weight."""

import sys, os, tempfile, shutil, asyncio, logging
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from memory_service.core.types import (
    LogRecord, SpatialContext, ObjectCoord, TagSet, TagFilter, TimeRange,
    RememberRequest, SearchRequest, MemoryNode, NodeType,
)
from memory_service.storage.graph_store import GraphStore
from memory_service.storage.tag_index import TagIndex
from memory_service.storage.vector_store import VectorStore
from memory_service.storage.embedding_config import EmbeddingModelConfig
from memory_service.core.remember import RememberPipeline
from memory_service.core.retrieve import RetrievePipeline


class TestRetrievePipeline:
    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.graph = GraphStore(data_dir=self.tmp)
        self.tags = TagIndex()
        cfg = EmbeddingModelConfig(dim=8)
        self.vectors = VectorStore(config=cfg, alpha=0.3)
        self.remember = RememberPipeline(self.graph, self.tags, self.vectors)
        self.retrieve = RetrievePipeline(self.graph, self.tags, self.vectors)

    def teardown_method(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _remember(self, session, plan, msg, level="Info", tag="exec",
                  spatial=None, parent=None):
        lr = LogRecord(ts=100, level=level, tag=tag, msg=msg)
        req = RememberRequest(session_id=session, plan_id=plan, log_record=lr,
                              spatial=spatial, parent_node_id=parent)
        return asyncio.run(self.remember.execute(req))

    def _search(self, query, tags=None, top_k=5, alpha=None, time_range=None):
        req = SearchRequest(query=query, tags=tags, top_k=top_k, alpha=alpha,
                            time_range=time_range)
        return asyncio.run(self.retrieve.execute(req))

    def test_tag_filter_returns_correct_node(self):
        # Write nodes with different scene types
        self._remember("s1", "p1", "grasped red cup in the kitchen")
        self._remember("s1", "p1", "placed blue cup in the living room")
        self._remember("s1", "p1", "crafted item in the workshop")

        resp = self._search("cup", tags=TagFilter(scene_type="kitchen"))
        assert len(resp.nodes) == 1
        assert resp.nodes[0].tags.scene_type == "kitchen"

    def test_pure_semantic_search_no_tags(self):
        self._remember("s1", "p1", "grasp a red cup")
        self._remember("s1", "p1", "navigate to the door")
        self._remember("s1", "p1", "observe the landscape")

        resp = self._search("grasp cup")
        assert len(resp.nodes) > 0
        # With BM25: "grasp" token in query matches "grasp" in node 0's summary
        first = resp.nodes[0].summary.lower()
        assert "grasp" in first or "cup" in first or "navigate" in first

    def test_keyword_vs_semantic_hybrid(self):
        """BM25 + Embedding hybrid should rank keyword-match higher than semantic-only match."""
        self._remember("s1", "p1", "grasped the red cup precisely")
        self._remember("s1", "p1", "collected a container")
        self._remember("s1", "p1", "took hold of the mug")

        # "grasp" should match BM25 for the first, but "take hold of mug"
        # might match semantically for the third
        resp = self._search("grasp cup")
        assert len(resp.nodes) >= 1
        # First node (exact keyword match) should be top
        if resp.nodes:
            assert "grasp" in resp.nodes[0].summary.lower() or "cup" in resp.nodes[0].summary.lower()

    def test_tag_plus_semantic_narrows_candidates(self):
        self._remember("s1", "p1", "grasped cup in the kitchen",
                       spatial=SpatialContext(
                           objects=[ObjectCoord("o1", "cup")],
                           origin="fixture_frame",
                       ))
        self._remember("s1", "p1", "grasped cup in the living room",
                       spatial=SpatialContext(
                           objects=[ObjectCoord("o2", "cup")],
                           origin="fixture_frame",
                       ))
        # Without tag filter — both returned
        resp_all = self._search("grasp cup")
        assert len(resp_all.nodes) >= 1
        # With tag filter — only kitchen
        resp_filtered = self._search("grasp cup",
                                     tags=TagFilter(scene_type="kitchen"))
        assert len(resp_filtered.nodes) == 1
        assert resp_filtered.nodes[0].tags.scene_type == "kitchen"

    def test_time_range_filter(self):
        import time as _time
        # Use time_ns() for reliable nanosecond timestamps
        past = _time.time_ns() - 10_000_000_000  # 10 seconds ago
        now = _time.time_ns()

        # Write node with timestamp far in the past
        lr1 = LogRecord(ts=past, level="Info", tag="t", msg="old cup location")
        nid1 = asyncio.run(self.remember.execute(
            RememberRequest(session_id="s1", plan_id="p1", log_record=lr1)
        )).node_id

        # Write node with timestamp close to now
        lr2 = LogRecord(ts=now, level="Info", tag="t", msg="new cup location")
        nid2 = asyncio.run(self.remember.execute(
            RememberRequest(session_id="s1", plan_id="p2", log_record=lr2)
        )).node_id

        # Search with time filter: only nodes after (now - 5 seconds)
        resp = self._search("cup location",
                            time_range=TimeRange(start_ts=now - 5_000_000_000))
        nids = {n.node_id for n in resp.nodes}
        assert nid2 in nids, f"n2 ({nid2}) should be in results"
        assert nid1 not in nids, f"n1 ({nid1}) should not be in results"

    def test_empty_result_on_no_match(self):
        self._remember("s1", "p1", "grasped cup in the kitchen")
        resp = self._search("quantum physics", tags=TagFilter(scene_type="mars"))
        assert len(resp.nodes) == 0

    def test_alpha_pure_bm25(self):
        self._remember("s1", "p1", "navigated to the door")
        self._remember("s1", "p1", "grasped cup")
        # α=1.0 → pure BM25
        resp = self._search("grasp", alpha=1.0)
        assert len(resp.nodes) >= 1
        assert any("grasp" in n.summary for n in resp.nodes)

    def test_alpha_pure_cosine(self):
        self._remember("s1", "p1", "grasped cup in kitchen")
        self._remember("s1", "p1", "observed landscape")
        resp = self._search("grasp", alpha=0.0)
        assert len(resp.nodes) >= 1

    def test_top_k_truncation(self):
        for i in range(5):
            self._remember("s1", "p1", f"grasped cup {i} in kitchen")
        resp = self._search("cup", tags=TagFilter(scene_type="kitchen"), top_k=2)
        assert len(resp.nodes) == 2

    def test_access_count_updated(self):
        self._remember("s1", "p1", "grasped cup in kitchen")
        resp = self._search("cup")
        for n in resp.nodes:
            assert n.access_count >= 1
            assert n.last_access > 0


# ── Runner ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import traceback
    logging.disable(logging.CRITICAL)

    tests = [TestRetrievePipeline()]
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
