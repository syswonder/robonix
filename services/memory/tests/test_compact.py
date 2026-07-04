"""Tests for CompactPipeline — ShortTerm → LongTerm promotion."""

import sys, os, tempfile, shutil, asyncio, logging
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from memory_service.core.types import MemoryNode, NodeType
from memory_service.storage.graph_store import GraphStore
from memory_service.core.compact import CompactPipeline, DEFAULT_SHORT_TERM_THRESHOLD


class TestCompactPipeline:
    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.graph = GraphStore(data_dir=self.tmp)
        self.pipeline = CompactPipeline(self.graph, short_term_threshold=3)

    def teardown_method(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _run(self):
        return asyncio.run(self.pipeline.execute())

    def test_below_threshold_skips(self):
        for i in range(2):
            self.graph.add_node(MemoryNode(summary=f"n{i}", node_type=NodeType.SHORT_TERM))
        resp = self._run()
        assert resp.nodes_compacted == 0
        assert "No compaction needed" in resp.summary

    def test_above_threshold_promotes(self):
        for i in range(5):
            self.graph.add_node(MemoryNode(summary=f"n{i}", node_type=NodeType.SHORT_TERM))
        resp = self._run()
        assert resp.nodes_compacted == 2  # 5 - 3 = 2 overflow
        # Check that short-term count is now ≤ threshold
        shorts = self.graph.list_by_type(NodeType.SHORT_TERM, limit=100)
        assert len(shorts) <= 3

    def test_empty_store(self):
        resp = self._run()
        assert resp.nodes_compacted == 0


# ── Runner ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import traceback
    logging.disable(logging.CRITICAL)
    tests = [TestCompactPipeline()]
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
