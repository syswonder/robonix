"""Tests for GraphStore — node CRUD, edges, persistence, promote."""

import sys, os, tempfile, shutil
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from memory_service.core.types import MemoryNode, NodeType, LogRecord, TagSet, SpatialContext
from memory_service.storage.graph_store import GraphStore


class TestNodeCRUD:
    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.store = GraphStore(data_dir=self.tmp)

    def teardown_method(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_add_and_get(self):
        n = MemoryNode(summary="test node")
        nid = self.store.add_node(n)
        assert nid >= 0
        got = self.store.get_node(nid)
        assert got is not None
        assert got.summary == "test node"

    def test_add_multiple_ids_increment(self):
        ids = []
        for i in range(5):
            n = MemoryNode(summary=f"node {i}")
            ids.append(self.store.add_node(n))
        assert len(set(ids)) == 5
        assert self.store.count() == 5

    def test_get_nodes_batch(self):
        ids = [self.store.add_node(MemoryNode(summary=f"n{i}")) for i in range(3)]
        nodes = self.store.get_nodes(ids)
        assert len(nodes) == 3

    def test_get_node_missing(self):
        assert self.store.get_node(99999) is None

    def test_update_node(self):
        nid = self.store.add_node(MemoryNode(summary="v1", weight=0.5, version=1))
        updated = MemoryNode(node_id=nid, summary="v2", weight=0.8, version=1)
        self.store.update_node(nid, updated)
        got = self.store.get_node(nid)
        assert got.summary == "v2"
        assert got.weight == 0.8
        assert got.version == 2  # incremented

    def test_update_version_conflict(self):
        nid = self.store.add_node(MemoryNode(summary="orig", version=1))
        updated = MemoryNode(node_id=nid, summary="new", version=2)  # wrong version
        try:
            self.store.update_node(nid, updated)
            assert False, "should have raised"
        except ValueError as e:
            assert "Version conflict" in str(e)

    def test_remove_node(self):
        nid = self.store.add_node(MemoryNode(summary="gone"))
        assert self.store.remove_node(nid) is True
        assert self.store.get_node(nid) is None
        assert self.store.count() == 0

    def test_remove_node_missing(self):
        assert self.store.remove_node(99999) is False


class TestNodeListing:
    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.store = GraphStore(data_dir=self.tmp)

    def teardown_method(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_list_by_type(self):
        for i in range(3):
            n = MemoryNode(summary=f"short {i}", node_type=NodeType.SHORT_TERM)
            self.store.add_node(n)
        n = MemoryNode(summary="long", node_type=NodeType.LONG_TERM)
        self.store.add_node(n)

        shorts = self.store.list_by_type(NodeType.SHORT_TERM)
        assert len(shorts) == 3
        longs = self.store.list_by_type(NodeType.LONG_TERM)
        assert len(longs) == 1

    def test_list_by_time(self):
        import time as _time
        base = _time.time_ns()
        for i in range(3):
            n = MemoryNode(summary=f"t{i}", timestamp=base + i * 1_000_000_000)
            self.store.add_node(n)

        results = self.store.list_by_time(base, base + int(1.5 * 1_000_000_000))
        assert len(results) == 2  # t0, t1; t2 at base+2s is beyond base+1.5s


class TestEdges:
    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.store = GraphStore(data_dir=self.tmp)

    def teardown_method(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_add_and_query_edges(self):
        n1 = self.store.add_node(MemoryNode(summary="parent"))
        n2 = self.store.add_node(MemoryNode(summary="child"))
        self.store.add_edge(n1, n2)

        assert self.store.get_children(n1) == [n2]
        assert self.store.get_parents(n2) == [n1]

    def test_edge_updates_causal_chain(self):
        n1 = self.store.add_node(MemoryNode(summary="A"))
        n2 = self.store.add_node(MemoryNode(summary="B"))
        self.store.add_edge(n1, n2)
        child = self.store.get_node(n2)
        assert n1 in child.causal_chain

    def test_get_all_edges(self):
        n1 = self.store.add_node(MemoryNode(summary="A"))
        n2 = self.store.add_node(MemoryNode(summary="B"))
        n3 = self.store.add_node(MemoryNode(summary="C"))
        self.store.add_edge(n1, n2)
        self.store.add_edge(n1, n3)
        edges = self.store.get_all_edges()
        assert len(edges) == 2

    def test_edge_missing_node_raises(self):
        n1 = self.store.add_node(MemoryNode(summary="exists"))
        try:
            self.store.add_edge(n1, 99999)
            assert False
        except ValueError:
            pass

    def test_remove_node_cleans_edges(self):
        n1 = self.store.add_node(MemoryNode(summary="A"))
        n2 = self.store.add_node(MemoryNode(summary="B"))
        self.store.add_edge(n1, n2)
        self.store.remove_node(n2)
        assert self.store.get_children(n1) == []


class TestPersistence:
    def test_save_and_reload(self):
        tmp = tempfile.mkdtemp()
        try:
            s1 = GraphStore(data_dir=tmp)
            n1 = s1.add_node(MemoryNode(summary="persisted"))
            n2 = s1.add_node(MemoryNode(summary="child"))
            s1.add_edge(n1, n2)

            # Reload
            s2 = GraphStore(data_dir=tmp)
            assert s2.count() == 2
            got = s2.get_node(n1)
            assert got.summary == "persisted"
            assert s2.get_children(n1) == [n2]
        finally:
            shutil.rmtree(tmp, ignore_errors=True)


class TestPromote:
    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.store = GraphStore(data_dir=self.tmp)

    def teardown_method(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def test_promote_short_to_long(self):
        nid = self.store.add_node(MemoryNode(summary="short", node_type=NodeType.SHORT_TERM))
        assert nid < 1000
        new_id = self.store.promote_to_long_term(nid)
        assert new_id is not None
        assert new_id >= 1000
        pn = self.store.get_node(new_id)
        assert pn.node_type == NodeType.LONG_TERM
        # Old ID should be gone
        assert self.store.get_node(nid) is None

    def test_promote_rewrites_child_causal_chain(self):
        """After promote, child nodes' causal_chain references the new parent ID."""
        # parent → child: causal edge + causal_chain in child
        parent_id = self.store.add_node(MemoryNode(summary="parent",
                                                    node_type=NodeType.SHORT_TERM))
        child = MemoryNode(summary="child", node_type=NodeType.SHORT_TERM)
        child_id = self.store.add_node(child)
        self.store.add_edge(parent_id, child_id)

        # Verify initial state
        child_node = self.store.get_node(child_id)
        assert parent_id in child_node.causal_chain
        assert parent_id in self.store.get_parents(child_id)

        # Promote parent: old ID → new ID (1000+)
        new_parent_id = self.store.promote_to_long_term(parent_id)
        assert new_parent_id is not None
        assert new_parent_id >= 1000
        assert self.store.get_node(parent_id) is None

        # Verify child's causal_chain was rewritten
        child_node = self.store.get_node(child_id)
        assert parent_id not in child_node.causal_chain, \
            f"stale parent {parent_id} still in child.causal_chain"
        assert new_parent_id in child_node.causal_chain, \
            f"new parent {new_parent_id} missing from child.causal_chain"

        # Verify adjacency sets match causal_chain
        adjacency_parents = self.store.get_parents(child_id)
        assert new_parent_id in adjacency_parents
        assert parent_id not in adjacency_parents
        assert set(child_node.causal_chain) == set(adjacency_parents), \
            f"causal_chain {child_node.causal_chain} != adjacency {adjacency_parents}"

    def test_promote_with_multiple_children(self):
        """Promote a node with 3 children — all causal_chains updated."""
        parent_id = self.store.add_node(MemoryNode(summary="parent",
                                                    node_type=NodeType.SHORT_TERM))
        child_ids = []
        for i in range(3):
            cid = self.store.add_node(MemoryNode(summary=f"child-{i}",
                                                  node_type=NodeType.SHORT_TERM))
            self.store.add_edge(parent_id, cid)
            child_ids.append(cid)

        new_parent_id = self.store.promote_to_long_term(parent_id)

        for cid in child_ids:
            child = self.store.get_node(cid)
            assert parent_id not in child.causal_chain, \
                f"child {cid}: stale parent {parent_id}"
            assert new_parent_id in child.causal_chain, \
                f"child {cid}: missing new parent {new_parent_id}"

    def test_promote_long_term_does_nothing(self):
        # Force the node into the long-term ID range explicitly
        n = MemoryNode(summary="long", node_type=NodeType.LONG_TERM, node_id=5000)
        nid = self.store.add_node(n)
        assert nid == 5000
        assert self.store.promote_to_long_term(nid) is None


# ── Runner ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import traceback
    tests = [TestNodeCRUD(), TestNodeListing(), TestEdges(), TestPersistence(), TestPromote()]
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
