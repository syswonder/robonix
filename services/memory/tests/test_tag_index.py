"""Tests for TagIndex — inverted index insert, query, remove, rebuild."""

import sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from memory_service.core.types import TagSet, TagFilter, MemoryNode, NodeType
from memory_service.storage.tag_index import TagIndex


class TestInsertQuery:
    def setup_method(self):
        self.idx = TagIndex()

    def test_insert_and_query_scene_type(self):
        tags = TagSet(scene_type="kitchen", action_type="grasp", task_type="fetch")
        self.idx.insert(1, tags)
        self.idx.insert(2, TagSet(scene_type="living_room", action_type="grasp"))

        result = self.idx.query(TagFilter(scene_type="kitchen"))
        assert result == {1}

    def test_query_action_type(self):
        self.idx.insert(1, TagSet(action_type="grasp"))
        self.idx.insert(2, TagSet(action_type="place"))
        self.idx.insert(3, TagSet(action_type="grasp"))
        result = self.idx.query(TagFilter(action_type="grasp"))
        assert result == {1, 3}

    def test_query_objects_intersection(self):
        self.idx.insert(1, TagSet(objects_present=["cup", "table"]))
        self.idx.insert(2, TagSet(objects_present=["cup"]))
        self.idx.insert(3, TagSet(objects_present=["table"]))
        # Must contain BOTH cup AND table
        result = self.idx.query(TagFilter(objects=["cup", "table"]))
        assert result == {1}

    def test_query_success(self):
        self.idx.insert(1, TagSet(success=True, action_type="grasp"))
        self.idx.insert(2, TagSet(success=False, action_type="grasp"))
        assert self.idx.query(TagFilter(success=True)) == {1}
        assert self.idx.query(TagFilter(success=False)) == {2}

    def test_query_task_type(self):
        self.idx.insert(1, TagSet(task_type="fetch"))
        self.idx.insert(2, TagSet(task_type="build"))
        assert self.idx.query(TagFilter(task_type="fetch")) == {1}

    def test_query_difficulty_max(self):
        self.idx.insert(1, TagSet(difficulty="easy", task_type="fetch"))
        self.idx.insert(2, TagSet(difficulty="medium", task_type="fetch"))
        self.idx.insert(3, TagSet(difficulty="hard", task_type="fetch"))
        # difficulty ≤ medium
        result = self.idx.query(TagFilter(difficulty_max="medium"))
        assert result == {1, 2}

    def test_multi_dimension_and(self):
        self.idx.insert(1, TagSet(scene_type="kitchen", action_type="grasp", task_type="fetch"))
        self.idx.insert(2, TagSet(scene_type="kitchen", action_type="place", task_type="fetch"))
        self.idx.insert(3, TagSet(scene_type="living_room", action_type="grasp", task_type="fetch"))
        result = self.idx.query(TagFilter(scene_type="kitchen", action_type="grasp"))
        assert result == {1}

    def test_empty_filter_returns_all(self):
        self.idx.insert(1, TagSet(scene_type="kitchen"))
        self.idx.insert(2, TagSet(scene_type="lab"))
        result = self.idx.query(TagFilter())
        assert result == {1, 2}

    def test_no_match_returns_empty(self):
        self.idx.insert(1, TagSet(scene_type="kitchen"))
        result = self.idx.query(TagFilter(scene_type="mars"))
        assert result == set()


class TestRemove:
    def setup_method(self):
        self.idx = TagIndex()

    def test_remove_cleans_all_dimensions(self):
        tags = TagSet(scene_type="kitchen", action_type="grasp", task_type="fetch")
        self.idx.insert(1, tags)
        self.idx.remove(1)
        # All queries should return empty
        assert self.idx.query(TagFilter(scene_type="kitchen")) == set()
        assert self.idx.query(TagFilter(action_type="grasp")) == set()
        assert self.idx.query(TagFilter()) == set()

    def test_remove_nonexistent(self):
        self.idx.remove(999)  # should not raise


class TestRebuild:
    def test_rebuild_from_nodes(self):
        idx = TagIndex()
        n1 = MemoryNode(node_id=1, tags=TagSet(scene_type="kitchen", task_type="fetch"))
        n2 = MemoryNode(node_id=2, tags=TagSet(scene_type="lab", task_type="build"))
        n3 = MemoryNode(node_id=3, tags=None)  # should be skipped
        idx.rebuild([n1, n2, n3])
        assert idx.count() == 2
        assert idx.query(TagFilter(scene_type="kitchen")) == {1}


class TestCount:
    def test_count_reflects_indexed_nodes(self):
        idx = TagIndex()
        assert idx.count() == 0
        idx.insert(1, TagSet())
        assert idx.count() == 1
        idx.insert(2, TagSet())
        assert idx.count() == 2
        idx.remove(1)
        assert idx.count() == 1


# ── Runner ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import traceback
    tests = [TestInsertQuery(), TestRemove(), TestRebuild(), TestCount()]
    passed = failed = 0
    for obj in tests:
        cls_name = type(obj).__name__
        for name in dir(obj):
            if name.startswith("test_"):
                try:
                    if hasattr(obj, "setup_method"):
                        obj.setup_method()
                    getattr(obj, name)()
                    print(f"  PASS {cls_name}.{name}")
                    passed += 1
                except Exception:
                    print(f"  FAIL {cls_name}.{name}")
                    traceback.print_exc()
                    failed += 1
    print(f"\n{passed} passed, {failed} failed")
    sys.exit(1 if failed else 0)
