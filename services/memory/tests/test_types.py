"""Tests for Scribe Mem core types — serialization round-trips and defaults."""

import sys
import os
import json

# Add the memory_service package to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from memory_service.core.types import (
    NodeType, CausalRelation, ConditionType, ForgetDecision,
    ObjectCoord, SpatialContext, TagSet, LogRecord, MemoryNode,
    CausalEdge, TimeRange, TagFilter,
    ForgetRisk, ForgetAuditEntry,
    RememberRequest, RememberResponse, SearchRequest, SearchResponse,
    CompactResponse,
    Condition, StepTemplate, Constraint, SkillTemplate, SkillSummary,
    difficulty_leq,
)


class TestObjectCoord:
    def test_defaults(self):
        o = ObjectCoord(obj_id="scene.obj.1", label="cup")
        assert o.x == 0.0 and o.y == 0.0 and o.z == 0.0

    def test_roundtrip(self):
        o = ObjectCoord(obj_id="a", label="red cup", x=1.0, y=2.0, z=3.0)
        d = o.to_dict()
        o2 = ObjectCoord.from_dict(d)
        assert o2.obj_id == "a"
        assert o2.label == "red cup"
        assert o2.x == 1.0 and o2.y == 2.0 and o2.z == 3.0


class TestSpatialContext:
    def test_empty(self):
        s = SpatialContext()
        assert s.objects == []
        assert s.origin == ""

    def test_roundtrip(self):
        s = SpatialContext(
            objects=[ObjectCoord(obj_id="o1", label="cup", x=1, y=2, z=3)],
            origin="robot_base"
        )
        s2 = SpatialContext.from_dict(s.to_dict())
        assert s2.origin == "robot_base"
        assert len(s2.objects) == 1
        assert s2.objects[0].label == "cup"


class TestTagSet:
    def test_defaults(self):
        t = TagSet()
        assert t.scene_type == ""
        assert t.action_type == ""
        assert t.success is True
        assert t.task_type == ""

    def test_roundtrip_full(self):
        t = TagSet(
            scene_type="kitchen",
            objects_present=["cup", "table"],
            region="north",
            action_type="grasp",
            success=True,
            tool_used=["gripper_v1"],
            task_type="fetch",
            difficulty="easy",
            intent="grab red cup",
            frequency=5,
            last_access=1000,
            quality_score=0.9,
        )
        t2 = TagSet.from_dict(t.to_dict())
        assert t2.scene_type == "kitchen"
        assert t2.objects_present == ["cup", "table"]
        assert t2.success is True
        assert t2.difficulty == "easy"

    def test_from_dict_none_lists(self):
        d = {"scene_type": "lab", "objects_present": None, "tool_used": None}
        t = TagSet.from_dict(d)
        assert t.objects_present == []
        assert t.tool_used == []

    def test_four_dimensions_separate_fields(self):
        t = TagSet(scene_type="workshop", action_type="craft", task_type="build")
        d = t.to_dict()
        # verify all four dimension groups exist
        assert "scene_type" in d
        assert "action_type" in d
        assert "task_type" in d
        assert "frequency" in d  # optimisation


class TestLogRecord:
    def test_roundtrip(self):
        lr = LogRecord(ts=123456789, level="Info", tag="pilot", msg="task started")
        lr2 = LogRecord.from_dict(lr.to_dict())
        assert lr2.ts == 123456789
        assert lr2.level == "Info"
        assert lr2.tag == "pilot"


class TestMemoryNode:
    def test_minimal_node(self):
        n = MemoryNode(node_id=1, summary="did something")
        d = n.to_dict()
        n2 = MemoryNode.from_dict(d)
        assert n2.node_id == 1
        assert n2.summary == "did something"
        assert n2.node_type == NodeType.SHORT_TERM

    def test_node_with_optional_none(self):
        n = MemoryNode(node_id=5, summary="no extras")
        d = n.to_dict()
        assert d["raw_log"] is None
        assert d["spatial_data"] is None
        assert d["tags"] is None
        n2 = MemoryNode.from_dict(d)
        assert n2.raw_log is None
        assert n2.spatial_data is None
        assert n2.tags is None

    def test_full_node_roundtrip(self):
        lr = LogRecord(ts=100, level="Warn", tag="exec", msg="failed")
        sp = SpatialContext(
            objects=[ObjectCoord(obj_id="o1", label="chair", x=1, y=2, z=0)],
            origin="fixture_frame",
        )
        tags = TagSet(scene_type="living_room", action_type="navigate", task_type="explore")
        n = MemoryNode(
            node_id=42,
            summary="navigated to living room, bumped chair",
            raw_log=lr,
            timestamp=100,
            spatial_data=sp,
            tags=tags,
            causal_chain=[10, 11],
            weight=0.75,
            embedding=[0.1, 0.2, 0.3],
            node_type=NodeType.LONG_TERM,
            created_at=100,
            version=2,
        )
        n2 = MemoryNode.from_dict(n.to_dict())
        assert n2.node_id == 42
        assert n2.summary == "navigated to living room, bumped chair"
        assert n2.raw_log is not None and n2.raw_log.level == "Warn"
        assert n2.spatial_data is not None and len(n2.spatial_data.objects) == 1
        assert n2.tags is not None and n2.tags.scene_type == "living_room"
        assert n2.causal_chain == [10, 11]
        assert n2.weight == 0.75
        assert n2.embedding == [0.1, 0.2, 0.3]
        assert n2.node_type == NodeType.LONG_TERM
        assert n2.version == 2


class TestNodeType:
    def test_enum_values(self):
        assert NodeType.SHORT_TERM.value == "short_term"
        assert NodeType.LONG_TERM.value == "long_term"
        assert NodeType.SKILL.value == "skill"
        assert NodeType.FIXED.value == "fixed"
        assert NodeType.LESSON.value == "lesson"

    def test_from_value(self):
        assert NodeType("short_term") == NodeType.SHORT_TERM
        assert NodeType("long_term") == NodeType.LONG_TERM


class TestCausalEdge:
    def test_roundtrip(self):
        e = CausalEdge(from_node_id=1, to_node_id=5, relation=CausalRelation.ENABLES)
        e2 = CausalEdge.from_dict(e.to_dict())
        assert e2.from_node_id == 1
        assert e2.to_node_id == 5
        assert e2.relation == CausalRelation.ENABLES


class TestTagFilter:
    def test_empty(self):
        f = TagFilter()
        assert f.is_empty()

    def test_not_empty(self):
        f = TagFilter(scene_type="kitchen")
        assert not f.is_empty()

    def test_roundtrip(self):
        f = TagFilter(scene_type="kitchen", action_type="grasp", success=True)
        f2 = TagFilter.from_dict(f.to_dict())
        assert f2.scene_type == "kitchen"
        assert f2.action_type == "grasp"
        assert f2.success is True
        assert f2.task_type is None

    def test_none_fields_not_in_dict(self):
        f = TagFilter(scene_type="kitchen")
        d = f.to_dict()
        assert "action_type" not in d
        assert "task_type" not in d


class TestRememberRequest:
    def test_minimal(self):
        lr = LogRecord(ts=100, level="Info", tag="test", msg="hello")
        req = RememberRequest(session_id="s1", plan_id="p1", log_record=lr,
                              kv={"key": "val"})
        d = req.to_dict()
        req2 = RememberRequest.from_dict(d)
        assert req2.session_id == "s1"
        assert req2.log_record.msg == "hello"
        assert req2.kv == {"key": "val"}

    def test_with_spatial(self):
        lr = LogRecord(ts=200, level="Info", tag="t", msg="m")
        sp = SpatialContext(
            objects=[ObjectCoord(obj_id="o1", label="cup")],
            origin="fixture_frame",
        )
        req = RememberRequest(session_id="s2", plan_id="p2", log_record=lr,
                              spatial=sp, parent_node_id=10)
        req2 = RememberRequest.from_dict(req.to_dict())
        assert req2.spatial is not None
        assert req2.spatial.objects[0].label == "cup"
        assert req2.parent_node_id == 10


class TestSearchRequest:
    def test_minimal(self):
        req = SearchRequest(query="find cups")
        d = req.to_dict()
        req2 = SearchRequest.from_dict(d)
        assert req2.query == "find cups"
        assert req2.top_k == 5

    def test_full(self):
        tf = TagFilter(scene_type="kitchen")
        tr = TimeRange(start_ts=0, end_ts=1000)
        req = SearchRequest(query="find cups", tags=tf, top_k=10, alpha=0.5,
                            time_range=tr)
        req2 = SearchRequest.from_dict(req.to_dict())
        assert req2.tags is not None and req2.tags.scene_type == "kitchen"
        assert req2.alpha == 0.5
        assert req2.time_range is not None and req2.time_range.end_ts == 1000


class TestDifficultyLeq:
    def test_ordering(self):
        assert difficulty_leq("easy", "medium") is True
        assert difficulty_leq("easy", "hard") is True
        assert difficulty_leq("medium", "medium") is True
        assert difficulty_leq("hard", "medium") is False
        assert difficulty_leq("medium", "easy") is False


class TestForgetRisk:
    def test_defaults(self):
        fr = ForgetRisk(node_id=1)
        assert fr.node_id == 1
        assert fr.total_risk == 0.0
        assert fr.is_protected is False


class TestEnums:
    def test_causal_relation_values(self):
        assert CausalRelation.ENABLES.value == "enables"
        assert CausalRelation.TRIGGERS.value == "triggers"

    def test_forget_decision_values(self):
        assert ForgetDecision.RETAIN.value == "retain"
        assert ForgetDecision.ARCHIVE.value == "archive"


# ── Runner ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import traceback
    tests = [
        TestObjectCoord(), TestSpatialContext(), TestTagSet(), TestLogRecord(),
        TestMemoryNode(), TestNodeType(), TestCausalEdge(), TestTagFilter(),
        TestRememberRequest(), TestSearchRequest(), TestDifficultyLeq(),
        TestForgetRisk(), TestEnums(),
    ]
    passed = failed = 0
    for obj in tests:
        cls_name = type(obj).__name__
        for name in dir(obj):
            if name.startswith("test_"):
                try:
                    getattr(obj, name)()
                    print(f"  PASS {cls_name}.{name}")
                    passed += 1
                except Exception:
                    print(f"  FAIL {cls_name}.{name}")
                    traceback.print_exc()
                    failed += 1
    print(f"\n{passed} passed, {failed} failed")
    sys.exit(1 if failed else 0)
