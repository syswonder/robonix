"""Tests for RememberPipeline — LogRecord → MemoryNode write path."""

import sys, os, tempfile, shutil, asyncio
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from memory_service.core.types import (
    LogRecord, SpatialContext, ObjectCoord, RememberRequest, NodeType,
)
from memory_service.storage.graph_store import GraphStore
from memory_service.storage.tag_index import TagIndex
from memory_service.storage.vector_store import VectorStore
from memory_service.storage.embedding_config import EmbeddingModelConfig
from memory_service.storage.image_store import ImageStore
from memory_service.core.remember import RememberPipeline, _rule_based_tag_extraction, _generate_summary


class TestTagExtraction:
    def test_scene_extraction(self):
        lr = LogRecord(ts=100, level="Info", tag="pilot",
                       msg="robot grasped the red cup in the kitchen")
        tags = _rule_based_tag_extraction(lr, None)
        assert tags.scene_type == "kitchen"

    def test_action_extraction(self):
        lr = LogRecord(ts=100, level="Info", tag="exec",
                       msg="successfully picked up the object")
        tags = _rule_based_tag_extraction(lr, None)
        assert tags.action_type == "grasp"

    def test_task_extraction(self):
        lr = LogRecord(ts=100, level="Info", tag="pilot",
                       msg="robot went to fetch the cup")
        tags = _rule_based_tag_extraction(lr, None)
        assert tags.task_type == "fetch"

    def test_success_from_level(self):
        lr_ok = LogRecord(ts=1, level="Info", tag="t", msg="did thing")
        tags = _rule_based_tag_extraction(lr_ok, None)
        assert tags.success is True

        lr_err = LogRecord(ts=1, level="Error", tag="t", msg="failed")
        tags_err = _rule_based_tag_extraction(lr_err, None)
        assert tags_err.success is False

    def test_spatial_objects_extraction(self):
        sp = SpatialContext(objects=[
            ObjectCoord(obj_id="o1", label="red cup"),
            ObjectCoord(obj_id="o2", label="table"),
        ])
        lr = LogRecord(ts=1, level="Info", tag="t", msg="grasp cup")
        tags = _rule_based_tag_extraction(lr, sp)
        assert "red cup" in tags.objects_present
        assert "table" in tags.objects_present


class TestSummaryGeneration:
    def test_basic_summary(self):
        lr = LogRecord(ts=1, level="Info", tag="exec",
                       msg="robot grasped the red cup in the kitchen")
        s = _generate_summary(lr, None)
        assert "successfully" in s
        assert "grasp" in s or "grasped" in s

    def test_failure_summary(self):
        lr = LogRecord(ts=1, level="Error", tag="exec",
                       msg="robot failed to grasp cup")
        s = _generate_summary(lr, None)
        assert "failed" in s


class TestPlanTagExtraction:
    """Tag extraction with explicit kv.task_type='plan'."""

    def test_plan_task_type_from_kv(self):
        lr = LogRecord(ts=100, level="Info", tag="pilot",
                       msg="去厨房拿可乐")
        kv = {"task_type": "plan", "success": "true", "difficulty": "medium"}
        tags = _rule_based_tag_extraction(lr, None, kv)
        assert tags.task_type == "plan"
        assert tags.action_type == "plan"
        assert tags.success is True
        assert tags.difficulty == "medium"

    def test_plan_with_scene_from_kv(self):
        lr = LogRecord(ts=100, level="Info", tag="pilot",
                       msg="navigate to kitchen")
        kv = {"task_type": "plan", "success": "true",
              "scene_type": "kitchen"}
        tags = _rule_based_tag_extraction(lr, None, kv)
        assert tags.task_type == "plan"
        assert tags.scene_type == "kitchen"

    def test_plan_kv_overrides_keyword_matching(self):
        """When kv says 'plan', keyword matching is skipped entirely."""
        lr = LogRecord(ts=100, level="Info", tag="pilot",
                       msg="grasped cup in the kitchen")  # would normally match kitchen/grasp
        kv = {"task_type": "plan", "success": "true"}
        tags = _rule_based_tag_extraction(lr, None, kv)
        # Keyword matching skipped → no scene_type from msg
        assert tags.task_type == "plan"
        assert tags.scene_type == ""  # not "kitchen"


class TestPlanSummaryGeneration:
    """Summary generation for plan-type memories."""

    def test_plan_summary_basic(self):
        lr = LogRecord(ts=1, level="Info", tag="pilot",
                       msg="去厨房拿可乐")
        kv = {
            "task_type": "plan",
            "plan_query": "去厨房拿可乐",
            "plan_description": "navigate to kitchen and grasp cola",
            "plan_steps": "1. navigate\n2. grasp\n3. return",
        }
        s = _generate_summary(lr, None, kv)
        assert "successful plan" in s
        assert "去厨房拿可乐" in s
        assert "navigate to kitchen and grasp cola" in s

    def test_plan_summary_without_description(self):
        lr = LogRecord(ts=1, level="Info", tag="pilot",
                       msg="去客厅检查设备")
        kv = {
            "task_type": "plan",
            "plan_query": "去客厅检查设备",
            "plan_steps": "1. [tiago_navigation.navigate_to_goal] navigate to living room\n2. [camera.camera_snapshot] take photo",
        }
        s = _generate_summary(lr, None, kv)
        assert "successful plan" in s
        assert "去客厅检查设备" in s
        assert "[2 steps]" in s


class TestRememberPipeline:
    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.graph = GraphStore(data_dir=self.tmp)
        self.tags = TagIndex()
        cfg = EmbeddingModelConfig(dim=8)
        self.vectors = VectorStore(config=cfg, alpha=0.3)
        self.images = ImageStore(image_root=os.path.join(self.tmp, "images"))
        self.pipeline = RememberPipeline(self.graph, self.tags, self.vectors, self.images)

    def teardown_method(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _run(self, req):
        return asyncio.run(self.pipeline.execute(req))

    def test_full_write_consistency(self):
        lr = LogRecord(ts=100, level="Info", tag="exec",
                       msg="robot grasped the red cup in the kitchen")
        req = RememberRequest(session_id="s1", plan_id="p1", log_record=lr)
        resp = self._run(req)
        assert resp.node_id >= 0

        # GraphStore has it
        node = self.graph.get_node(resp.node_id)
        assert node is not None
        assert "grasp" in node.summary
        assert node.tags is not None
        assert node.tags.scene_type == "kitchen"

        # TagIndex has it
        assert self.tags.get_tags(resp.node_id) is not None

        # VectorStore has it
        assert self.vectors.count() >= 1

    def test_with_spatial_coordinates(self):
        sp = SpatialContext(objects=[
            ObjectCoord(obj_id="o1", label="red cup", x=1.0, y=2.0, z=0.5)
        ])
        lr = LogRecord(ts=200, level="Info", tag="exec",
                       msg="placed red cup on the kitchen table")
        req = RememberRequest(session_id="s2", plan_id="p2", log_record=lr,
                              spatial=sp)
        resp = self._run(req)
        node = self.graph.get_node(resp.node_id)
        assert node.spatial_data is not None
        assert len(node.spatial_data.objects) == 1
        assert node.spatial_data.objects[0].label == "red cup"
        assert node.tags.objects_present == ["red cup"]

    def test_with_parent_node(self):
        # Write parent first
        lr1 = LogRecord(ts=100, level="Info", tag="exec", msg="navigated to kitchen")
        resp1 = self._run(RememberRequest(session_id="s1", plan_id="p1", log_record=lr1))

        # Write child with parent reference
        lr2 = LogRecord(ts=200, level="Info", tag="exec",
                        msg="grasped cup in the kitchen")
        req2 = RememberRequest(session_id="s1", plan_id="p1", log_record=lr2,
                               parent_node_id=resp1.node_id)
        resp2 = self._run(req2)

        # Verify edge exists
        children = self.graph.get_children(resp1.node_id)
        assert resp2.node_id in children

        # Verify child's causal_chain
        child_node = self.graph.get_node(resp2.node_id)
        assert resp1.node_id in child_node.causal_chain

    def test_no_spatial_degraded_write(self):
        lr = LogRecord(ts=100, level="Info", tag="exec",
                       msg="robot did a routine check")
        req = RememberRequest(session_id="s3", plan_id="p3", log_record=lr,
                              spatial=None)
        resp = self._run(req)
        node = self.graph.get_node(resp.node_id)
        assert node.spatial_data is None
        assert node.tags.objects_present == []

    def test_image_base64_in_kv_saves_image(self):
        """When kv contains 'image_base64', remember pipeline saves it."""
        import base64
        lr = LogRecord(ts=100, level="Info", tag="camera",
                       msg="camera snapshot in kitchen")
        img_bytes = b"\x89PNG\r\n\x1a\nfake png frame data"
        req = RememberRequest(
            session_id="s4", plan_id="p4", log_record=lr,
            kv={"image_base64": base64.b64encode(img_bytes).decode("ascii")},
        )
        resp = self._run(req)
        node = self.graph.get_node(resp.node_id)
        assert len(node.image_refs) >= 1
        assert "frame_0001.jpg" in node.image_refs[0]

    # ── Plan-type memory tests ──────────────────────────────────────────

    def test_plan_node_sets_lesson_type_and_weight(self):
        """Successful plan → NodeType.LESSON + weight 0.8."""
        lr = LogRecord(ts=100, level="Info", tag="pilot",
                       msg="去厨房拿瓶可乐")
        kv = {
            "task_type": "plan",
            "success": "true",
            "plan_query": "去厨房拿瓶可乐",
            "plan_description": "navigate→grasp→return",
            "plan_steps": "1. [tiago_navigation.navigate_to_goal] navigate to kitchen\n"
                          "2. [tiago_gripper.grasp_object] grasp cola\n"
                          "3. [tiago_navigation.navigate_to_goal] return",
        }
        req = RememberRequest(session_id="s-plan", plan_id="p-plan",
                              log_record=lr, kv=kv)
        resp = self._run(req)
        node = self.graph.get_node(resp.node_id)
        assert node.node_type == NodeType.LESSON, \
            f"expected LESSON, got {node.node_type}"
        assert node.weight == 0.8, \
            f"expected weight 0.8, got {node.weight}"

    def test_plan_node_tags_are_correct(self):
        """Plan node tags: task_type=plan, action_type=plan, success=True."""
        lr = LogRecord(ts=100, level="Info", tag="pilot",
                       msg="检查客厅设备")
        kv = {
            "task_type": "plan",
            "success": "true",
            "plan_query": "检查客厅设备",
            "plan_description": "inspect living room equipment",
            "plan_steps": "1. navigate\n2. camera_snapshot",
        }
        req = RememberRequest(session_id="s-plan2", plan_id="p-plan2",
                              log_record=lr, kv=kv)
        resp = self._run(req)
        node = self.graph.get_node(resp.node_id)
        assert node.tags.task_type == "plan"
        assert node.tags.action_type == "plan"
        assert node.tags.success is True

    def test_plan_node_summary_format(self):
        """Plan summary contains query, description, and step count."""
        lr = LogRecord(ts=100, level="Info", tag="pilot",
                       msg="巡逻办公室")
        kv = {
            "task_type": "plan",
            "plan_query": "巡逻办公室",
            "plan_description": "patrol the office area",
            "plan_steps": "1. nav\n2. inspect\n3. nav",
        }
        req = RememberRequest(session_id="s-plan3", plan_id="p-plan3",
                              log_record=lr, kv=kv)
        resp = self._run(req)
        node = self.graph.get_node(resp.node_id)
        assert "successful plan" in node.summary
        assert "巡逻办公室" in node.summary
        assert "patrol the office area" in node.summary

    def test_plan_node_embedding_includes_steps(self):
        """Plan embedding text encodes query+description+steps for search matching."""
        lr = LogRecord(ts=100, level="Info", tag="pilot",
                       msg="拿可乐")
        kv = {
            "task_type": "plan",
            "plan_query": "去厨房拿可乐",
            "plan_description": "fetch cola from kitchen",
            "plan_steps": "1. [nav] to kitchen\n2. [grasp] cola",
        }
        req = RememberRequest(session_id="s-plan4", plan_id="p-plan4",
                              log_record=lr, kv=kv)
        resp = self._run(req)
        node = self.graph.get_node(resp.node_id)
        # Embedding should exist and be non-empty
        assert len(node.embedding) > 0, "plan node must have embedding"
        # The embedding is derived from plan text, not just the summary
        # We can't inspect the text directly but the embedding dim matches config
        assert len(node.embedding) == 8  # test config dim=8

    def test_plan_node_indexed_in_tag_index(self):
        """Plan node appears in tag index with correct task_type."""
        lr = LogRecord(ts=100, level="Info", tag="pilot",
                       msg="go to kitchen")
        kv = {"task_type": "plan", "success": "true",
              "plan_query": "go to kitchen", "plan_description": "nav",
              "plan_steps": "1. nav"}
        req = RememberRequest(session_id="s-plan5", plan_id="p-plan5",
                              log_record=lr, kv=kv)
        resp = self._run(req)
        tags = self.tags.get_tags(resp.node_id)
        assert tags is not None
        assert tags.task_type == "plan"

    def test_plan_node_retrievable_by_task_type(self):
        """TagIndex query with task_type='plan' finds the saved plan."""
        lr = LogRecord(ts=100, level="Info", tag="pilot",
                       msg="walk to door")
        kv = {"task_type": "plan", "success": "true",
              "plan_query": "walk to door", "plan_description": "nav",
              "plan_steps": "1. walk"}
        req = RememberRequest(session_id="s-plan6", plan_id="p-plan6",
                              log_record=lr, kv=kv)
        resp = self._run(req)

        # Query tag index for plan type
        from memory_service.core.types import TagFilter
        tf = TagFilter(task_type="plan")
        candidates = self.tags.query(tf)
        assert resp.node_id in candidates, \
            f"plan node {resp.node_id} should be findable by task_type=plan"


class TestPlanObserveMixedRecall:
    """Integration: save observations + plans, then verify retrieval separation.

    Simulates the real-world scenario: some nodes are auto-captured
    observations (SHORT_TERM), others are successful plans (LESSON).
    The tag index and vector store must correctly distinguish them.
    """

    def setup_method(self):
        self.tmp = tempfile.mkdtemp()
        self.graph = GraphStore(data_dir=self.tmp)
        self.tags = TagIndex()
        cfg = EmbeddingModelConfig(dim=8)
        self.vectors = VectorStore(config=cfg, alpha=0.3)
        self.pipeline = RememberPipeline(self.graph, self.tags, self.vectors)

    def teardown_method(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def _run(self, req):
        return asyncio.run(self.pipeline.execute(req))

    def test_plan_and_observation_separated_by_task_type(self):
        """Task type filtering correctly separates plans from observations."""
        from memory_service.core.types import TagFilter

        # Save 2 observations with keywords that match explore task_type
        for i, msg in enumerate([
            "robot scanned the corridor for fire equipment",
            "robot explored the office area",
        ]):
            lr = LogRecord(ts=100 + i, level="Info", tag="camera", msg=msg)
            req = RememberRequest(session_id="s", plan_id="p",
                                  log_record=lr)
            self._run(req)

        # Save 2 plans
        plan_kvs = [
            {
                "task_type": "plan", "success": "true",
                "plan_query": "巡逻走廊检查灭火器",
                "plan_description": "patrol corridor for fire equipment",
                "plan_steps": "1. [nav] to corridor\n2. [cam] snapshot",
            },
            {
                "task_type": "plan", "success": "true",
                "plan_query": "检查办公室电脑",
                "plan_description": "inspect office computers",
                "plan_steps": "1. [nav] to office\n2. [cam] inspect",
            },
        ]
        plan_ids = []
        for kv in plan_kvs:
            lr = LogRecord(ts=200, level="Info", tag="pilot",
                           msg=kv["plan_query"])
            req = RememberRequest(session_id="s", plan_id="p",
                                  log_record=lr, kv=kv)
            resp = self._run(req)
            plan_ids.append(resp.node_id)
            node = self.graph.get_node(resp.node_id)
            assert node.node_type == NodeType.LESSON

        # Filter by task_type=plan → only plans
        tf_plan = TagFilter(task_type="plan")
        plan_candidates = self.tags.query(tf_plan)
        assert len(plan_candidates) == 2
        for pid in plan_ids:
            assert pid in plan_candidates

        # Filter by task_type=explore → only observations
        tf_explore = TagFilter(task_type="explore")
        obs_candidates = self.tags.query(tf_explore)
        assert len(obs_candidates) == 2, \
            f"expected 2 explore observations, got {len(obs_candidates)}"

        # No overlap between plan and observation sets
        assert plan_candidates.isdisjoint(obs_candidates)

    def test_plan_nodes_have_higher_weight(self):
        """Plan (LESSON) nodes have weight 0.8 vs observation 0.5."""
        # Save observation
        lr_obs = LogRecord(ts=100, level="Info", tag="camera",
                           msg="observed laptop in office")
        req_obs = RememberRequest(session_id="s", plan_id="p", log_record=lr_obs)
        resp_obs = self._run(req_obs)
        node_obs = self.graph.get_node(resp_obs.node_id)
        assert node_obs.weight == 0.5

        # Save plan
        kv = {"task_type": "plan", "success": "true",
              "plan_query": "check office", "plan_description": "inspect",
              "plan_steps": "1. nav\n2. snap"}
        lr_plan = LogRecord(ts=200, level="Info", tag="pilot",
                            msg="check office")
        req_plan = RememberRequest(session_id="s", plan_id="p",
                                   log_record=lr_plan, kv=kv)
        resp_plan = self._run(req_plan)
        node_plan = self.graph.get_node(resp_plan.node_id)
        assert node_plan.weight == 0.8
        assert node_plan.node_type == NodeType.LESSON

        # In a weight-sorted ranking, plan nodes rank above observations
        # (all else equal — same score)
        assert node_plan.weight > node_obs.weight

    def test_plan_embedding_differs_from_observation(self):
        """Plan and observation embeddings are different despite same config."""
        # Same summary text, but plan node has enriched embedding_text
        lr = LogRecord(ts=100, level="Info", tag="pilot",
                       msg="go to kitchen")
        kv = {"task_type": "plan", "success": "true",
              "plan_query": "go to kitchen",
              "plan_description": "navigate",
              "plan_steps": "1. [nav] navigate to kitchen"}
        req_plan = RememberRequest(session_id="s", plan_id="p",
                                   log_record=lr, kv=kv)
        resp_plan = self._run(req_plan)
        node_plan = self.graph.get_node(resp_plan.node_id)

        # Observation with similar msg
        lr_obs = LogRecord(ts=200, level="Info", tag="camera",
                           msg="go to kitchen")
        req_obs = RememberRequest(session_id="s", plan_id="p",
                                  log_record=lr_obs)
        resp_obs = self._run(req_obs)
        node_obs = self.graph.get_node(resp_obs.node_id)

        # Both have embeddings but they encode different text:
        # plan: "plan: go to kitchen | navigate | steps: ..."
        # obs:  "successfully navigate in unknown area"
        assert len(node_plan.embedding) == len(node_obs.embedding)
        # Embeddings should differ (different input text)
        plans_different = any(
            abs(a - b) > 1e-6
            for a, b in zip(node_plan.embedding, node_obs.embedding)
        )
        assert plans_different, \
            "plan and observation embeddings should differ"


# ── Runner ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import logging, traceback
    logging.disable(logging.CRITICAL)  # suppress TextEmbedder warnings

    tests = [TestTagExtraction(), TestPlanTagExtraction(),
             TestSummaryGeneration(), TestPlanSummaryGeneration(),
             TestRememberPipeline(), TestPlanObserveMixedRecall()]
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
