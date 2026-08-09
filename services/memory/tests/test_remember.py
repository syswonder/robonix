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
        ], origin="fixture_frame")
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
        ], origin="fixture_frame")
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


# ── Runner ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    import logging, traceback
    logging.disable(logging.CRITICAL)  # suppress TextEmbedder warnings

    tests = [TestTagExtraction(), TestSummaryGeneration(), TestRememberPipeline()]
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
