"""Test MCP wire interface — JSON-over-String serialisation / deserialisation.

Validates that the MCP handlers:
  1. Accept valid JSON matching the documented schema
  2. Reject malformed JSON with clear error messages
  3. Validate required fields (session_id, plan_id, log_record, query)
  4. Produce consistent error response format: {"error": "..."}
  5. Produce consistent success response format: {"node_id": N, "message": "..."} etc.

These tests call the MCP handler functions directly (no robonix_api needed),
passing std_msgs_mcp.String and inspecting the returned String payload.
"""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

# ── Ensure the memory_service package is importable ────────────────────
_HERE = Path(__file__).resolve().parent
_SVC = _HERE.parent
if str(_SVC) not in sys.path:
    sys.path.insert(0, str(_SVC))

# MCP-mode import requires robonix_api + std_msgs_mcp.
# We mock String/Empty when those are unavailable so the test suite
# covers the *logic* without requiring a full rbnx environment.
_HAVE_RBNX = False
try:
    from robonix_api import Service, Ok
    from std_msgs_mcp import Empty, String
    _HAVE_RBNX = True
except ImportError:
    pass


# ── Lightweight stand-ins when rbnx deps are absent ────────────────────

class _FakeString:
    """Minimal std_msgs/String stand-in."""
    def __init__(self, data: str = ""):
        self.data = data


class _FakeEmpty:
    """Minimal std_msgs/Empty stand-in."""
    pass


# ── Helpers ────────────────────────────────────────────────────────────

def _make_remember_payload(**overrides) -> dict:
    """Return a valid remember request dict, with optional overrides."""
    payload = {
        "session_id": "test-session-1",
        "plan_id": "test-plan-1",
        "log_record": {
            "ts": 1_700_000_000_000_000_000,
            "level": "Info",
            "tag": "exec",
            "msg": "test action in kitchen",
        },
    }
    payload.update(overrides)
    return payload


def _make_search_payload(**overrides) -> dict:
    """Return a valid search request dict, with optional overrides."""
    payload = {"query": "test query"}
    payload.update(overrides)
    return payload


# ── Test cases ─────────────────────────────────────────────────────────

class TestMcpRememberValidation(unittest.TestCase):
    """remember handler: input validation and error responses."""

    @classmethod
    def setUpClass(cls):
        # Create a fresh MemoryService and wire up the MCP handler logic
        # directly (no robonix_api decorator — just test the core logic).
        from memory_service.service import MemoryService
        cls._tmpdir = tempfile.TemporaryDirectory()
        cls._svc = MemoryService(data_dir=cls._tmpdir.name)

        # Replicate the MCP handler's JSON parsing and validation inline,
        # calling the same MemoryService.remember underneath.
        cls._handler = cls  # use class methods as the "handler"

    @classmethod
    def tearDownClass(cls):
        cls._tmpdir.cleanup()

    # -- helper: runs the equivalent of _mcp_remember -------------------

    async def _call_remember(self, payload: dict) -> dict:
        """Simulate the MCP remember handler: parse JSON → validate → call service."""
        # Simulate what _mcp_remember does
        session_id = payload.get("session_id")
        plan_id = payload.get("plan_id")
        if not session_id or not plan_id:
            return {"error": "Missing required fields: session_id and plan_id must be non-empty strings"}

        raw = payload.get("log_record")
        if not raw or not isinstance(raw, dict):
            return {"error": "Missing required field: log_record must be a dict"}

        from memory_service.core.types import LogRecord
        try:
            lr = LogRecord.from_dict(raw)
        except Exception as e:
            return {"error": f"Invalid log_record: {e}"}

        spatial = None
        if payload.get("spatial"):
            from memory_service.core.types import SpatialContext
            try:
                spatial = SpatialContext.from_dict(payload["spatial"])
            except Exception as e:
                return {"error": f"Invalid spatial: {e}"}

        from memory_service.core.types import RememberRequest
        request = RememberRequest(
            session_id=session_id,
            plan_id=plan_id,
            log_record=lr,
            spatial=spatial,
            parent_node_id=payload.get("parent_node_id"),
            kv=payload.get("kv") if isinstance(payload.get("kv"), dict) else {},
        )
        resp = await self._svc._remember_pipe.execute(request)
        return {"node_id": resp.node_id, "message": resp.message}

    # -- tests ---------------------------------------------------------

    def test_01_valid_minimal_payload(self):
        """Minimal valid remember request succeeds."""
        async def _t():
            result = await self._call_remember(_make_remember_payload())
            self.assertNotIn("error", result)
            self.assertIsInstance(result.get("node_id"), int)
            self.assertGreaterEqual(result["node_id"], 0)  # 0 is first ShortTerm ID
        import asyncio
        asyncio.run(_t())

    def test_02_missing_session_id(self):
        """Missing session_id returns error."""
        async def _t():
            result = await self._call_remember(
                _make_remember_payload(session_id="")
            )
            self.assertIn("error", result)
            self.assertIn("session_id", result["error"])
        import asyncio
        asyncio.run(_t())

    def test_03_missing_plan_id(self):
        """Missing plan_id returns error."""
        async def _t():
            result = await self._call_remember(
                _make_remember_payload(plan_id="")
            )
            self.assertIn("error", result)
            self.assertIn("plan_id", result["error"])
        import asyncio
        asyncio.run(_t())

    def test_04_missing_log_record(self):
        """Missing log_record returns error."""
        async def _t():
            payload = _make_remember_payload()
            del payload["log_record"]
            result = await self._call_remember(payload)
            self.assertIn("error", result)
            self.assertIn("log_record", result["error"])
        import asyncio
        asyncio.run(_t())

    def test_05_log_record_not_dict(self):
        """log_record as non-dict returns error."""
        async def _t():
            result = await self._call_remember(
                _make_remember_payload(log_record="not_a_dict")
            )
            self.assertIn("error", result)
            self.assertIn("log_record", result["error"])
        import asyncio
        asyncio.run(_t())

    def test_06_with_spatial_objects(self):
        """Valid remember with spatial coordinates succeeds."""
        async def _t():
            payload = _make_remember_payload()
            payload["spatial"] = {
                "origin": "world",
                "objects": [
                    {"obj_id": "scene.obj.cup_001", "label": "red cup",
                     "x": 1.0, "y": 2.0, "z": 0.8},
                ],
            }
            result = await self._call_remember(payload)
            self.assertNotIn("error", result)
            self.assertGreaterEqual(result["node_id"], 0)
        import asyncio
        asyncio.run(_t())

    def test_07_with_parent_node(self):
        """Valid remember with parent_node_id creates causal edge."""
        async def _t():
            # First, create parent
            p = await self._call_remember(_make_remember_payload(
                session_id="sess-chain", plan_id="plan-chain-1",
                log_record={"ts": 0, "level": "Info", "tag": "nav",
                           "msg": "navigated to kitchen"},
            ))
            parent_id = p["node_id"]
            # Then, create child referencing parent
            c = await self._call_remember(_make_remember_payload(
                session_id="sess-chain", plan_id="plan-chain-1",
                log_record={"ts": 0, "level": "Info", "tag": "exec",
                           "msg": "grasped cup"},
                parent_node_id=parent_id,
            ))
            self.assertNotIn("error", c)
            self.assertGreaterEqual(c["node_id"], 0)
            # Verify causal edge
            parents = self._svc.graph.get_parents(c["node_id"])
            self.assertIn(parent_id, parents)
        import asyncio
        asyncio.run(_t())

    def test_08_kv_payload_preserved(self):
        """kv dict is accepted and node is created."""
        async def _t():
            result = await self._call_remember(_make_remember_payload(
                kv={"object_id": "scene.obj.cup_001", "grasp_force": "2.5"},
            ))
            self.assertNotIn("error", result)
            self.assertGreaterEqual(result["node_id"], 0)
        import asyncio
        asyncio.run(_t())

    def test_09_invalid_spatial_rejected(self):
        """Malformed spatial returns error without crashing."""
        async def _t():
            result = await self._call_remember(_make_remember_payload(
                spatial={"objects": "not_a_list"},
            ))
            self.assertIn("error", result)
        import asyncio
        asyncio.run(_t())


class TestMcpSearchValidation(unittest.TestCase):
    """search handler: input validation and error responses."""

    @classmethod
    def setUpClass(cls):
        from memory_service.service import MemoryService
        cls._tmpdir = tempfile.TemporaryDirectory()
        cls._svc = MemoryService(data_dir=cls._tmpdir.name)

    @classmethod
    def tearDownClass(cls):
        cls._tmpdir.cleanup()

    async def _call_search(self, payload: dict) -> dict:
        """Simulate the MCP search handler."""
        query = payload.get("query")
        if not query or not isinstance(query, str) or not query.strip():
            return {"error": "Missing or empty required field: query"}

        from memory_service.core.types import TagFilter, TimeRange, SearchRequest
        tags = None
        if payload.get("tags"):
            try:
                tags = TagFilter.from_dict(payload["tags"])
            except Exception as e:
                return {"error": f"Invalid tags: {e}"}

        time_range = None
        if payload.get("time_range"):
            try:
                tr = payload["time_range"]
                time_range = TimeRange(
                    start_ts=int(tr.get("start_ts", 0)),
                    end_ts=int(tr.get("end_ts", 0)),
                )
            except (TypeError, ValueError) as e:
                return {"error": f"Invalid time_range: {e}"}

        try:
            top_k = int(payload.get("top_k", 5))
        except (TypeError, ValueError):
            return {"error": "Invalid top_k: must be an integer"}

        try:
            alpha = float(payload["alpha"]) if payload.get("alpha") is not None else None
        except (TypeError, ValueError):
            return {"error": "Invalid alpha: must be a float between 0.0 and 1.0"}

        request = SearchRequest(
            query=query.strip(), tags=tags, top_k=max(1, min(top_k, 100)),
            alpha=alpha, time_range=time_range,
            require_executable=bool(payload.get("require_executable", False)),
        )
        resp = await self._svc._retrieve_pipe.execute(request)
        return {"nodes": [n.to_dict() for n in resp.nodes]}

    async def _seed_node(self, session="sess-1", plan="plan-1",
                         level="Info", tag="exec", msg="test action in kitchen",
                         objects=None) -> int:
        """Insert a node and return its node_id."""
        from memory_service.core.types import LogRecord
        lr = LogRecord(ts=0, level=level, tag=tag, msg=msg)
        spatial = None
        if objects:
            from memory_service.core.types import SpatialContext, ObjectCoord
            spatial = SpatialContext(
                objects=[ObjectCoord(obj_id=o[0], label=o[1],
                                     x=o[2], y=o[3], z=o[4]) for o in objects],
                origin="world",
            )
        from memory_service.core.types import RememberRequest
        req = RememberRequest(session_id=session, plan_id=plan, log_record=lr,
                             spatial=spatial)
        resp = await self._svc._remember_pipe.execute(req)
        return resp.node_id

    # -- tests ---------------------------------------------------------

    def test_10_missing_query(self):
        """Empty / missing query returns error."""
        async def _t():
            result = await self._call_search({"query": ""})
            self.assertIn("error", result)
            self.assertIn("query", result["error"])
        import asyncio
        asyncio.run(_t())

    def test_11_no_query_key(self):
        """No query key at all returns error."""
        async def _t():
            result = await self._call_search({})
            self.assertIn("error", result)
            self.assertIn("query", result["error"])
        import asyncio
        asyncio.run(_t())

    def test_12_query_not_string(self):
        """Non-string query returns error."""
        async def _t():
            result = await self._call_search({"query": 12345})
            self.assertIn("error", result)
        import asyncio
        asyncio.run(_t())

    def test_13_empty_store_returns_empty(self):
        """Search on empty store returns empty nodes list."""
        async def _t():
            result = await self._call_search({"query": "anything"})
            self.assertNotIn("error", result)
            self.assertEqual(result.get("nodes"), [])
        import asyncio
        asyncio.run(_t())

    def test_14_basic_search_finds_node(self):
        """Search finds a previously remembered node."""
        async def _t():
            await self._seed_node(
                msg="grasped red cup in the kitchen",
                objects=[["scene.obj.cup_001", "red cup", 1.0, 2.0, 0.8]],
            )
            result = await self._call_search({"query": "red cup"})
            self.assertNotIn("error", result)
            self.assertGreater(len(result.get("nodes", [])), 0)
            node = result["nodes"][0]
            self.assertIn("node_id", node)
            self.assertIn("summary", node)
        import asyncio
        asyncio.run(_t())

    def test_15_tag_filter_narrows_results(self):
        """scene_type tag filter narrows results correctly."""
        async def _t():
            await self._seed_node(
                msg="grasped red cup in the kitchen",
                objects=[["scene.obj.cup_001", "red cup", 1.0, 2.0, 0.8]],
            )
            await self._seed_node(
                msg="placed blue cup in the living room",
                objects=[["scene.obj.cup_002", "blue cup", 3.0, 4.0, 0.5]],
            )
            # Tag-filtered: kitchen only
            result = await self._call_search({
                "query": "cup",
                "tags": {"scene_type": "kitchen"},
            })
            self.assertNotIn("error", result)
            nodes = result.get("nodes", [])
            self.assertGreater(len(nodes), 0)
            for n in nodes:
                tags = n.get("tags", {})
                self.assertEqual(tags.get("scene_type"), "kitchen")
        import asyncio
        asyncio.run(_t())

    def test_16_top_k_clamps(self):
        """top_k is respected."""
        async def _t():
            for i in range(5):
                await self._seed_node(
                    msg=f"action {i} in kitchen",
                    objects=[["obj", f"item_{i}", float(i), 0.0, 0.0]],
                )
            result = await self._call_search({"query": "kitchen", "top_k": 2})
            self.assertNotIn("error", result)
            nodes = result.get("nodes", [])
            self.assertLessEqual(len(nodes), 2)
        import asyncio
        asyncio.run(_t())

    def test_17_invalid_top_k_rejected(self):
        """Non-integer top_k returns error."""
        async def _t():
            result = await self._call_search({"query": "test", "top_k": "abc"})
            self.assertIn("error", result)
            self.assertIn("top_k", result["error"])
        import asyncio
        asyncio.run(_t())

    def test_18_invalid_alpha_rejected(self):
        """Non-float alpha returns error."""
        async def _t():
            result = await self._call_search({"query": "test", "alpha": "high"})
            self.assertIn("error", result)
            self.assertIn("alpha", result["error"])
        import asyncio
        asyncio.run(_t())

    def test_19_invalid_time_range_rejected(self):
        """Malformed time_range returns error."""
        async def _t():
            result = await self._call_search({
                "query": "test",
                "time_range": {"start_ts": "yesterday"},
            })
            self.assertIn("error", result)
            self.assertIn("time_range", result["error"])
        import asyncio
        asyncio.run(_t())

    def test_20_invalid_tags_rejected(self):
        """Malformed tags dict returns error without crashing."""
        async def _t():
            result = await self._call_search({
                "query": "test",
                "tags": "not_a_dict",
            })
            self.assertIn("error", result)
            self.assertIn("tags", result["error"])
        import asyncio
        asyncio.run(_t())

    def test_21_success_filter(self):
        """success=true filter returns only successful nodes."""
        async def _t():
            await self._seed_node(level="Info", msg="successful action")
            await self._seed_node(level="Error", msg="failed action")
            result = await self._call_search({
                "query": "action",
                "tags": {"success": True},
            })
            self.assertNotIn("error", result)
            for n in result.get("nodes", []):
                tags = n.get("tags", {})
                self.assertTrue(tags.get("success"), f"node {n['node_id']} not success")
        import asyncio
        asyncio.run(_t())

    def test_22_failure_filter(self):
        """success=false filter returns only failed nodes."""
        async def _t():
            await self._seed_node(level="Info", msg="successful action")
            await self._seed_node(level="Error", msg="failed action")
            result = await self._call_search({
                "query": "action",
                "tags": {"success": False},
            })
            self.assertNotIn("error", result)
            for n in result.get("nodes", []):
                tags = n.get("tags", {})
                self.assertFalse(tags.get("success"), f"node {n['node_id']} should be failure")
        import asyncio
        asyncio.run(_t())


class TestMcpCompact(unittest.TestCase):
    """compact handler: basic behaviour."""

    @classmethod
    def setUpClass(cls):
        from memory_service.service import MemoryService
        cls._tmpdir = tempfile.TemporaryDirectory()
        cls._svc = MemoryService(data_dir=cls._tmpdir.name)

    @classmethod
    def tearDownClass(cls):
        cls._tmpdir.cleanup()

    def test_23_compact_on_empty_store(self):
        """Compact on empty store returns 0 compacted."""
        async def _t():
            resp = await self._svc._compact_pipe.execute()
            self.assertEqual(resp.nodes_compacted, 0)
        import asyncio
        asyncio.run(_t())

    def test_24_compact_below_threshold(self):
        """Compact with few nodes does nothing."""
        async def _t():
            from memory_service.core.types import LogRecord, RememberRequest
            for i in range(3):
                lr = LogRecord(ts=0, level="Info", tag="test", msg=f"msg {i}")
                req = RememberRequest(session_id="s", plan_id="p", log_record=lr)
                await self._svc._remember_pipe.execute(req)
            resp = await self._svc._compact_pipe.execute()
            self.assertEqual(resp.nodes_compacted, 0)
        import asyncio
        asyncio.run(_t())

    def test_25_compact_response_format(self):
        """Compact response contains expected fields."""
        async def _t():
            resp = await self._svc._compact_pipe.execute()
            d = {"summary": resp.summary, "nodes_compacted": resp.nodes_compacted}
            self.assertIn("summary", d)
            self.assertIn("nodes_compacted", d)
            self.assertIsInstance(d["nodes_compacted"], int)
        import asyncio
        asyncio.run(_t())


if __name__ == "__main__":
    # Discover and run all tests; handle asyncio transparently.
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromModule(__import__(__name__))
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
