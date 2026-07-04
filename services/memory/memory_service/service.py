"""Scribe Mem service (memgraph) — structured CKG memory for Robonix.

Runs as a robonix-api Service with MCP tool registration.
Entrypoint for rbnx boot: `python -m memory_service.service`

MCP tools (registered under robonix/service/memgraph/):
  - robonix/service/memgraph/remember  — write structured MemoryNode
  - robonix/service/memgraph/search    — 3-stage hybrid retrieval
  - robonix/service/memgraph/compact   — promote short-term → long-term

Runs in parallel with memsearch (robonix/service/memory/*) — separate
namespace so Pilot can discover both sets of tools.

Phase1: JSON-over-std_msgs/String (avoids protobuf codegen changes).
Phase2: structured IDL when contract schema is frozen.
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import sys
import time
import traceback
from pathlib import Path

# ── 0. Scribe logging setup ──────────────────────────────────────────────
from .scribe_log import setup_scribe_logging  # noqa: E402

_LOG_LEVEL = os.environ.get("MEMORY_LOG_LEVEL", "INFO").upper()
_LOG_DIR = os.environ.get("SCRIBE_LOG_DIR",
                           os.path.join(os.path.dirname(os.path.dirname(__file__)), "logs"))

log = setup_scribe_logging(log_dir=_LOG_DIR, tag="service_memory", level=_LOG_LEVEL)

os.environ.setdefault("GRPC_VERBOSITY", "ERROR")
os.environ.setdefault("GLOG_minloglevel", "2")

log.info("scribe_mem: starting")


def _log_environment() -> None:
    log.info("python   : %s (%s)", sys.version.replace("\n", " "), sys.executable)
    log.info("cwd      : %s", os.getcwd())
    model_path = os.environ.get(
        "EMBEDDING_MODEL_PATH",
        os.path.join(os.path.expanduser("~"),
                     "EmbodyMemory", "memory", "all-MiniLM-L6-v2"),
    )
    log.info("embedding model path: %s (exists=%s)",
             model_path, os.path.isdir(model_path))


# ── 1. Core backend imports (no rbnx-api dependency) ───────────────────

from .core.types import (  # noqa: E402
    LogRecord, SpatialContext, ObjectCoord,
    TagFilter, TimeRange, RememberRequest, SearchRequest,
)
from .storage.graph_store import GraphStore  # noqa: E402
from .storage.tag_index import TagIndex  # noqa: E402
from .storage.vector_store import VectorStore  # noqa: E402
from .storage.embedding_config import EmbeddingModelConfig  # noqa: E402
from .core.remember import RememberPipeline  # noqa: E402
from .core.retrieve import RetrievePipeline  # noqa: E402
from .core.compact import CompactPipeline  # noqa: E402

# ── 2. Backend construction ─────────────────────────────────────────────

# Phase1: local directory anchored to the package root (this file is at
# memory_service/service.py, so parent.parent = services/memory/).
# Override via AGENT_MEMORY_DIR env var to point elsewhere.
_DEFAULT_MEMORY_DIR = str(Path(__file__).resolve().parent.parent / "memory")
MEMORY_DIR = str(
    Path(os.environ.get("AGENT_MEMORY_DIR", _DEFAULT_MEMORY_DIR))
    .resolve()
)

log.info("scribe_mem: graph_store → %s", os.path.join(MEMORY_DIR, "graph_store.json"))
log.info("scribe_mem: scribe_log  → %s", os.path.join(_LOG_DIR, "service_memory.log"))
log.info("scribe_mem: backend     → GraphStore + TagIndex + VectorStore(BM25+Embedding)")

_graph = GraphStore(data_dir=MEMORY_DIR)
_tags = TagIndex()
_vectors = VectorStore(alpha=0.3)
_remember_pipe = RememberPipeline(_graph, _tags, _vectors)
_retrieve_pipe = RetrievePipeline(_graph, _tags, _vectors)
_compact_pipe = CompactPipeline(_graph)

# ── 3. Standalone API (MemoryService) — works without rbnx ─────────────


class MemoryService:
    """Scribe Mem service — manages the full memory lifecycle.

    Works standalone (for scripts) and also used by the rbnx MCP entrypoint.
    When data_dir is provided, creates independent stores + pipelines.
    """

    def __init__(self, data_dir: str = "",
                 embedding_config=None, alpha: float = 0.3):
        if data_dir:
            self.graph = GraphStore(data_dir=data_dir)
            self.tags = TagIndex()
            ecfg = embedding_config or EmbeddingModelConfig()
            self.vectors = VectorStore(config=ecfg, alpha=alpha)
        else:
            self.graph = _graph
            self.tags = _tags
            self.vectors = _vectors

        # Always create own pipelines bound to own stores
        self._remember_pipe = RememberPipeline(self.graph, self.tags, self.vectors)
        self._retrieve_pipe = RetrievePipeline(self.graph, self.tags, self.vectors)
        self._compact_pipe = CompactPipeline(self.graph)

    @property
    def is_initialized(self) -> bool:
        return True

    async def init(self, _cfg=None) -> str:
        nodes = [self.graph.get_node(nid) for nid in self.graph.all_ids()]
        nodes = [n for n in nodes if n is not None]
        self.tags.rebuild(nodes)
        vec_triples = [(n.node_id, n.embedding, n.summary) for n in nodes]
        self.vectors.rebuild(vec_triples)
        log.info("init: rebuilt indices for %d nodes", len(nodes))
        return "ok"

    async def remember(self, session_id: str, plan_id: str,
                       log_record, spatial=None, parent_node_id=None,
                       kv=None):
        req = RememberRequest(
            session_id=session_id, plan_id=plan_id, log_record=log_record,
            spatial=spatial, parent_node_id=parent_node_id, kv=kv or {},
        )
        return await self._remember_pipe.execute(req)

    async def search(self, query: str, tags=None, top_k: int = 5,
                     alpha=None, time_range=None, require_executable: bool = False):
        req = SearchRequest(
            query=query, tags=tags, top_k=top_k, alpha=alpha,
            time_range=time_range, require_executable=require_executable,
        )
        return await self._retrieve_pipe.execute(req)

    async def compact(self):
        return await self._compact_pipe.execute()

    async def remember_from_log(self, session_id: str, plan_id: str,
                                level: str, tag: str, msg: str,
                                objects=None, parent_node_id=None):
        lr = LogRecord(level=level, tag=tag, msg=msg)
        spatial = None
        if objects:
            spatial = SpatialContext(
                objects=[ObjectCoord(obj_id=o[0], label=o[1],
                                     x=o[2] if len(o)>2 else 0,
                                     y=o[3] if len(o)>3 else 0,
                                     z=o[4] if len(o)>4 else 0)
                         for o in objects]
            )
        resp = await self.remember(session_id, plan_id, lr,
                                   spatial=spatial, parent_node_id=parent_node_id)
        return resp.node_id

    async def search_as_dicts(self, query: str, top_k: int = 5,
                              tag_filter=None):
        tf = TagFilter.from_dict(tag_filter) if tag_filter else None
        resp = await self.search(query, tags=tf, top_k=top_k)
        return [n.to_dict() for n in resp.nodes]


# ── 4. rbnx MCP entrypoint ──────────────────────────────────────────────
# Register MCP tools via robonix-api at module level so get_type_hints()
# can resolve String/Empty from globalns. Wrapped in try/except so the
# MemoryService class is still importable without robonix_api installed.

_MCP_AVAILABLE = False
try:
    from robonix_api import Service, Ok  # noqa: E402, F811
    from std_msgs_mcp import Empty, String  # noqa: E402
    _MCP_AVAILABLE = True
except ImportError:
    log.info("robonix_api not available — MCP server disabled "
             "(standalone/script mode OK)")


if _MCP_AVAILABLE:
    _memory_svc = Service(id="memgraph", namespace="robonix/service/memgraph")
    _indices_initialized = False

    # ── JSON response helpers ──────────────────────────────────────────

    def _json_ok(obj: dict) -> String:
        """Encode a success response dict as JSON-over-String."""
        return String(data=json.dumps(obj, ensure_ascii=False))

    def _json_error(message: str, *, ctx: str = "") -> String:
        """Encode an error response as JSON-over-String."""
        if ctx:
            log.warning("MCP %s error: %s", ctx, message)
        else:
            log.warning("MCP error: %s", message)
        return String(data=json.dumps({"error": message}))

    def _parse_json(data: str, *, ctx: str = "") -> dict | None:
        """Parse JSON from the wire, returning None on failure (error already emitted)."""
        try:
            return json.loads(data)  # type: ignore[no-any-return]
        except json.JSONDecodeError as e:
            log.warning("MCP %s: invalid JSON (%d chars): %s",
                        ctx, len(data), e)
            return None

    # ── MCP tool handlers ──────────────────────────────────────────────

    @_memory_svc.mcp("robonix/service/memgraph/remember")
    async def _mcp_remember(msg: String) -> String:
        """Write a structured memory node into the CKG (Causal Knowledge Graph).

        Use this to record a robot action, observation, or decision for
        later retrieval.  Each node is tagged across four dimensions
        (spatial / behavioural / cognitive / optimisation) and
        automatically embedded for hybrid BM25+vector search.

        Request JSON schema:
        {
          "session_id":    "<session uuid>",           // required
          "plan_id":       "<plan uuid>",              // required
          "log_record": {                              // required
            "ts":    1765432100123456789,  // ns timestamp (0 = now)
            "level": "Info",               // Debug|Info|Warn|Error
            "tag":   "exec",               // source provider_id
            "msg":   "grasped red cup in kitchen"
          },
          "spatial": {                               // optional
            "origin":  "world",
            "objects": [
              {"obj_id": "scene.obj.cup_001", "label": "red cup",
               "x": 1.0, "y": 2.0, "z": 0.8}
            ]
          },
          "parent_node_id": 5,           // optional — causal parent
          "kv": {}                        // optional — extra metadata
        }

        Response JSON: {"node_id": 42, "message": "Memory saved as node 42"}
        On error:       {"error": "<description>"}
        """
        req_dict = _parse_json(msg.data, ctx="remember")
        if req_dict is None:
            return _json_error("Invalid JSON in request body", ctx="remember")

        session_id = req_dict.get("session_id")
        plan_id = req_dict.get("plan_id")
        log.info("API remember: sid=%s pid=%s tag=%s msg=%r (%d chars)",
                 session_id, plan_id,
                 req_dict.get("log_record", {}).get("tag", "?"),
                 (req_dict.get("log_record", {}).get("msg", "") or "")[:60],
                 len(req_dict.get("log_record", {}).get("msg", "")))

        if not session_id or not plan_id:
            return _json_error(
                "Missing required fields: session_id and plan_id must be non-empty strings",
                ctx="remember",
            )

        raw = req_dict.get("log_record")
        if not raw or not isinstance(raw, dict):
            return _json_error("Missing required field: log_record must be a dict",
                              ctx="remember")

        try:
            lr = LogRecord.from_dict(raw)
        except Exception as e:
            return _json_error(f"Invalid log_record: {e}", ctx="remember")

        spatial = None
        if req_dict.get("spatial"):
            try:
                spatial = SpatialContext.from_dict(req_dict["spatial"])
            except Exception as e:
                return _json_error(f"Invalid spatial: {e}", ctx="remember")

        request = RememberRequest(
            session_id=session_id,
            plan_id=plan_id,
            log_record=lr,
            spatial=spatial,
            parent_node_id=req_dict.get("parent_node_id"),
            kv=req_dict.get("kv") if isinstance(req_dict.get("kv"), dict) else {},
        )
        resp = await _remember_pipe.execute(request)
        node = _graph.get_node(resp.node_id)
        summary = node.summary if node else "?"
        log.info("API remember: sid=%s pid=%s node_id=%d summary=%r",
                 request.session_id, request.plan_id, resp.node_id, summary)
        return _json_ok({"node_id": resp.node_id, "message": resp.message})

    @_memory_svc.mcp("robonix/service/memgraph/search")
    async def _mcp_search(msg: String) -> String:
        """Search the CKG memory using a 3-stage pipeline: tag filter → hybrid
        BM25+embedding ranking → causal/time/weight filter.

        Tags are used for O(1) inverted-index pre-filtering BEFORE vector
        ranking.  Always set as many tag fields as the user's intent
        implies — an empty TagFilter searches ALL nodes, which is slow
        and noisy.

        Request JSON schema:
        {
          "query":    "red cup in kitchen",          // required — semantic query
          "top_k":    5,                              // default 5
          "alpha":    0.3,                            // optional — BM25 weight (1.0=pure keyword, 0.0=pure semantic)
          "tags": {                                    // optional — all fields optional (AND semantics)
            "scene_type":     "kitchen",               // kitchen|living_room|workshop|bedroom|outdoor
            "objects":        ["red cup"],             // intersection match
            "action_type":    "grasp",                 // grasp|place|navigate|craft|observe
            "success":        true,                    // true = only successes, false = only failures
            "task_type":      "fetch",                 // fetch|build|explore|dialogue
            "difficulty_max": "medium"                 // easy|medium|hard — filter ≤ this level
          },
          "time_range": {                              // optional — ns timestamps
            "start_ts": 1765432100000000000,
            "end_ts":   1765432200000000000             // 0 = no upper bound
          },
          "require_executable": false                   // default false (Phase2)
        }

        Response JSON:
        {
          "nodes": [
            {
              "node_id": 42,
              "summary": "successfully grasp red cup in kitchen",
              "tags": {"scene_type": "kitchen", "action_type": "grasp", ...},
              "spatial_data": {"objects": [...], "origin": "world"},
              "weight": 0.5,
              "timestamp": 1765432100123456789,
              "raw_log": {"ts": ..., "level": "Info", "tag": "exec", "msg": "..."},
              "causal_chain": [...],
              "embedding": [...]
            }
          ]
        }
        On error: {"error": "<description>", "nodes": []}
        """
        req_dict = _parse_json(msg.data, ctx="search")
        if req_dict is None:
            return _json_error("Invalid JSON in request body", ctx="search")

        query = req_dict.get("query")
        log.info("API search: query=%r tags=%s top_k=%s alpha=%s time_range=%s",
                 (query or "")[:80],
                 "set" if req_dict.get("tags") else "none",
                 req_dict.get("top_k", "5"),
                 req_dict.get("alpha", "default"),
                 "set" if req_dict.get("time_range") else "none")

        if not query or not isinstance(query, str) or not query.strip():
            return _json_error("Missing or empty required field: query", ctx="search")

        tags = None
        if req_dict.get("tags"):
            try:
                tags = TagFilter.from_dict(req_dict["tags"])
            except Exception as e:
                return _json_error(f"Invalid tags: {e}", ctx="search")

        time_range = None
        if req_dict.get("time_range"):
            try:
                tr = req_dict["time_range"]
                time_range = TimeRange(
                    start_ts=int(tr.get("start_ts", 0)),
                    end_ts=int(tr.get("end_ts", 0)),
                )
            except (TypeError, ValueError) as e:
                return _json_error(f"Invalid time_range: {e}", ctx="search")

        try:
            top_k = int(req_dict.get("top_k", 5))
        except (TypeError, ValueError):
            return _json_error("Invalid top_k: must be an integer", ctx="search")

        try:
            alpha = float(req_dict["alpha"]) if req_dict.get("alpha") is not None else None
        except (TypeError, ValueError):
            return _json_error("Invalid alpha: must be a float between 0.0 and 1.0",
                              ctx="search")

        request = SearchRequest(
            query=query.strip(), tags=tags, top_k=max(1, min(top_k, 100)),
            alpha=alpha, time_range=time_range,
            require_executable=bool(req_dict.get("require_executable", False)),
        )
        resp = await _retrieve_pipe.execute(request)
        log.info("API search: query=%r tags=%s top_k=%d alpha=%s → %d results",
                 request.query[:80],
                 request.tags.to_dict() if request.tags else "{}",
                 request.top_k, request.alpha, len(resp.nodes))
        return _json_ok({"nodes": [n.to_dict() for n in resp.nodes]})

    @_memory_svc.mcp("robonix/service/memgraph/compact")
    async def _mcp_compact(msg: Empty) -> String:
        """Promote overflowing ShortTerm memory nodes (oldest first) to LongTerm.

        The compaction threshold defaults to 50 ShortTerm nodes.  When the
        count exceeds the threshold the oldest ShortTerm nodes are promoted
        to the LongTerm ID range (1000+).  This keeps the retrieval index
        fast — ShortTerm nodes are a small hot set; LongTerm nodes are the
        persistent archive.

        Takes no parameters (Empty trigger).

        Response JSON:
        {
          "summary":          "Compacted 7 short-term nodes → long-term (50 remaining, 65 total).",
          "nodes_compacted":  7
        }
        """
        _ = msg
        log.info("API compact: trigger (%d nodes total, %d tags indexed)",
                 _graph.count(), _tags.count())
        resp = await _compact_pipe.execute()
        log.info("API compact: done — %d compacted (%d total)",
                 resp.nodes_compacted, _graph.count())
        return _json_ok({"summary": resp.summary, "nodes_compacted": resp.nodes_compacted})

    @_memory_svc.on_init
    def _on_init(cfg):
        _ = cfg
        global _indices_initialized
        log.info("on_init: MEMORY_DIR=%s initialized=%s", MEMORY_DIR, _indices_initialized)

        # Idempotent guard: only rebuild indices once per process lifetime.
        # Subsequent chat sessions trigger on_init again, but the in-memory
        # indices (_tags, _vectors) already contain all data written by
        # previous remember calls. Rebuilding from _graph would destroy
        # concurrent writes (clear-then-rebuild is not atomic).
        if _indices_initialized:
            log.info("on_init: indices already initialized (%d nodes), skipping rebuild",
                     _tags.count())
            return Ok()

        try:
            nodes = [_graph.get_node(nid) for nid in _graph.all_ids()]
            nodes = [n for n in nodes if n is not None]
            _tags.rebuild(nodes)
            vec_triples = [(n.node_id, n.embedding, n.summary) for n in nodes]
            _vectors.rebuild(vec_triples)
            _indices_initialized = True
            log.info("on_init: indices rebuilt for %d nodes", len(nodes))
        except Exception as e:
            log.warning("on_init: index rebuild failed (empty store?): %s: %s",
                        type(e).__name__, e)
        return Ok()


# ── 5. Entrypoint ───────────────────────────────────────────────────────

def main() -> int:
    """Run the MCP server (requires robonix_api)."""
    if not _MCP_AVAILABLE:
        log.error("robonix_api not available — cannot start MCP server")
        return 1

    _log_environment()
    log.info("memory_dir = %s", MEMORY_DIR)
    log.info("backend: GraphStore(%d nodes), embedding=%s (semantic=%s)",
             _graph.count(), _vectors._config.model_name, _vectors.is_semantic)
    log.info("scribe_mem service ready; entering memory_svc.run()")
    _memory_svc.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
