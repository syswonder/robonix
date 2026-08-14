"""Scribe Mem service (memgraph) — structured CKG memory for Robonix.

Runs as a robonix-api Service with MCP tool registration.
Entrypoint for rbnx boot: `python -m memory_service.service`

MCP tools (registered under robonix/service/memory/ alongside memsearch):
  - robonix/service/memory/remember       — write structured MemoryNode
  - robonix/service/memory/hybrid_search  — 3-stage hybrid retrieval (BM25+Embedding)
  - robonix/service/memory/promote        — promote short-term → long-term

Runs in parallel with memsearch (save/search/compact) under the same
robonix/service/memory/ namespace so Pilot discovers both sets of tools.

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
    log.info("embedding: enabled=%s (set MEMGRAPH_ENABLE_EMBEDDING=1 to activate)",
             _ENABLE_EMBEDDING)
    log.info("clean_start: enabled=%s (set MEMGRAPH_KEEP_DATA=1 to preserve)",
             not _KEEP_DATA)
    model_path = os.environ.get("EMBEDDING_MODEL_PATH", "")
    if model_path:
        log.info("embedding model path: %s (exists=%s)",
                 model_path, os.path.isdir(model_path))
    else:
        log.info("embedding model source: sentence-transformers standard cache")


# ── 1. Core backend imports (no rbnx-api dependency) ───────────────────

from .core.types import (  # noqa: E402
    LogRecord, SpatialContext, ObjectCoord,
    TagFilter, TimeRange, RememberRequest, SearchRequest,
)
from .storage.graph_store import GraphStore  # noqa: E402
from .storage.tag_index import TagIndex  # noqa: E402
from .storage.vector_store import VectorStore  # noqa: E402
from .storage.embedding_config import EmbeddingModelConfig  # noqa: E402
from .storage.image_store import ImageStore  # noqa: E402
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

# ── 2a. Clean slate: wipe persisted data on every boot ────────────────
# The memory graph is ephemeral per session — delete prior graph and
# all observation images so each rbnx boot starts from a clean state.
# Set MEMGRAPH_KEEP_DATA=1 to opt out (e.g. debugging persistence bugs).
_KEEP_DATA = os.environ.get("MEMGRAPH_KEEP_DATA", "0") in ("1", "true", "yes")

def _clean_slate() -> None:
    """Delete graph_store.json and all image directories from prior runs."""
    import shutil
    graph_json = os.path.join(MEMORY_DIR, "graph_store.json")
    if os.path.exists(graph_json):
        os.remove(graph_json)
        log.info("scribe_mem: cleaned %s", graph_json)

    images_dir = str(Path(__file__).resolve().parent.parent / "data" / "images")
    if os.path.isdir(images_dir):
        count = 0
        for entry in os.listdir(images_dir):
            entry_path = os.path.join(images_dir, entry)
            try:
                if os.path.isdir(entry_path):
                    shutil.rmtree(entry_path, ignore_errors=True)
                else:
                    os.remove(entry_path)
                count += 1
            except OSError as e:
                log.debug("scribe_mem: clean_images: %s: %s", entry, e)
        if count:
            log.info("scribe_mem: cleaned %d entries from %s", count, images_dir)

if not _KEEP_DATA:
    _clean_slate()

# ── 2b. Embedding toggle ──────────────────────────────────────────────
# Embedding (sentence-transformers / all-MiniLM-L6-v2) is OFF by default.
# Set MEMGRAPH_ENABLE_EMBEDDING=1 to load the model and enable the
# BM25+Cosine hybrid ranking path (Path B in retrieve.py).
_ENABLE_EMBEDDING = os.environ.get("MEMGRAPH_ENABLE_EMBEDDING", "0") in ("1", "true", "yes")

graph_type = "GraphStore"
idx_type = "TagIndex"
vec_type = "VectorStore(BM25 only)" if not _ENABLE_EMBEDDING else "VectorStore(BM25+Embedding)"
log.info("scribe_mem: graph_store → %s", os.path.join(MEMORY_DIR, "graph_store.json"))
log.info("scribe_mem: scribe_log  → %s", os.path.join(_LOG_DIR, "service_memory.log"))
log.info("scribe_mem: backend     → %s + %s + %s + ImageStore", graph_type, idx_type, vec_type)

_graph = GraphStore(data_dir=MEMORY_DIR)
_tags = TagIndex()
_vectors = VectorStore(alpha=0.3, embedding_enabled=_ENABLE_EMBEDDING)
_images = ImageStore()
_remember_pipe = RememberPipeline(_graph, _tags, _vectors, _images)
_retrieve_pipe = RetrievePipeline(_graph, _tags, _vectors)
_compact_pipe = CompactPipeline(_graph)

# ── 2c. Scene Hook HTTP server ───────────────────────────────────────
# Must start at module level — rbnx boot calls robonix_api.Service.run()
# directly, bypassing main().  The server listens for raw JSON POSTs
# from Scene's ObjectWatchdog and Scene Hook (mcp_tools.py).
# Called AFTER _start_scene_hook_server is defined below.
# See line ~640 for the actual call.

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
            self.vectors = VectorStore(config=ecfg, alpha=alpha,
                                      embedding_enabled=_ENABLE_EMBEDDING)
        else:
            self.graph = _graph
            self.tags = _tags
            self.vectors = _vectors

        # Always create own pipelines bound to own stores
        self._images = ImageStore()
        self._remember_pipe = RememberPipeline(self.graph, self.tags, self.vectors, self._images)
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
                                objects=None, parent_node_id=None,
                                spatial_origin: str = ""):
        """Build a memory record while preserving explicit spatial origin."""
        lr = LogRecord(level=level, tag=tag, msg=msg)
        spatial = None
        if objects:
            spatial = SpatialContext(
                objects=[ObjectCoord(obj_id=o[0], label=o[1],
                                     x=o[2] if len(o)>2 else 0,
                                     y=o[3] if len(o)>3 else 0,
                                     z=o[4] if len(o)>4 else 0)
                         for o in objects],
                origin=spatial_origin,
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
    _memory_svc = Service(id="memgraph", namespace="robonix/service/memory")
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

    @_memory_svc.mcp("robonix/service/memory/remember")
    async def _mcp_remember(msg: String) -> String:
        """Write a structured memory node into the CKG (Causal Knowledge Graph).

        Observations are saved AUTOMATICALLY — when scene.list_objects detects
        objects, the Scene service captures the camera frame and saves
        everything to memory.  Do NOT call remember after list_objects
        or camera_snapshot — it is already handled for you.

        Use remember only for EXPLICIT saves: user preferences ("the user
        likes the red cup"), action results ("grasp succeeded"), navigation
        events, or any fact the robot should recall later.

        Each node is automatically tagged across four dimensions (spatial /
        behavioural / cognitive / optimisation) and embedded for hybrid
        BM25+vector search.

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
            "origin":  "frame_from_scene",
            "objects": [
              {"obj_id": "scene.obj.cup_001", "label": "red cup",
               "x": 1.0, "y": 2.0, "z": 0.8}
            ]
          },
          "parent_node_id": 5,           // optional — causal parent
          "image_base64": "<base64>",    // optional — manual image attach
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
        b64_len = len(req_dict.get("image_base64", ""))
        n_objs = len((req_dict.get("spatial") or {}).get("objects", []))
        log.info("API remember: sid=%s pid=%s tag=%s msg=\"%s\" (%s chars) "
                 "has_image=%s b64len=%d objects=%d",
                 session_id, plan_id,
                 req_dict.get("log_record", {}).get("tag", "?"),
                 (req_dict.get("log_record", {}).get("msg", "") or "")[:60],
                 str(len(req_dict.get("log_record", {}).get("msg", ""))),
                 "yes" if req_dict.get("image_base64") else "no",
                 b64_len, n_objs)

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
            image_base64=req_dict.get("image_base64", ""),
            kv=req_dict.get("kv") if isinstance(req_dict.get("kv"), dict) else {},
        )
        resp = await _remember_pipe.execute(request)
        node = _graph.get_node(resp.node_id)
        summary = node.summary if node else "?"
        log.info("API remember: sid=%s pid=%s node_id=%s summary=\"%s\"",
                 request.session_id, request.plan_id, str(resp.node_id), summary)
        return _json_ok({"node_id": resp.node_id, "message": resp.message})

    @_memory_svc.mcp("robonix/service/memory/hybrid_search")
    async def _mcp_search(msg: String) -> String:
        """Search the CKG memory using a 3-stage pipeline: tag filter → hybrid
        BM25+embedding ranking → causal/time/weight filter.

        Returns ONLY memories that were previously saved via remember.  If no
        observations have been persisted (e.g. you looked but forgot to call
        remember), this will return empty results.

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
          "require_executable": false,                  // default false (Phase2)
          "vlm_qa": false                                // if true, return VLM answer from node images
        }

        Response JSON:
        {
          "nodes": [
            ...
          ],
          "vlm_answer": "There was a red cup on the kitchen counter."  // only when vlm_qa=true
        }
            {
              "node_id": 42,
              "summary": "successfully grasp red cup in kitchen",
              "tags": {"scene_type": "kitchen", "action_type": "grasp", ...},
              "spatial_data": {"objects": [...], "origin": "frame_from_scene"},
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
            vlm_qa=bool(req_dict.get("vlm_qa", False)),
        )
        resp = await _retrieve_pipe.execute(request)
        log.info("API search: query=\"%s\" tags=%s top_k=%s alpha=%s vlm_qa=%s → %s results",
                 request.query[:80],
                 request.tags.to_dict() if request.tags else "{}",
                 str(request.top_k), str(request.alpha), str(request.vlm_qa),
                 str(len(resp.nodes)))
        result = {"nodes": [n.to_dict() for n in resp.nodes]}
        if resp.vlm_answer:
            result["vlm_answer"] = resp.vlm_answer
        return _json_ok(result)

    @_memory_svc.mcp("robonix/service/memory/promote")
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
    log.info("backend: GraphStore(%d nodes), embedding=%s (enabled=%s)",
             _graph.count(),
             _vectors._config.model_name if _ENABLE_EMBEDDING else "disabled",
             _ENABLE_EMBEDDING)
    # ── Scene Hook HTTP endpoint (raw, no MCP session needed) ─────
    _start_scene_hook_server()

    log.info("scribe_mem service ready; entering memory_svc.run()")
    _memory_svc.run()
    return 0


_scene_hook_started = False

def _start_scene_hook_server() -> None:
    """Start a tiny HTTP server on port 37798 for Scene Hook direct POST.

    FastMCP's Streamable HTTP transport requires SSE session negotiation
    which is too complex for a simple service-to-service Hook.  This
    endpoint accepts raw JSON POST directly — no MCP protocol overhead.

    Idempotent: safe to call multiple times (e.g. module-level + main()).
    """
    global _scene_hook_started
    if _scene_hook_started:
        return
    _scene_hook_started = True

    import json as _json
    import threading
    from http.server import HTTPServer, BaseHTTPRequestHandler

    class _HookHandler(BaseHTTPRequestHandler):
        def do_POST(self):
            t0 = time.time()
            length = int(self.headers.get("Content-Length", 0))
            body = self.rfile.read(length) if length else b""
            try:
                data = _json.loads(body)
            except Exception:
                log.warning("scene_hook.http: bad JSON body (%d bytes)", len(body))
                self._reply(400, {"error": "invalid json"})
                return

            has_img = "yes" if data.get("image_base64") else "no"
            n_objs = len(data.get("spatial", {}).get("objects", []))
            b64_len = len(data.get("image_base64", ""))
            log.info("scene_hook.http: received — has_image=%s objects=%d b64len=%d body=%dB",
                     has_img, n_objs, b64_len, length)

            # Run the remember pipeline synchronously in this thread.
            import asyncio as _asyncio
            try:
                lr = LogRecord.from_dict(data.get("log_record", {}))
                spatial = None
                if data.get("spatial"):
                    spatial = SpatialContext.from_dict(data["spatial"])
                req = RememberRequest(
                    session_id=data.get("session_id", "scene-hook"),
                    plan_id=data.get("plan_id", "scene-hook"),
                    log_record=lr,
                    spatial=spatial,
                    image_base64=data.get("image_base64", ""),
                    kv=data.get("kv") if isinstance(data.get("kv"), dict) else {},
                )
                t_pipe = time.time()
                resp = _asyncio.run(_remember_pipe.execute(req))
                pipe_ms = (time.time() - t_pipe) * 1000
                total_ms = (time.time() - t0) * 1000
                log.info("scene_hook.http: → node %d (pipe %dms, total %dms)",
                         resp.node_id, round(pipe_ms), round(total_ms))
                self._reply(200, {"node_id": resp.node_id})
            except Exception as e:
                total_ms = (time.time() - t0) * 1000
                log.warning("scene_hook.http: ! pipeline error after %dms: %s: %s",
                           round(total_ms), type(e).__name__, e)
                self._reply(500, {"error": str(e)})

        def _reply(self, code, obj):
            body = _json.dumps(obj, ensure_ascii=False).encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, fmt, *args):
            log.debug("scene_hook: %s", fmt % args)

    port = int(os.environ.get("SCENE_HOOK_PORT", "37798"))
    try:
        server = HTTPServer(("0.0.0.0", port), _HookHandler)
    except OSError as e:
        # A stale sibling process (e.g. one killed without cleanup) can still
        # hold this port. The Hook endpoint is a best-effort side channel, not
        # part of the MCP capability surface (remember/hybrid_search/promote)
        # — losing it must not crash the whole service.
        log.warning("scene_hook: could not bind 0.0.0.0:%d (%s); "
                    "Scene Hook POST endpoint disabled for this run", port, e)
        return
    thread = threading.Thread(target=server.serve_forever, name="scene-hook", daemon=True)
    thread.start()
    log.info("scene_hook: HTTP endpoint on 0.0.0.0:%d", port)


# Start at module level so it works regardless of whether the process
# enters through main() or robonix_api.Service.run().
_start_scene_hook_server()

if __name__ == "__main__":
    sys.exit(main())
