
system/scene/scene_service/mcp_tools.py
Comment on lines +151 to +152
        log.info("scene_hook: encoded %dx%d → JPEG %.1f KB (raw %.1f KB, ratio %.0f%%, %dms)",
                 w, h, jpg_kb, raw_kb, 100 * jpg_kb / max(raw_kb, 1), encode_ms, round(encode_ms))
Copilot commented 1 minute ago
Copilot
CopilotAI
1m ago

    High

The log format string and the number of arguments don’t match here (there are 6 %-placeholders but 7 arguments). This will raise a TypeError during logging and abort the Scene Hook execution path.
Suggested changeset 1 (1)
system/scene/scene_service/mcp_tools.py
Original file line number	Diff line number	Diff line change
         img_b64 = base64.b64encode(jpg_bytes).decode("ascii")
         raw_kb, jpg_kb = len(raw) / 1024, len(jpg_bytes) / 1024
         encode_ms = (time.time() - t_encode) * 1000
         log.info("scene_hook: encoded %dx%d → JPEG %.1f KB (raw %.1f KB, ratio %.0f%%, %dms)",
                  w, h, jpg_kb, raw_kb, 100 * jpg_kb / max(raw_kb, 1), encode_ms, round(encode_ms))
         log.info(
             "scene_hook: encoded %dx%d → JPEG %.1f KB (raw %.1f KB, ratio %.0f%%, %dms)",
             w, h, jpg_kb, raw_kb, 100 * jpg_kb / max(raw_kb, 1), round(encode_ms),
         )
         # ── build remember request ────────────────────────────────
         msg = (
Copilot uses AI. Check for mistakes.
@ohhhHwH
services/memory/memory_service/core/types.py
Comment on lines +335 to +343
    def to_dict(self) -> Dict[str, Any]:
        return {
            "session_id": self.session_id,
            "plan_id": self.plan_id,
            "log_record": self.log_record.to_dict(),
            "spatial": self.spatial.to_dict() if self.spatial else None,
            "parent_node_id": self.parent_node_id,
            "kv": self.kv,
        }
Copilot commented 1 minute ago
Copilot
CopilotAI
1m ago

    Medium

RememberRequest.image_base64 is never serialized in to_dict(), so callers (e.g. scene_hook/object_watchdog) sending image_base64 over JSON will silently drop images and the remember pipeline won’t persist them.

This issue also appears on line 346 of the same file.
Suggested changeset 1 (1)
services/memory/memory_service/core/types.py
Original file line number	Diff line number	Diff line change
             "log_record": self.log_record.to_dict(),
             "spatial": self.spatial.to_dict() if self.spatial else None,
             "parent_node_id": self.parent_node_id,
             "image_base64": self.image_base64,
             "kv": self.kv,
         }
Copilot uses AI. Check for mistakes.
@ohhhHwH
services/memory/memory_service/core/types.py
Comment on lines +376 to +386
    def to_dict(self) -> Dict[str, Any]:
        d: Dict[str, Any] = {"query": self.query, "top_k": self.top_k}
        if self.tags is not None:
            d["tags"] = self.tags.to_dict()
        if self.alpha is not None:
            d["alpha"] = self.alpha
        if self.time_range is not None:
            d["time_range"] = {"start_ts": self.time_range.start_ts,
                               "end_ts": self.time_range.end_ts}
        d["require_executable"] = self.require_executable
        return d
Copilot commented 1 minute ago
Copilot
CopilotAI
1m ago

    Medium

SearchRequest.vlm_qa is used by the retrieval pipeline, but it’s never serialized in to_dict(). That makes it impossible to enable VLM QA from JSON-over-String callers.

This issue also appears on line 389 of the same file.
Suggested changeset 1 (1)
services/memory/memory_service/core/types.py
Original file line number	Diff line number	Diff line change
             d["time_range"] = {"start_ts": self.time_range.start_ts,
                                "end_ts": self.time_range.end_ts}
         d["require_executable"] = self.require_executable
         if self.vlm_qa:
             d["vlm_qa"] = True
         return d
     @classmethod
Copilot uses AI. Check for mistakes.
@ohhhHwH
services/memory/memory_service/core/retrieve.py
Comment on lines +72 to +83
        elif self._vectors.is_semantic:
            # Path B: BM25 + Cosine hybrid (embedding model installed)
            log.info("search: LLM unavailable — using embedding ranker")
            ranked = self._vectors.search(
                query=request.query,
                candidate_ids=candidate_ids,
                top_k=overfetch,
                alpha=request.alpha,
            )
        else:
            # Path C: chronological fallback (best-effort)
            log.info("search: LLM and embedding unavailable — chronological")
Copilot commented 1 minute ago
Copilot
CopilotAI
1m ago

    Medium

When no LLM is configured and sentence-transformers isn’t available, this code falls back to chronological ranking and ignores the BM25 index (and any hash/vector fallback). That makes search quality unpredictable and can break the intended “tag filter → BM25” behavior in tests and demos.
Suggested changeset 1 (1)
services/memory/memory_service/core/retrieve.py
Copilot uses AI. Check for mistakes.
@ohhhHwH
services/memory/memory_service/storage/image_store.py
Comment on lines +1 to +6
"""Image Store — local image persistence under data/images/{node_id}/.

Each MemoryNode's observation frames are stored as PNG files in a
per-node subdirectory.  Paths returned are relative to the service
package root (services/memory/).
"""
Copilot commented 1 minute ago
Copilot
CopilotAI
1m ago

    Low

The ImageStore docstring says frames are stored as PNG, but save() always writes .jpg. This mismatch makes it harder to reason about the on-disk format and can mislead callers/debugging.
Suggested changeset 1 (1)
services/memory/memory_service/storage/image_store.py
Original file line number	Diff line number	Diff line change
 """Image Store — local image persistence under data/images/{node_id}/.
 Each MemoryNode's observation frames are stored as PNG files in a
 per-node subdirectory.  Paths returned are relative to the service
 package root (services/memory/).
 Each MemoryNode's observation frames are stored as JPEG files (with legacy
 support for PNG) in a per-node subdirectory. Paths returned are relative to the
 service package root (services/memory/).
 """
 from __future__ import annotations
Copilot uses AI. Check for mistakes.
@ohhhHwH
services/memory/memory_service/core/remember.py
Comment on lines +229 to +231
                # Persist image_refs to JSON — add_node() already wrote
                # a snapshot without them.
                self._graph._persist()
Copilot commented 1 minute ago
Copilot
CopilotAI
1m ago

    Low

This updates GraphStore persistence by calling a private method. Using the public update_node() keeps versioning consistent and avoids relying on GraphStore internals.
Suggested changeset 1 (1)
services/memory/memory_service/core/remember.py
Original file line number	Diff line number	Diff line change
                 img_bytes = _b64.b64decode(img_b64)
                 saved_path = self._images.save(node_id, img_bytes)
                 node.image_refs = self._images.list(node_id)
                 # Persist image_refs to JSON — add_node() already wrote
                 # a snapshot without them.
                 self._graph._persist()
                 # Persist image_refs to GraphStore (add_node() already wrote a snapshot without them).
                 self._graph.update_node(node_id, node)
                 img_ms = (time.time() - t_img) * 1000
                 log.info("remember: node %d → saved image %s (%.1f KB, %dms)",
                          node_id, saved_path, len(img_bytes) / 1024, round(img_ms))
Copilot uses AI. Check for mistakes.
@ohhhHwH
system/scene/scene_service/object_watchdog.py
Comment on lines +199 to +200
        import json as _json
        import httpx  # noqa: F811 (module-level, but kept lazy for non-watchdog paths)
Copilot commented 1 minute ago
Copilot
CopilotAI
1m ago

    Low

This import is unused. Keeping it will trigger common linters (flake8/ruff) and is easy to remove.
Suggested changeset 1 (1)
system/scene/scene_service/object_watchdog.py
Original file line number	Diff line number	Diff line change
     async def _save_object(self, obj, img_b64: str) -> bool:
         """POST a single-object remember request to the memgraph Scene Hook."""
         import json as _json
         import httpx  # noqa: F811 (module-level, but kept lazy for non-watchdog paths)
         now_ns = time.time_ns()