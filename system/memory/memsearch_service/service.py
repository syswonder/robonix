"""memsearch_service — long-term memory store + 3 MCP search/save/compact tools.

Indexes markdown notes under AGENT_MEMORY_DIR with milvus-lite (ONNX
embeddings). VLM credentials reused from pilot's env (VLM_BASE_URL /
VLM_API_KEY / VLM_MODEL) so memory doesn't need a separate API key.
"""
from __future__ import annotations

import asyncio
import logging
import os
from datetime import date
from pathlib import Path

# Suppress verbose gRPC / absl warnings from milvus-lite.
os.environ.setdefault("GRPC_VERBOSITY", "ERROR")
os.environ.setdefault("GLOG_minloglevel", "2")
logging.getLogger("absl").setLevel(logging.ERROR)

from robonix_api import Capability, Ok, Err, Deferred  # noqa: E402
from std_msgs_mcp import Empty, String  # noqa: E402
from memsearch import MemSearch  # noqa: E402

cap = Capability(id="memory", namespace="robonix/system/memory")

MEMORY_DIR = os.environ.get("AGENT_MEMORY_DIR", "./agent_memory")
MILVUS_URI = os.environ.get("AGENT_MILVUS_URI", "./agent_milvus.db")

mem = MemSearch(
    paths=[MEMORY_DIR],
    embedding_provider="onnx",
    milvus_uri=MILVUS_URI,
)


@cap.mcp("robonix/system/memory/search")
async def search(msg: String) -> String:
    """Search the agent's long-term memory for relevant past context, decisions, or user preferences.
    Contract: robonix/system/memory/search."""
    try:
        results = await mem.search(msg.data, top_k=2)
    except Exception as e:
        logging.warning("[memsearch] search failed (returning empty): %s", e)
        return String(data="No relevant memories found (search unavailable).")
    if not results:
        return String(data="No relevant memories found.")
    context = "\n\n".join(f"- {m['content']}" for m in results)
    return String(data=f"Relevant memories:\n{context}")


@cap.mcp("robonix/system/memory/save")
async def save(msg: String) -> String:
    """Save an important fact, user preference, or decision to long-term memory.
    Contract: robonix/system/memory/save."""
    p = Path(MEMORY_DIR) / f"{date.today()}_notes.md"
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "a") as f:
        f.write(f"\n{msg.data}\n")
    try:
        await mem.index()
    except Exception as e:
        logging.warning("[memsearch] re-index after save failed: %s", e)
    return String(data="Memory saved and indexed.")


@cap.mcp("robonix/system/memory/compact")
async def compact(msg: Empty) -> String:
    """Compact and summarize recent memories. Call this at the end of a session.
    Returns std_msgs/String JSON. Contract: robonix/system/memory/compact.

    Reuses pilot's OpenAI-compatible LLM endpoint so memory doesn't need a
    separate API key. Reads VLM_BASE_URL / VLM_API_KEY / VLM_MODEL (or the
    OPENAI_* equivalents as fallback)."""
    _ = msg
    base_url = os.environ.get("VLM_BASE_URL") or os.environ.get("OPENAI_BASE_URL")
    api_key = os.environ.get("VLM_API_KEY") or os.environ.get("OPENAI_API_KEY")
    model = os.environ.get("VLM_MODEL") or os.environ.get("OPENAI_MODEL") or "gpt-5.4-mini-mini"
    if not api_key:
        return String(data="compact: no LLM credentials available. "
                           "Set VLM_API_KEY in the deploy manifest's system.memory env block.")
    try:
        summary_path = await mem.compact(
            llm_provider="openai", model=model,
            base_url=base_url, api_key=api_key,
        )
        return String(data=f"Memory compacted successfully to {summary_path}.")
    except Exception as e:
        return String(data=f"Failed to compact memory: {e}")


@cap.on_init
def init(cfg):
    """Boot-time index of the corpus. Empty corpus is fine — index() returns
    quickly with no docs. Don't fail Init on indexing errors; search/save
    handlers degrade gracefully."""
    try:
        asyncio.run(mem.index())
    except Exception as e:
        logging.warning("[memsearch] initial index failed (empty corpus?): %s", e)
    return Ok()


def main() -> int:
    cap.run()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
