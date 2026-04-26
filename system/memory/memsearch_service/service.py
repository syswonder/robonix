import json
import os
import sys
import asyncio
import logging
from pathlib import Path

# Suppress verbose gRPC / absl warnings from milvus-lite
os.environ["GRPC_VERBOSITY"] = "ERROR"
os.environ["GLOG_minloglevel"] = "2"
logging.getLogger("absl").setLevel(logging.ERROR)


def _ensure_proto_gen() -> None:
    """Locate proto_gen/ produced by `rbnx codegen --out-dir rbnx-build/codegen`.
    Falls back to the legacy package-root layout for not-yet-migrated builds."""
    d = Path(__file__).resolve().parent
    while d.parent != d:
        for pg in (d / "rbnx-build" / "codegen" / "proto_gen", d / "proto_gen"):
            if pg.is_dir() and (pg / "atlas_legacy_pb2.py").exists():
                if str(pg) not in sys.path:
                    sys.path.insert(0, str(pg))
                return
        d = d.parent


def _ensure_mcp_types() -> None:
    """Locate robonix_mcp_types/ from `rbnx codegen --mcp --out-dir rbnx-build/codegen`,
    or the legacy package-root layout."""
    d = Path(__file__).resolve().parent
    while d.parent != d:
        for mt in (d / "rbnx-build" / "codegen" / "robonix_mcp_types", d / "robonix_mcp_types"):
            if mt.is_dir() and (mt / "__init__.py").exists():
                if str(mt) not in sys.path:
                    sys.path.insert(0, str(mt))
                return
        d = d.parent


import grpc
import atlas_legacy_pb2 as pb
import atlas_legacy_pb2_grpc as pb_grpc
import std_msgs_mcp

from mcp.server.fastmcp import FastMCP
from memsearch import MemSearch

mcp = FastMCP("memsearch_provider")

MEMORY_DIR = os.environ.get("AGENT_MEMORY_DIR", "./agent_memory")
MILVUS_URI = os.environ.get("AGENT_MILVUS_URI", "./agent_milvus.db")

mem = MemSearch(
    paths=[MEMORY_DIR],
    embedding_provider="onnx",
    milvus_uri=MILVUS_URI,
)


# ── Contract: robonix/system/memory/search ──────────────────────────────────────

@mcp.tool(name="search")
async def search(data: str) -> dict:
    """Search the agent's long-term memory for relevant past context, decisions, or user preferences.
    Returns std_msgs/String JSON.
    Contract: robonix/system/memory/search."""
    try:
        results = await mem.search(data, top_k=2)
    except Exception as e:
        logging.warning("[memsearch] search failed (returning empty): %s", e)
        return {"data": "No relevant memories found (search unavailable)."}
    if not results:
        return {"data": "No relevant memories found."}
    context = "\n\n".join(f"- {m['content']}" for m in results)
    return {"data": f"Relevant memories:\n{context}"}


# ── Contract: robonix/system/memory/compact ─────────────────────────────────────

@mcp.tool(name="compact")
async def compact() -> dict:
    """Compact and summarize recent memories. Call this at the end of a session.
    Returns std_msgs/String JSON.
    Contract: robonix/system/memory/compact.

    Reuses pilot's OpenAI-compatible LLM endpoint so memory doesn't need a
    separate API key. Reads:

        VLM_BASE_URL  / OPENAI_BASE_URL   — base URL of the OpenAI-compatible API
        VLM_API_KEY   / OPENAI_API_KEY    — API key
        VLM_MODEL     / OPENAI_MODEL      — model identifier (default: gpt-4o-mini)

    `VLM_*` are the names used in robonix_manifest.yaml `system.pilot.vlm.*`;
    `OPENAI_*` are accepted as a fallback so this also works under a stock
    OpenAI deployment."""
    base_url = os.environ.get("VLM_BASE_URL") or os.environ.get("OPENAI_BASE_URL")
    api_key = os.environ.get("VLM_API_KEY") or os.environ.get("OPENAI_API_KEY")
    model = os.environ.get("VLM_MODEL") or os.environ.get("OPENAI_MODEL") or "gpt-4o-mini"
    if not api_key:
        return {"data": "compact: no LLM credentials available. "
                        "Set VLM_API_KEY (and VLM_BASE_URL for non-default endpoints) "
                        "in the deploy manifest's system.memory env block."}
    try:
        summary_path = await mem.compact(
            llm_provider="openai",
            model=model,
            base_url=base_url,
            api_key=api_key,
        )
        return {"data": f"Memory compacted successfully to {summary_path}."}
    except Exception as e:
        return {"data": f"Failed to compact memory: {e}"}


# ── Contract: robonix/system/memory/save ────────────────────────────────────────

@mcp.tool(name="save")
async def save(data: str) -> dict:
    """Save an important fact, user preference, or decision to long-term memory.
    Returns std_msgs/String JSON.
    Contract: robonix/system/memory/save."""
    from datetime import date
    p = Path(MEMORY_DIR) / f"{date.today()}_notes.md"
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "a") as f:
        f.write(f"\n{data}\n")
    try:
        await mem.index()
    except Exception as e:
        logging.warning("[memsearch] re-index after save failed: %s", e)
    return {"data": "Memory saved and indexed."}


def _meta(
    name: str,
    description: str,
    properties: dict | None = None,
    required: list[str] | None = None,
) -> str:
    """JSON-Schema for the @mcp.tool() function actually registered on
    this server. Kept lockstep with the Python signature so pydantic
    validation matches the schema the LLM sees."""
    return json.dumps({
        "tools": [{
            "name": name,
            "description": description,
            "input_schema": {
                "type": "object",
                "properties": properties or {},
                "required": list(required or []),
            },
        }]
    })


def main():
    channel = grpc.insecure_channel(os.environ.get("ROBONIX_ATLAS", "localhost:50051"))
    stub = pb_grpc.RobonixRuntimeStub(channel)

    port = int(os.environ.get("MEMSEARCH_PORT", "50105"))
    endpoint = f"http://127.0.0.1:{port}"
    node_id = "com.robonix.services.memsearch"

    try:
        stub.RegisterNode(
            pb.RegisterNodeRequest(
                node_id=node_id,
                namespace="robonix/system/memory",
                kind="service",
                skill_md="# Memsearch MCP\nProvides memory search, save, and compact operations.",
            )
        )

        # One DeclareInterface per contract —————————————————————————————————

        # robonix/system/memory/search
        stub.DeclareInterface(
            pb.DeclareInterfaceRequest(
                node_id=node_id,
                name="search",
                supported_transports=["mcp"],
                metadata_json=_meta(
                    "search",
                    "Search the agent's long-term memory for relevant past context, decisions, or user preferences.",
                    properties={"data": {"type": "string", "description": "the search query string"}},
                    required=["data"],
                ),
                listen_port=port,
                contract_id="robonix/system/memory/search",
            )
        )

        # robonix/system/memory/save
        stub.DeclareInterface(
            pb.DeclareInterfaceRequest(
                node_id=node_id,
                name="save",
                supported_transports=["mcp"],
                metadata_json=_meta(
                    "save",
                    "Save an important fact, user preference, or decision to long-term memory.",
                    properties={"data": {"type": "string", "description": "the content to save"}},
                    required=["data"],
                ),
                listen_port=port,
                contract_id="robonix/system/memory/save",
            )
        )

        # robonix/system/memory/compact (no parameters)
        stub.DeclareInterface(
            pb.DeclareInterfaceRequest(
                node_id=node_id,
                name="compact",
                supported_transports=["mcp"],
                metadata_json=_meta(
                    "compact",
                    "Compact and summarize recent memories. Call at the end of a session.",
                ),
                listen_port=port,
                contract_id="robonix/system/memory/compact",
            )
        )

        print(f"[memsearch-service] Registered node {node_id} — 3 interfaces at {endpoint}")
    except Exception as e:
        print(f"[memsearch-service] Warning: Failed to register with control plane: {e}")

    try:
        asyncio.run(mem.index())
    except Exception as e:
        logging.warning("[memsearch] initial index failed (empty corpus?): %s", e)

    print(f"[memsearch-service] Starting MCP Streamable HTTP server on port {port}...")
    import uvicorn
    logging.getLogger("absl").setLevel(logging.ERROR)
    app = mcp.streamable_http_app()
    uvicorn.run(app, host="127.0.0.1", port=port, log_level="warning")


if __name__ == "__main__":
    main()
