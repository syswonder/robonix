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
            if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
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


def _ensure_robonix_py() -> None:
    """Add the shared Python helper lib (crates/robonix-py) to sys.path.

    Uses `rbnx path robonix-py` when available; falls back to PYTHONPATH
    injected by the package build.sh (rbnx-build/ws/install/setup.bash).
    """
    import subprocess
    try:
        out = subprocess.run(
            ["rbnx", "path", "robonix-py"],
            capture_output=True, text=True, timeout=5, check=False,
        )
        if out.returncode == 0:
            lib = Path(out.stdout.strip())
            if lib.is_dir() and str(lib) not in sys.path:
                sys.path.insert(0, str(lib))
    except (FileNotFoundError, subprocess.TimeoutExpired):
        pass  # rbnx not installed → rely on PYTHONPATH set by build.sh


_ensure_robonix_py()

import grpc
import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc
import std_msgs_mcp

from mcp.server.fastmcp import FastMCP
from memsearch import MemSearch

from robonix_py import mcp_contract

mcp = FastMCP("memsearch_provider")

MEMORY_DIR = os.environ.get("AGENT_MEMORY_DIR", "./agent_memory")
MILVUS_URI = os.environ.get("AGENT_MILVUS_URI", "./agent_milvus.db")

mem = MemSearch(
    paths=[MEMORY_DIR],
    embedding_provider="onnx",
    milvus_uri=MILVUS_URI,
)


# ── Contract: robonix/srv/memory/search ──────────────────────────────────────

@mcp_contract(
    mcp,
    contract_id="robonix/srv/memory/search",
)
async def search_memory(msg: std_msgs_mcp.String) -> std_msgs_mcp.String:
    """Search the agent's long-term memory for relevant past context, decisions, or user preferences.
    Contract: robonix/srv/memory/search (input/output std_msgs/String)."""
    try:
        results = await mem.search(msg.data, top_k=2)
    except Exception as e:
        logging.warning("[memsearch] search_memory failed (returning empty): %s", e)
        return std_msgs_mcp.String(
            data="No relevant memories found (search unavailable)."
        )
    if not results:
        return std_msgs_mcp.String(data="No relevant memories found.")
    context = "\n\n".join(f"- {m['content']}" for m in results)
    return std_msgs_mcp.String(data=f"Relevant memories:\n{context}")


# ── Contract: robonix/srv/memory/compact ─────────────────────────────────────

@mcp_contract(
    mcp,
    contract_id="robonix/srv/memory/compact",
)
async def compact_memory(msg: std_msgs_mcp.Empty) -> std_msgs_mcp.String:
    """Compact and summarize recent memories. Call this at the end of a session.
    Contract: robonix/srv/memory/compact (input std_msgs/Empty, output std_msgs/String).

    Reuses pilot's OpenAI-compatible LLM endpoint so memory doesn't need a
    separate API key. Reads:

        VLM_BASE_URL  / OPENAI_BASE_URL   — base URL of the OpenAI-compatible API
        VLM_API_KEY   / OPENAI_API_KEY    — API key
        VLM_MODEL     / OPENAI_MODEL      — model identifier (default: gpt-4o-mini)

    `VLM_*` are the names used in robonix_manifest.yaml `system.pilot.vlm.*`;
    `OPENAI_*` are accepted as a fallback so this also works under a stock
    OpenAI deployment."""
    _ = msg
    base_url = os.environ.get("VLM_BASE_URL") or os.environ.get("OPENAI_BASE_URL")
    api_key = os.environ.get("VLM_API_KEY") or os.environ.get("OPENAI_API_KEY")
    model = os.environ.get("VLM_MODEL") or os.environ.get("OPENAI_MODEL") or "gpt-4o-mini"
    if not api_key:
        return std_msgs_mcp.String(
            data="compact_memory: no LLM credentials available. "
                 "Set VLM_API_KEY (and VLM_BASE_URL for non-default endpoints) "
                 "in the deploy manifest's system.memory env block."
        )
    try:
        summary_path = await mem.compact(
            llm_provider="openai",
            model=model,
            base_url=base_url,
            api_key=api_key,
        )
        return std_msgs_mcp.String(
            data=f"Memory compacted successfully to {summary_path}."
        )
    except Exception as e:
        return std_msgs_mcp.String(data=f"Failed to compact memory: {e}")


# ── Contract: robonix/srv/memory/save ────────────────────────────────────────

@mcp_contract(
    mcp,
    contract_id="robonix/srv/memory/save",
)
async def save_memory(msg: std_msgs_mcp.String) -> std_msgs_mcp.String:
    """Save an important fact, user preference, or decision to long-term memory.
    Contract: robonix/srv/memory/save (input/output std_msgs/String)."""
    from datetime import date
    p = Path(MEMORY_DIR) / f"{date.today()}_notes.md"
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "a") as f:
        f.write(f"\n{msg.data}\n")
    try:
        await mem.index()
    except Exception as e:
        logging.warning("[memsearch] re-index after save failed: %s", e)
    return std_msgs_mcp.String(data="Memory saved and indexed.")


def _single_tool_meta(tool_name: str, description: str, input_schema: dict) -> str:
    return json.dumps({
        "tools": [{
            "name": tool_name,
            "description": description,
            "input_schema": input_schema,
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
                namespace="robonix/srv/memory",
                kind="service",
                skill_md="# Memsearch MCP\nProvides memory search, save, and compact operations.",
            )
        )

        # One DeclareInterface per contract —————————————————————————————————

        # robonix/srv/memory/search
        stub.DeclareInterface(
            pb.DeclareInterfaceRequest(
                node_id=node_id,
                name="search",
                supported_transports=["mcp"],
                metadata_json=_single_tool_meta(
                    "search_memory",
                    "Search agent long-term memory. data: the search query string.",
                    std_msgs_mcp.String.json_schema(),
                ),
                listen_port=port,
                contract_id="robonix/srv/memory/search",
            )
        )

        # robonix/srv/memory/save
        stub.DeclareInterface(
            pb.DeclareInterfaceRequest(
                node_id=node_id,
                name="save",
                supported_transports=["mcp"],
                metadata_json=_single_tool_meta(
                    "save_memory",
                    "Save fact or preference to long-term memory. data: content to save.",
                    std_msgs_mcp.String.json_schema(),
                ),
                listen_port=port,
                contract_id="robonix/srv/memory/save",
            )
        )

        # robonix/srv/memory/compact  (input: Empty → no parameters)
        stub.DeclareInterface(
            pb.DeclareInterfaceRequest(
                node_id=node_id,
                name="compact",
                supported_transports=["mcp"],
                metadata_json=_single_tool_meta(
                    "compact_memory",
                    "Compact and summarize recent memories. No input required.",
                    std_msgs_mcp.Empty.json_schema(),
                ),
                listen_port=port,
                contract_id="robonix/srv/memory/compact",
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
