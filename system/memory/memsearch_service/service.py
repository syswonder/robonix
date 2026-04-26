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
    """Locate rbnx-build/codegen/proto_gen produced by `rbnx codegen
    --out-dir rbnx-build/codegen`. All build artefacts live under
    `<pkg>/rbnx-build/`; nothing should land at package root."""
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "rbnx-build" / "codegen" / "proto_gen"
        if pg.is_dir() and (pg / "atlas_pb2.py").exists():
            if str(pg) not in sys.path:
                sys.path.insert(0, str(pg))
            return
        d = d.parent


def _ensure_mcp_types() -> None:
    """Locate rbnx-build/codegen/robonix_mcp_types from
    `rbnx codegen --mcp --out-dir rbnx-build/codegen`."""
    d = Path(__file__).resolve().parent
    while d.parent != d:
        mt = d / "rbnx-build" / "codegen" / "robonix_mcp_types"
        if mt.is_dir() and (mt / "__init__.py").exists():
            if str(mt) not in sys.path:
                sys.path.insert(0, str(mt))
            return
        d = d.parent


def _ensure_robonix_py() -> None:
    """Find pylib/robonix-py via `rbnx path` (host) or walk-up (container)."""
    d = Path(__file__).resolve().parent
    while d.parent != d:
        for cand in (d / "pylib" / "robonix-py", d / "robonix-py"):
            if cand.is_dir() and (cand / "robonix_py" / "__init__.py").exists():
                if str(cand) not in sys.path:
                    sys.path.insert(0, str(cand))
                return
        d = d.parent
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
        pass


_ensure_robonix_py()

import grpc
import atlas_pb2 as pb
import atlas_pb2_grpc as pb_grpc
import std_msgs_mcp
from std_msgs_mcp import Empty, String

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


# ── Contract: robonix/system/memory/search ──────────────────────────────────────

@mcp_contract(mcp, contract_id="robonix/system/memory/search")
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


# ── Contract: robonix/system/memory/compact ─────────────────────────────────────

@mcp_contract(mcp, contract_id="robonix/system/memory/compact")
async def compact(msg: Empty) -> String:
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
    _ = msg
    base_url = os.environ.get("VLM_BASE_URL") or os.environ.get("OPENAI_BASE_URL")
    api_key = os.environ.get("VLM_API_KEY") or os.environ.get("OPENAI_API_KEY")
    model = os.environ.get("VLM_MODEL") or os.environ.get("OPENAI_MODEL") or "gpt-4o-mini"
    if not api_key:
        return String(data="compact: no LLM credentials available. "
                           "Set VLM_API_KEY (and VLM_BASE_URL for non-default endpoints) "
                           "in the deploy manifest's system.memory env block.")
    try:
        summary_path = await mem.compact(
            llm_provider="openai",
            model=model,
            base_url=base_url,
            api_key=api_key,
        )
        return String(data=f"Memory compacted successfully to {summary_path}.")
    except Exception as e:
        return String(data=f"Failed to compact memory: {e}")


# ── Contract: robonix/system/memory/save ────────────────────────────────────────

@mcp_contract(mcp, contract_id="robonix/system/memory/save")
async def save(msg: String) -> String:
    """Save an important fact, user preference, or decision to long-term memory.
    Contract: robonix/system/memory/save."""
    from datetime import date
    p = Path(MEMORY_DIR) / f"{date.today()}_notes.md"
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "a") as f:
        f.write(f"\n{msg.data}\n")
    try:
        await mem.index()
    except Exception as e:
        logging.warning("[memsearch] re-index after save failed: %s", e)
    return String(data="Memory saved and indexed.")


def _decl_mcp(stub, cap_id: str, contract_id: str, port: int, fn) -> None:
    """Atlas registration derived from the @mcp_contract handler."""
    description = (fn.__doc__ or "").strip()
    input_cls = getattr(fn, "_robonix_input_cls", None)
    schema_json = json.dumps(
        input_cls.json_schema() if input_cls is not None
        else {"type": "object", "properties": {}, "required": []}
    )
    stub.DeclareInterface(pb.DeclareInterfaceRequest(
        capability_id=cap_id,
        contract_id=contract_id,
        transport=pb.TRANSPORT_MCP,
        endpoint=f"http://127.0.0.1:{port}/mcp/",
        params=pb.TransportParams(mcp=pb.McpParams(
            description=description,
            input_schema_json=schema_json,
        )),
    ))


def main():
    channel = grpc.insecure_channel(os.environ.get("ROBONIX_ATLAS", "localhost:50051"))
    stub = pb_grpc.AtlasStub(channel)

    port = int(os.environ.get("MEMSEARCH_PORT", "50105"))
    cap_id = os.environ.get("ROBONIX_CAPABILITY_ID", "com.robonix.system.memory")

    try:
        pkg_dir = os.environ.get("ROBONIX_PKG_HOST_DIR", "")
        md_path = f"{pkg_dir}/CAPABILITY.md" if pkg_dir else ""
        stub.RegisterCapability(pb.RegisterCapabilityRequest(
            capability_id=cap_id,
            namespace="robonix/system/memory",
            capability_md_path=md_path,
        ))
        _decl_mcp(stub, cap_id, "robonix/system/memory/search",  port, search)
        _decl_mcp(stub, cap_id, "robonix/system/memory/save",    port, save)
        _decl_mcp(stub, cap_id, "robonix/system/memory/compact", port, compact)
        print(f"[memsearch-service] registered cap {cap_id} → 3 interfaces on port {port}")
    except Exception as e:
        print(f"[memsearch-service] WARN: atlas registration failed: {e}")

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
