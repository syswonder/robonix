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
    d = Path(__file__).resolve().parent
    while d.parent != d:
        pg = d / "proto_gen"
        if pg.is_dir() and (pg / "robonix_runtime_pb2.py").exists():
            sys.path.insert(0, str(pg))
            return
        d = d.parent

_ensure_proto_gen()
import grpc
import robonix_runtime_pb2 as pb
import robonix_runtime_pb2_grpc as pb_grpc

from mcp.server.fastmcp import FastMCP
from memsearch import MemSearch

mcp = FastMCP("memsearch_provider")

# Initialize MemSearch
MEMORY_DIR = os.environ.get("AGENT_MEMORY_DIR", "./agent_memory")
MILVUS_URI = os.environ.get("AGENT_MILVUS_URI", "./agent_milvus.db")

mem = MemSearch(
    paths=[MEMORY_DIR],
    embedding_provider="onnx",
    milvus_uri=MILVUS_URI
)

@mcp.tool()
async def search_memory(query: str) -> str:
    """Search the agent's long-term memory for relevant past context, decisions, or user preferences."""
    results = await mem.search(query, top_k=3)
    if not results:
        return "No relevant memories found."
    context = "\n\n".join(f"- {m['content']}" for m in results)
    return f"Relevant memories:\n{context}"

@mcp.tool()
async def compact_memory() -> str:
    """Compact and summarize recent memories. Call this at the end of a session."""
    # Note: requires llm_provider setup for compacting, defaulting to openai if not set
    try:
        summary_path = await mem.compact(llm_provider="openai")
        return f"Memory compacted successfully to {summary_path}."
    except Exception as e:
        return f"Failed to compact memory: {e}"

@mcp.tool()
async def save_memory(content: str) -> str:
    """Save an important fact, user preference, or decision to long-term memory."""
    from datetime import date
    p = Path(MEMORY_DIR) / f"{date.today()}_notes.md"
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "a") as f:
        f.write(f"\n{content}\n")
    await mem.index()
    return "Memory saved and indexed."

def main():
    import threading
    
    # 1. Register with Robonix Control Plane
    channel = grpc.insecure_channel(os.environ.get("ROBONIX_SERVER", "localhost:50051"))
    stub = pb_grpc.RobonixRuntimeStub(channel)

    # Pick a port for FastMCP SSE
    port = int(os.environ.get("MEMSEARCH_PORT", "50105"))
    endpoint = f"http://127.0.0.1:{port}"

    metadata = {
        "endpoint": endpoint,
        "tools": [
            {
                "name": "search_memory",
                "description": "Search the agent's long-term memory for relevant past context, decisions, or user preferences.",
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "query": {"type": "string"}
                    },
                    "required": ["query"]
                }
            },
            {
                "name": "compact_memory",
                "description": "Compact and summarize recent memories. Call this at the end of a session.",
                "input_schema": {"type": "object", "properties": {}}
            },
            {
                "name": "save_memory",
                "description": "Save an important fact, user preference, or decision to long-term memory.",
                "input_schema": {
                    "type": "object",
                    "properties": {
                        "content": {"type": "string"}
                    },
                    "required": ["content"]
                }
            }
        ]
    }

    try:
        stub.RegisterNode(
            pb.RegisterNodeRequest(
                node_id="com.robonix.services.memsearch",
                namespace="robonix/prm/memsearch",
                kind="service",
                skill_md="# Memsearch MCP\nProvides memory operations."
            )
        )
        
        stub.DeclareInterface(
            pb.DeclareInterfaceRequest(
                node_id="com.robonix.services.memsearch",
                name="mcp_interface",
                supported_transports=["mcp"],
                metadata_json=json.dumps(metadata),
                listen_port=port,
            )
        )
        print(f"[memsearch-service] Registered node robonix/prm/memsearch with endpoint {endpoint}")
    except Exception as e:
        print(f"[memsearch-service] Warning: Failed to register with control plane: {e}")

    # Initialize memory index once
    asyncio.run(mem.index())

    # Start FastMCP server using streamable HTTP
    print(f"[memsearch-service] Starting MCP Streamable HTTP server on port {port}...")
    import uvicorn
    import logging
    # Suppress verbose gRPC / absl warnings from milvus-lite
    logging.getLogger("absl").setLevel(logging.ERROR)
    
    app = mcp.streamable_http_app()
    uvicorn.run(app, host="127.0.0.1", port=port, log_level="warning")

if __name__ == "__main__":
    main()
