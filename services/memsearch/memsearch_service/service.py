"""memsearch_service — long-term memory store + 3 MCP search/save/compact tools.

Indexes markdown notes under AGENT_MEMORY_DIR with milvus-lite (ONNX
embeddings). VLM credentials reused from pilot's env (VLM_BASE_URL /
VLM_API_KEY / VLM_MODEL) so memory doesn't need a separate API key.

Logging: this module writes to stdout/stderr only. `rbnx boot` captures
those into rbnx-boot/logs/system_memory.log automatically. No per-package
log file — keeps cross-component debugging in one place.
"""
from __future__ import annotations

import asyncio
import logging
import os
import platform
import sys
import traceback
from datetime import date
from pathlib import Path
from urllib.parse import urlsplit


def _redact_url(url: str | None) -> str:
    """Strip userinfo (`user:password@`) from a URL before logging it.
    Treats anything that doesn't parse as an opaque string and returns it
    unchanged — the goal is just to keep credentials out of the log when
    someone wrote `https://user:pw@host/v1` in their env."""
    if not url:
        return "(unset)"
    try:
        u = urlsplit(url)
    except ValueError:
        return url
    if not u.netloc or "@" not in u.netloc:
        return url
    host = u.hostname or ""
    netloc = f"{host}:{u.port}" if u.port else host
    return u._replace(netloc=netloc).geturl()

# ── 0. Logging setup: stdout only, prefixed with [memsearch] for grep-ability.
# We configure the root logger here BEFORE importing robonix_api / memsearch /
# milvus, otherwise their imports may install their own handlers and we lose
# control of the format. PYTHONUNBUFFERED=1 in start.sh ensures flush.
_LOG_LEVEL = os.environ.get("MEMSEARCH_LOG_LEVEL", "INFO").upper()
logging.basicConfig(
    level=getattr(logging, _LOG_LEVEL, logging.INFO),
    format="%(asctime)s %(levelname)s [memsearch] %(message)s",
    stream=sys.stdout,
    force=True,  # override anything inherited from imports
)
log = logging.getLogger("memsearch")

# Suppress verbose gRPC / absl warnings from milvus-lite. These dwarf the
# real memsearch logs and make the file unreadable.
os.environ.setdefault("GRPC_VERBOSITY", "ERROR")
os.environ.setdefault("GLOG_minloglevel", "2")
logging.getLogger("absl").setLevel(logging.ERROR)


def _log_environment() -> None:
    """Print a compact environment summary at boot. Many of the reported
    "memsearch fails to start" issues come down to: wrong Python ABI in the
    venv, missing ONNX runtime wheel for the arch, missing AGENT_MEMORY_DIR
    permissions, or an unset PYTHONPATH. Knowing exactly what the process
    sees is the difference between 5 minutes and 5 days of debugging."""
    log.info("python   : %s (%s)", sys.version.replace("\n", " "), sys.executable)
    log.info("platform : %s / %s / %s",
             platform.system(), platform.machine(), platform.release())
    log.info("cwd      : %s", os.getcwd())
    log.info("pythonpath:")
    for p in (os.environ.get("PYTHONPATH") or "").split(":"):
        if p:
            log.info("  - %s%s", p, "" if Path(p).exists() else "  (MISSING)")
    log.info("env (non-secret):")
    _URL_KEYS = {"VLM_BASE_URL", "OPENAI_BASE_URL", "ROBONIX_ATLAS"}
    for k in ("AGENT_MEMORY_DIR", "AGENT_MILVUS_URI",
              "MEMSEARCH_LOG_LEVEL",
              "VLM_BASE_URL", "VLM_MODEL", "OPENAI_BASE_URL", "OPENAI_MODEL",
              "ROBONIX_ATLAS"):
        v = os.environ.get(k)
        if v and k in _URL_KEYS:
            v = _redact_url(v)
        log.info("  %-22s = %s", k, v if v else "(unset)")
    log.info("vlm api key set: %s",
             "yes" if (os.environ.get("VLM_API_KEY") or os.environ.get("OPENAI_API_KEY"))
             else "NO — compact() will refuse")


# ── 1. Imports. Keep them after the logger so any import-time error has
# the formatter already installed and the stack trace ends up in the
# unified log instead of being printed to a raw stderr that nobody reads.
log.info("phase 1/4: importing robonix_api + memsearch backend")
try:
    from robonix_api import Service, Ok, Err, Deferred  # noqa: E402,F401
    from std_msgs_mcp import Empty, String  # noqa: E402
    from memsearch import MemSearch  # noqa: E402
except Exception:
    log.error("import failed — see the stack trace below")
    log.error("common causes:")
    log.error("  - venv missing or wrong Python version "
              "(rerun: rbnx build -p services/memsearch with RBNX_BUILD_CLEAN=1)")
    log.error("  - codegen dirs not on PYTHONPATH "
              "(check rbnx-build/codegen/{proto_gen,robonix_mcp_types} exist)")
    log.error("  - milvus-lite / onnxruntime wheel not available for this arch "
              "(Jetson aarch64 + Python 3.12 occasionally needs manual install)")
    traceback.print_exc(file=sys.stdout)
    sys.stdout.flush()
    raise


# ── 2. Service object + paths. Paths are resolved to absolute form so the
# log makes the actual disk location obvious; relative paths are a common
# source of "where did the index go?" confusion.
memory = Service(id="memory", namespace="robonix/service/memory")

MEMORY_DIR = str(Path(os.environ.get("AGENT_MEMORY_DIR", "./agent_memory")).resolve())
MILVUS_URI = os.environ.get("AGENT_MILVUS_URI", "./agent_milvus.db")
# Resolve only filesystem-style milvus URIs (skip remote `host:port` forms).
if "/" in MILVUS_URI or MILVUS_URI.endswith(".db"):
    MILVUS_URI = str(Path(MILVUS_URI).resolve())


_log_environment()
log.info("phase 2/4: resolved paths")
log.info("  memory_dir = %s%s", MEMORY_DIR,
         "" if Path(MEMORY_DIR).exists() else "  (will be created)")
log.info("  milvus_uri = %s", MILVUS_URI)


# ── 3. Backend construction. This is where most user-reported "startup
# fails" issues actually originate (milvus-lite native binary, embedding
# model download, write permission on milvus_uri parent). Capture the
# failure with enough context for someone reading the log to act.
log.info("phase 3/4: constructing MemSearch (embedding=onnx, milvus_lite)")
try:
    mem = MemSearch(
        paths=[MEMORY_DIR],
        embedding_provider="onnx",
        milvus_uri=MILVUS_URI,
    )
except Exception as e:
    log.error("MemSearch construction failed: %s: %s", type(e).__name__, e)
    log.error("common causes:")
    log.error("  - milvus_lite cannot create / open %s "
              "(check parent dir exists and is writable)", MILVUS_URI)
    log.error("  - onnxruntime wheel incompatible "
              "(arch=%s python=%s.%s — try `pip install onnxruntime` in the venv)",
              platform.machine(), *sys.version_info[:2])
    log.error("  - embedding model download blocked "
              "(first run needs network egress for HuggingFace; set HF_ENDPOINT "
              "or pre-stage models if behind a firewall)")
    traceback.print_exc(file=sys.stdout)
    sys.stdout.flush()
    raise

log.info("phase 4/4: registering MCP tools + awaiting Driver(CMD_INIT)")


@memory.mcp("robonix/service/memory/search")
async def search(msg: String) -> String:
    """Search the agent's long-term memory for relevant past context, decisions, or user preferences.
    Contract: robonix/service/memory/search."""
    # Log shape only at INFO; raw query text only at DEBUG to keep user
    # content out of the default log.
    log.info("search (%d chars)", len(msg.data))
    log.debug("search query: %r", msg.data[:80])
    try:
        results = await mem.search(msg.data, top_k=2)
    except Exception as e:
        log.warning("search failed (returning empty): %s: %s", type(e).__name__, e)
        return String(data="No relevant memories found (search unavailable).")
    if not results:
        return String(data="No relevant memories found.")
    context = "\n\n".join(f"- {m['content']}" for m in results)
    return String(data=f"Relevant memories:\n{context}")


@memory.mcp("robonix/service/memory/save")
async def save(msg: String) -> String:
    """Save an important fact, user preference, or decision to long-term memory.
    Contract: robonix/service/memory/save."""
    p = Path(MEMORY_DIR) / f"{date.today()}_notes.md"
    log.info("save → %s (%d chars)", p, len(msg.data))
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "a") as f:
        f.write(f"\n{msg.data}\n")
    try:
        await mem.index()
    except Exception as e:
        log.warning("re-index after save failed: %s: %s", type(e).__name__, e)
    return String(data="Memory saved and indexed.")


@memory.mcp("robonix/service/memory/compact")
async def compact(msg: Empty) -> String:
    """Compact and summarize recent memories. Call this at the end of a session.
    Returns std_msgs/String JSON. Contract: robonix/service/memory/compact.

    Reuses pilot's OpenAI-compatible LLM endpoint so memory doesn't need a
    separate API key. Reads VLM_BASE_URL / VLM_API_KEY / VLM_MODEL (or the
    OPENAI_* equivalents as fallback)."""
    _ = msg
    base_url = os.environ.get("VLM_BASE_URL") or os.environ.get("OPENAI_BASE_URL")
    api_key = os.environ.get("VLM_API_KEY") or os.environ.get("OPENAI_API_KEY")
    model = os.environ.get("VLM_MODEL") or os.environ.get("OPENAI_MODEL") or "gpt-5.5"
    log.info("compact (model=%s, base_url=%s)", model, _redact_url(base_url))
    if not api_key:
        log.warning("compact: no LLM key set (VLM_API_KEY / OPENAI_API_KEY)")
        return String(data="compact: no LLM credentials available. "
                           "Set VLM_API_KEY in the deploy manifest's system.memory env block.")
    try:
        summary_path = await mem.compact(
            llm_provider="openai", model=model,
            base_url=base_url, api_key=api_key,
        )
        log.info("compact done → %s", summary_path)
        return String(data=f"Memory compacted successfully to {summary_path}.")
    except Exception as e:
        # Keep details in the log; return only the exception type to the MCP
        # caller so any base_url / api_key fragments that the exception
        # string might have captured don't get echoed back over the wire.
        log.error("compact failed: %s: %s", type(e).__name__, e)
        traceback.print_exc(file=sys.stdout)
        sys.stdout.flush()
        return String(data=f"Failed to compact memory ({type(e).__name__}).")


@memory.on_init
def init(cfg):
    """Boot-time index of the corpus. Empty corpus is fine — index() returns
    quickly with no docs. Don't fail Init on indexing errors; search/save
    handlers degrade gracefully."""
    _ = cfg
    log.info("on_init: building initial index from %s", MEMORY_DIR)
    try:
        asyncio.run(mem.index())
        log.info("on_init: index built")
    except Exception as e:
        # Empty corpus or transient I/O — degrade rather than crash boot.
        log.warning("on_init: initial index failed (empty corpus?): %s: %s",
                    type(e).__name__, e)
    return Ok()


def main() -> int:
    log.info("memsearch_service ready; entering memory.run()")
    memory.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
