"""memsearch_service — long-term memory store + 3 MCP search/save/compact tools.

Indexes markdown notes under AGENT_MEMORY_DIR with milvus-lite (ONNX
embeddings). VLM credentials reused from pilot's env (VLM_BASE_URL /
VLM_API_KEY / VLM_MODEL) so memory doesn't need a separate API key.

Logging: routed entirely through Scribe via
`scribe_logger.install_stdlib_bridge("memory")`, so the full startup trace —
phase markers, env summary, error causes, stack traces — shows up under
`rbnx logs -t memory`, and the package owns no log file or stdout sink of its
own. Without that bridge a startup that stalls in phase 3 (embedding-model
download / milvus-lite init) looked like total silence after "memory service
starting" (issue #113).
"""
from __future__ import annotations

import asyncio
import logging
from robonix_api import scribe_logger
import os
import platform
import sys
import traceback
from datetime import date
from pathlib import Path
from typing import Any
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

# ── 0. Logging setup: route everything through Scribe.
# Install the shared stdlib-logging → Scribe bridge on the root logger BEFORE
# importing robonix_api / memsearch / milvus, so their imports' logging is
# captured too. The bridge drops any stdout/file handlers and forwards every
# record to Scribe under the "memory" tag, so `rbnx logs -t memory` sees the
# whole startup trace and the package owns no log file of its own.
_LOG_LEVEL = os.environ.get("MEMSEARCH_LOG_LEVEL", "INFO").upper()
scribe_logger.install_stdlib_bridge(
    "memory", level=getattr(logging, _LOG_LEVEL, logging.INFO)
)
log = logging.getLogger("memsearch")

# Suppress verbose gRPC / absl warnings from milvus-lite. These dwarf the
# real memsearch logs and make the file unreadable.
os.environ.setdefault("GRPC_VERBOSITY", "ERROR")
os.environ.setdefault("GLOG_minloglevel", "2")
logging.getLogger("absl").setLevel(logging.ERROR)

# Transitional: also emit key lifecycle events through Scribe so the
# unified `rbnx logs` tool can see them alongside other components.
scribe_logger.info("memory", "memory service starting")

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
    from memsearch_service.onnx_compat import configure_onnxruntime  # noqa: E402
    from robonix_api import Service, Ok, Err  # noqa: E402
    from std_msgs_mcp import Empty, String  # noqa: E402
except Exception:
    log.error("import failed — see the stack trace below")
    log.error("common causes:")
    log.error("  - venv missing or wrong Python version "
              "(rerun: rbnx build -p services/memsearch with RBNX_BUILD_CLEAN=1)")
    log.error("  - codegen dirs not on PYTHONPATH "
              "(check rbnx-build/codegen/{proto_gen,robonix_mcp_types} exist)")
    log.error("  - milvus-lite / onnxruntime wheel not available for this arch "
              "(Jetson aarch64 + Python 3.12 occasionally needs manual install)")
    log.error("traceback:\n%s", traceback.format_exc())
    raise


# ── 2. Service object + lifecycle-owned backend state.
memory = Service(id="memory", namespace="robonix/service/memory")
_log_environment()
log.info("phase 2/4: registering MCP tools + awaiting Driver(CMD_INIT)")

mem: Any | None = None
MEMORY_DIR: str | None = None
MILVUS_URI: str | None = None
_APPLIED_CONFIG: tuple[str, str, int] | None = None


def _nonempty_string(cfg: dict, key: str, env_key: str, default: str) -> str:
    """Resolve one string config value with an environment compatibility fallback."""
    value = cfg[key] if key in cfg else os.environ.get(env_key, default)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a non-empty string")
    return value.strip()


def _resolve_milvus_uri(value: str) -> str:
    """Resolve filesystem stores while preserving remote Milvus endpoints."""
    if "://" in value:
        return value
    if ":" in value and "/" not in value and not value.endswith(".db"):
        return value
    return str(Path(value).expanduser().resolve())


def _resolve_config(cfg: dict) -> tuple[str, str, int]:
    """Normalize Driver config; explicit config wins over legacy environment values."""
    memory_dir = str(
        Path(
            _nonempty_string(cfg, "memory_dir", "AGENT_MEMORY_DIR", "./agent_memory")
        )
        .expanduser()
        .resolve()
    )
    milvus_uri = _resolve_milvus_uri(
        _nonempty_string(cfg, "milvus_uri", "AGENT_MILVUS_URI", "./agent_milvus.db")
    )
    raw_threads = cfg.get(
        "onnx_threads", os.environ.get("MEMSEARCH_ONNX_THREADS", "1")
    )
    try:
        onnx_threads = int(raw_threads)
    except (TypeError, ValueError) as exc:
        raise ValueError("onnx_threads must be an integer") from exc
    if onnx_threads < 1:
        raise ValueError("onnx_threads must be at least 1")
    return memory_dir, milvus_uri, onnx_threads


def _require_backend() -> Any:
    """Return the lifecycle-initialized backend or fail without hidden lazy init."""
    if mem is None:
        raise RuntimeError("memory backend is not initialized")
    return mem


@memory.mcp("robonix/service/memory/search")
async def search(msg: String) -> String:
    """Search the agent's long-term memory for relevant past context, decisions, or user preferences.
    Contract: robonix/service/memory/search."""
    # Log shape only at INFO; raw query text only at DEBUG to keep user
    # content out of the default log.
    log.info("search (%d chars)", len(msg.data))
    log.debug("search query: %r", msg.data[:80])
    try:
        results = await _require_backend().search(msg.data, top_k=2)
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
    backend = _require_backend()
    if MEMORY_DIR is None:
        raise RuntimeError("memory directory is not initialized")
    p = Path(MEMORY_DIR) / f"{date.today()}_notes.md"
    log.info("save → %s (%d chars)", p, len(msg.data))
    p.parent.mkdir(parents=True, exist_ok=True)
    with open(p, "a") as f:
        f.write(f"\n{msg.data}\n")
    try:
        await backend.index()
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
        summary_path = await _require_backend().compact(
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
        log.error("traceback:\n%s", traceback.format_exc())
        return String(data=f"Failed to compact memory ({type(e).__name__}).")


@memory.on_init
def init(cfg: dict):
    """Build and index the configured backend before the provider can activate."""
    global mem, MEMORY_DIR, MILVUS_URI, _APPLIED_CONFIG
    try:
        resolved = _resolve_config(cfg)
        if mem is not None:
            if resolved == _APPLIED_CONFIG:
                log.info("on_init: already initialized with the same configuration")
                return Ok()
            return Err("memory is already initialized with different configuration")

        memory_dir, milvus_uri, onnx_threads = resolved
        Path(memory_dir).mkdir(parents=True, exist_ok=True)
        if "://" not in milvus_uri and not (
            ":" in milvus_uri and "/" not in milvus_uri and not milvus_uri.endswith(".db")
        ):
            Path(milvus_uri).parent.mkdir(parents=True, exist_ok=True)

        os.environ["MEMSEARCH_ONNX_THREADS"] = str(onnx_threads)
        configure_onnxruntime()
        from memsearch import MemSearch  # noqa: PLC0415

        log.info("phase 3/4: constructing MemSearch (embedding=onnx, milvus_lite)")
        log.info("  memory_dir = %s", memory_dir)
        log.info("  milvus_uri = %s", milvus_uri)
        backend = MemSearch(
            paths=[memory_dir],
            embedding_provider="onnx",
            milvus_uri=milvus_uri,
        )
        log.info("phase 4/4: building initial index")
        asyncio.run(backend.index())

        mem = backend
        MEMORY_DIR = memory_dir
        MILVUS_URI = milvus_uri
        _APPLIED_CONFIG = resolved
        log.info("on_init: index built")
        return Ok()
    except Exception as exc:
        mem = None
        MEMORY_DIR = None
        MILVUS_URI = None
        _APPLIED_CONFIG = None
        log.error("on_init failed: %s: %s", type(exc).__name__, exc)
        log.error("traceback:\n%s", traceback.format_exc())
        return Err(f"memory init failed: {type(exc).__name__}: {exc}")


def main() -> int:
    log.info("memsearch_service ready; entering memory.run()")
    memory.run()
    return 0


if __name__ == "__main__":
    sys.exit(main())
