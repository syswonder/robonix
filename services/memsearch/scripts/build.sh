#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Memory service build phase.
#
# Robonix package convention: every Python package gets a uv-managed venv
# AND a per-package codegen output AND a runtime data dir, all under
# `rbnx-build/` at the package root. That dir is rbnx-cli's existing
# scratch convention — we just put everything else generated alongside
# rbnx-cli's own `ws/` subtree so there's exactly one gitignored umbrella.
#
# Layout under rbnx-build/:
#   rbnx-build/venv/                   per-package Python venv (uv)
#   rbnx-build/codegen/proto_gen/      ROS IDL → .proto + grpc stubs
#   rbnx-build/codegen/robonix_mcp_types/  MCP dataclasses
#   rbnx-build/ws/install/setup.bash   rbnx-cli's PYTHONPATH stub (existing)
#   rbnx-build/data/                   runtime data (memsearch.log, milvus db, …)
#
# Rule for ALL packages: build phase does every download / model check /
# dependency probe. Runtime (`start:`) only activates the venv and serves;
# no network egress, no first-request stalls.

set -euo pipefail
# Use TUNA mirror for pip / uv when on a domestic (CN) network. Override via env.
: "${UV_INDEX_URL:=https://pypi.tuna.tsinghua.edu.cn/simple}"
: "${PIP_INDEX_URL:=https://pypi.tuna.tsinghua.edu.cn/simple}"
export UV_INDEX_URL PIP_INDEX_URL
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

BUILD="rbnx-build"
VENV="$BUILD/venv"
CLEAN="${RBNX_BUILD_CLEAN:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"

# ── 1. uv venv ──────────────────────────────────────────────────────────────
if ! command -v uv >/dev/null 2>&1; then
    echo "[build] error: 'uv' not found on PATH. Install: https://docs.astral.sh/uv/" >&2
    exit 1
fi

if [[ ! -d "$VENV" ]]; then
    echo "[build] uv venv → $VENV"
    uv venv "$VENV"
fi

# ── 2. uv sync (deps from pyproject.toml + uv.lock into the venv) ──────────
echo "[build] uv sync (pyproject.toml → $VENV)"
VIRTUAL_ENV="$PKG/$VENV" uv sync --active --no-managed-python

# ── 3. Codegen (.proto + grpc stubs + MCP dataclasses → rbnx-build/codegen/) ─
FLAGS=(--mcp)
# The complete build directory was already removed above. A second clean here
# would delete the freshly synchronized venv before model warm-up.
echo "[build] rbnx codegen ${FLAGS[*]}"
PATH="$PKG/$VENV/bin:$PATH" rbnx codegen -p "$PKG" "${FLAGS[@]}"

CODEGEN_PYTHONPATH="$PKG/$BUILD/codegen/proto_gen:$PKG/$BUILD/codegen/robonix_mcp_types"
PYTHONPATH="$CODEGEN_PYTHONPATH:${PYTHONPATH:-}" "$VENV/bin/python" - <<'PY'
import robonix_contracts_pb2_grpc
import memory_mcp

assert hasattr(
    robonix_contracts_pb2_grpc,
    "RobonixServiceMemoryDriverServicer",
)
print("[build] Memory lifecycle servicer and MCP imports OK")
PY

# ── 4. Warm the ONNX embedding model at BUILD time ──────────────────────────
# The service constructs MemSearch(embedding_provider="onnx") at start, which
# downloads the embedding model (gpahal/bge-m3-onnx-int8) from HuggingFace on
# first use. If that download happens at `rbnx start`, a cold/slow network
# blows past boot's 60s register window and the service "explodes" with an
# opaque timeout (issue #113). Per this script's own rule — build does every
# download, runtime only serves — pull the model NOW, into the shared HF cache,
# by building a throwaway one-doc index exactly like the service will at boot.
echo "[build] warming ONNX embedding model (so start needs no network)"
# huggingface_hub needs repository commit and ETag metadata. Some mirrors only
# redirect downloads to huggingface.co and omit that metadata, so use the
# library's official endpoint by default. Deployments may still override
# HF_ENDPOINT explicitly when their mirror implements the full Hub API.
: "${HF_ENDPOINT:=https://huggingface.co}"
export HF_ENDPOINT
warmed=0
for attempt in 1 2 3; do
    if "$VENV/bin/python" - <<'PY'
import asyncio, os, tempfile
from memsearch_service.onnx_compat import configure_onnxruntime
configure_onnxruntime()
from memsearch import MemSearch
with tempfile.TemporaryDirectory(prefix="memsearch_warm_") as d:
    with open(os.path.join(d, "warm.md"), "w") as f:
        f.write("# warm\nPrefetch the embedding model at build time.\n")
    m = MemSearch(paths=[d], embedding_provider="onnx", milvus_uri=os.path.join(d, "warm.db"))
    asyncio.run(m.index())
print("[build]   embedding model cached OK")
PY
    then
        warmed=1
        break
    fi
    echo "[build] ONNX warm attempt $attempt/3 failed" >&2
    [[ "$attempt" == "3" ]] || sleep "$((attempt * 2))"
done
if [[ "$warmed" != "1" ]]; then
    echo "[build] error: ONNX embedding-model warm failed" >&2
    exit 1
fi

echo "[build] done."
