#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scribe Mem service build phase.
#
# Layout under rbnx-build/:
#   rbnx-build/venv/                   per-package Python venv (uv)
#   rbnx-build/codegen/proto_gen/      ROS IDL → .proto + grpc stubs
#   rbnx-build/codegen/robonix_mcp_types/  MCP dataclasses
#   rbnx-build/data/                   runtime data (memory.log, graph_store.json, …)

set -euo pipefail
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

# ── 1. Locate uv (search common install paths) ──────────────────────────
_UV=""
for _candidate in \
    "$(command -v uv 2>/dev/null || true)" \
    "$HOME/.cargo/bin/uv" \
    "$HOME/.local/bin/uv" \
    /usr/local/bin/uv \
; do
    if [[ -n "$_candidate" && -x "$_candidate" ]]; then
        _UV="$_candidate"
        break
    fi
done
if [[ -z "$_UV" ]]; then
    echo "[build] error: 'uv' not found." >&2
    echo "[build] Checked: PATH, ~/.cargo/bin/uv, ~/.local/bin/uv, /usr/local/bin/uv." >&2
    echo "[build] Install: https://docs.astral.sh/uv/getting-started/installation" >&2
    echo "[build] Or:   conda activate env_robonix && rbnx boot" >&2
    exit 1
fi
echo "[build] uv → $_UV ($($_UV --version 2>/dev/null || echo unknown))"

# ── 2. uv venv ──────────────────────────────────────────────────────────
if [[ ! -d "$VENV" ]]; then
    echo "[build] uv venv → $VENV"
    "$_UV" venv "$VENV"
fi

# ── 3. uv sync (deps from pyproject.toml into the venv) ──────────────────
echo "[build] uv sync (pyproject.toml → $VENV)"
VIRTUAL_ENV="$PKG/$VENV" "$_UV" sync --active --no-managed-python

# ── 4. Codegen (.proto + grpc stubs + MCP dataclasses → rbnx-build/codegen/) ─
FLAGS=(--mcp)
# The complete build directory was already removed above. Passing --clean here
# would delete the freshly synchronized runtime venv.
echo "[build] rbnx codegen ${FLAGS[*]}"
RBNX_CODEGEN_PYTHON="$PKG/$VENV/bin/python" \
    PATH="$PKG/$VENV/bin:$PATH" \
    rbnx codegen -p "$PKG" "${FLAGS[@]}"

CODEGEN_PYTHONPATH="$PKG/$BUILD/codegen/proto_gen:$PKG/$BUILD/codegen/robonix_mcp_types"
PYTHONPATH="$CODEGEN_PYTHONPATH:${PYTHONPATH:-}" "$VENV/bin/python" - <<'PY'
import std_msgs_mcp

print("[build] generated Memory MCP imports OK")
PY

echo "[build] done."
