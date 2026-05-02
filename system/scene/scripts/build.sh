#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scene service build phase. Mirrors system/memory/scripts/build.sh.
#
# Layout under rbnx-build/ (matches the rest of system/*):
#   rbnx-build/venv/                       per-package Python venv (uv)
#   rbnx-build/codegen/proto_gen/          ROS IDL → .proto + grpc stubs
#   rbnx-build/codegen/robonix_mcp_types/  MCP dataclasses (PoR for FastMCP)
#   rbnx-build/data/                       runtime data (scene.log, …)
#
# Build-phase rule for ALL robonix packages: every download / model
# warm-up / dep probe happens here. Runtime (`start:` body) only
# activates the venv and serves; no network egress at runtime.

set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

BUILD="rbnx-build"
VENV="$BUILD/venv"
GEN="$BUILD/codegen"
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
FLAGS=(--mcp --out-dir "$GEN")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

echo "[build] done."
