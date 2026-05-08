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
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

echo "[build] done."
