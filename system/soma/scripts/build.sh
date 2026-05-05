#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Soma service build phase. Same convention as system/memory — uv venv
# + rbnx codegen — except soma's runtime deps are tiny (urdf_parser_py
# + numpy + a ROS msg/srv stack already inherited from base ROS humble).

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

# ── 2. uv sync (deps from pyproject.toml) ──────────────────────────────────
echo "[build] uv sync (pyproject.toml → $VENV)"
VIRTUAL_ENV="$PKG/$VENV" uv sync --active --no-managed-python

# ── 3. Codegen (rbnx mcp types + grpc stubs) ────────────────────────────────
FLAGS=(--mcp --out-dir "$GEN")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

# Sentinel for `rbnx boot` prerequisite check.
mkdir -p "$BUILD"
touch "$BUILD/.rbnx-built"

echo "[build] done."
