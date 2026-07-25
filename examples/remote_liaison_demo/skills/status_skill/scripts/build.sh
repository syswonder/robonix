#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
BUILD="$PKG/rbnx-build"
VENV="$BUILD/venv"
ROBONIX_ROOT="$(cd "$PKG/../../../.." && pwd)"

if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"
uv venv --allow-existing "$VENV"
uv pip install --python "$VENV/bin/python" --quiet "$ROBONIX_ROOT/pylib/robonix-api"
RBNX_CODEGEN_PYTHON="$VENV/bin/python" \
    PATH="$VENV/bin:$PATH" \
    rbnx codegen -p "$PKG" --mcp
PYTHONPATH="$PKG:$BUILD/codegen/proto_gen:$BUILD/codegen/robonix_mcp_types" \
    "$VENV/bin/python" -c 'import memory_mcp, std_msgs_mcp'
echo "[status_skill/build] done."
