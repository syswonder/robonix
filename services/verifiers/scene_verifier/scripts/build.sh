#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
VENV="$PKG/rbnx-build/venv"
command -v uv >/dev/null
command -v rbnx >/dev/null
if [[ ! -d "$VENV" ]]; then
    uv venv "$VENV"
fi
VIRTUAL_ENV="$VENV" uv sync --active --no-managed-python
RBNX_CODEGEN_PYTHON="$VENV/bin/python" \
    PATH="$VENV/bin:$PATH" \
    rbnx codegen -p "$PKG" --mcp
PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}" \
    "$VENV/bin/python" -c 'from verifier_mcp import Verify_Request, Verify_Response'