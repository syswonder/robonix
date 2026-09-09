#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
command -v uv >/dev/null
command -v rbnx >/dev/null
if [[ ! -d rbnx-build/venv ]]; then
    uv venv rbnx-build/venv
fi
VIRTUAL_ENV="$PKG/rbnx-build/venv" uv sync --active --no-managed-python
RBNX_CODEGEN_PYTHON="$PKG/rbnx-build/venv/bin/python" \
    PATH="$PKG/rbnx-build/venv/bin:$PATH" \
    rbnx codegen -p "$PKG" --mcp
PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}" \
    rbnx-build/venv/bin/python -c 'from verifier_mcp import Verify_Request, Verify_Response'