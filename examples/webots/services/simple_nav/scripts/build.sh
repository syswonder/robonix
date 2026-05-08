#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
if command -v rbnx >/dev/null 2>&1; then
    # `--out-dir rbnx-build/codegen` so proto_gen/ + robonix_mcp_types/
    # land under rbnx-build/ (which is gitignored). Without it the
    # codegen default drops them straight at $PKG, polluting the
    # package root with hundreds of generated *.py files.
    # `--mcp` so robonix_mcp_types/<ns>_mcp.py gets generated; simple_nav
    # uses `@cap.mcp(...)` so the MCP-typed dataclasses are required.
    FLAGS=(--mcp --out-dir "$PKG/rbnx-build/codegen")
    [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
    rbnx codegen -p "$PKG" "${FLAGS[@]}"
fi
echo "[simple_nav] build done."
