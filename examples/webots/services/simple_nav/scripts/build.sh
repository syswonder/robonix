#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
if command -v rbnx >/dev/null 2>&1; then
    # --mcp emits robonix_mcp_types/ alongside proto_gen/ since simple_nav
    # uses `@cap.mcp(...)`.
    FLAGS=(--mcp)
    [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
    rbnx codegen -p "$PKG" "${FLAGS[@]}"
fi
echo "[simple_nav] build done."
