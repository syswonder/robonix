#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Codegen is done by `rbnx codegen`. Run `rbnx setup` once from the robonix
# source root first.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

FLAGS=(--mcp)
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)

rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[build] done."
