#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# All codegen is done by `rbnx codegen`. Run `rbnx setup` once from the
# robonix source root first so paths resolve regardless of where this
# package lives on disk.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

FLAGS=()
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)

rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[build] done."
