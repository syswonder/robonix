#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
FLAGS=()
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)

# rbnx codegen (generates proto stubs from ROS IDL + contracts)
rbnx codegen -p "$PKG" "${FLAGS[@]}"

echo "[build] done."
