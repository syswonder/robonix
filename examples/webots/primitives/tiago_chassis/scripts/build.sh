#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# tiago_chassis build — codegen only. The driver runs inside the simulator
# container, which already provides ROS 2 and the standard message packages.
# Package builds must not depend on or mutate a running simulator container.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[tiago_chassis/build] rbnx codegen ${FLAGS[*]}"
"$PKG/../../scripts/run_python_codegen.sh" "$PKG" "${FLAGS[@]}"
echo "[tiago_chassis/build] done."
