#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
CLEAN="${RBNX_BUILD_CLEAN:-}"
# The simulator image supplies sensor_msgs and the other standard ROS 2
# interfaces used by this driver.  Build only the Robonix proto/MCP bindings.
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[tiago_camera/build] rbnx codegen ${FLAGS[*]}"
"$PKG/../../scripts/run_python_codegen.sh" "$PKG" "${FLAGS[@]}"
echo "[tiago_camera/build] done."
