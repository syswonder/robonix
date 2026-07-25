#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
if command -v rbnx >/dev/null 2>&1; then
    # --mcp emits robonix_mcp_types/; --ros2 emits rbnx-build/codegen/ros2_idl
    # (canonical ROS 2 messages) so nav's rclpy types are Robonix's.
    FLAGS=(--mcp --ros2)
    [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)
    bash "$PKG/../../scripts/run_python_codegen.sh" "$PKG" "${FLAGS[@]}"

    # Build the ROS 2 overlay inside the sim container (host has no ROS 2).
    if docker ps --format '{{.Names}}' | grep -qx robonix_tiago_sim; then
      _IDL="/robonix_pkgs/$(basename "$(dirname "$PKG")")/$(basename "$PKG")/rbnx-build/codegen/ros2_idl"
      docker exec robonix_tiago_sim bash -lc "source /opt/ros/humble/setup.bash && cd $_IDL && colcon build"
    else
      echo "[simple_nav] sim container down — ROS 2 overlay not built; run sim/start.sh then rebuild"
    fi
fi
echo "[simple_nav] build done."
