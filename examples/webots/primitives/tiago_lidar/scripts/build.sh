#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
CLEAN="${RBNX_BUILD_CLEAN:-}"
# --ros2 also emits rbnx-build/codegen/ros2_idl (canonical ROS 2 messages).
FLAGS=(--mcp --ros2)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[tiago_lidar/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

# Build the ROS 2 overlay inside the sim container (host has no ROS 2).
if docker ps --format '{{.Names}}' | grep -qx robonix_tiago_sim; then
  _IDL="/robonix_pkgs/$(basename "$(dirname "$PKG")")/$(basename "$PKG")/rbnx-build/codegen/ros2_idl"
  docker exec robonix_tiago_sim bash -lc "source /opt/ros/humble/setup.bash && cd $_IDL && colcon build"
else
  echo "[tiago_lidar/build] sim container down — ROS 2 overlay not built; run sim/start.sh then rebuild"
fi
echo "[tiago_lidar/build] done."
