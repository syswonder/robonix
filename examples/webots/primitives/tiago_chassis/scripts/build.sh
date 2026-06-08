#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# tiago_chassis build — codegen only. The driver runs inside the sim
# docker container (which already has rclpy + numpy + fastmcp + grpc),
# so no per-package venv. The codegen output (rbnx-build/codegen/) is
# bind-mounted into the container via sim/compose.yaml's ../primitives
# volume; the driver picks it up via _ensure_proto_gen / _ensure_mcp_types.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

CLEAN="${RBNX_BUILD_CLEAN:-}"
# --ros2 also emits rbnx-build/codegen/ros2_idl (canonical ROS 2 message
# overlay) so the driver's rclpy types are Robonix's, not the distro's.
FLAGS=(--mcp --ros2)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[tiago_chassis/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

# Build the ROS 2 overlay inside the sim container (host has no ROS 2);
# rbnx-build/ is bind-mounted into the container at the same package path.
if docker ps --format '{{.Names}}' | grep -qx robonix_tiago_sim; then
  _IDL="/robonix_pkgs/$(basename "$(dirname "$PKG")")/$(basename "$PKG")/rbnx-build/codegen/ros2_idl"
  echo "[tiago_chassis/build] colcon build ros2_idl in sim container"
  docker exec robonix_tiago_sim bash -lc "source /opt/ros/humble/setup.bash && cd $_IDL && colcon build"
else
  echo "[tiago_chassis/build] sim container down — ROS 2 overlay not built; run sim/start.sh then rebuild"
fi
echo "[tiago_chassis/build] done."
