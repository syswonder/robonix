#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
set +u
source "${ROS_SETUP:-/opt/ros/${ROS_DISTRO:-humble}/setup.bash}"
source "$PKG/rbnx-build/codegen/ros2_idl/install/setup.bash"
set -u
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:$(rbnx path robonix-api):${PYTHONPATH:-}"
if [[ -n "${OUTDOOR_DEPLOY_ROOT:-}" ]]; then
  export PYTHONPATH="$OUTDOOR_DEPLOY_ROOT:$OUTDOOR_DEPLOY_ROOT/packages/service-map-outdoor-rbnx:$OUTDOOR_DEPLOY_ROOT/packages/service-navigation-outdoor-rbnx:$PYTHONPATH"
fi
export SCENE_GRAPH_ENABLED=false SCENE_OBJECT_WATCHDOG=0
export SCENE_PROFILE=annotate SCENE_WEB_VIEWER=off SCENE_OBJECT_MEMORY_ENABLED=false
export SCENE_ANNOTATIONS_DIR="$PKG/rbnx-build/data/annotations"
export SCENE_MAP_META_DIR="$PKG/rbnx-build/data/maps"
export SCENE_GRAPH_CACHE_DIR="$PKG/rbnx-build/data/graph"
export ROS_LOG_DIR="$PKG/rbnx-build/ros_logs"
export ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-127.0.0.1}"
mkdir -p "$ROS_LOG_DIR" "$SCENE_ANNOTATIONS_DIR" "$SCENE_MAP_META_DIR" "$SCENE_GRAPH_CACHE_DIR"
exec "$PKG/rbnx-build/annotate-venv/bin/python" -m scene_service.service
