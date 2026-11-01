#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# scene native start — run scene_service directly on the host (no docker).
#
# Used on Jetson hosts with ROS 2 and the TRT/ORT engine runtime.
# Selected by the jetson-native manifest.
#
# Unlike the docker path this does NOT force a specific RMW / SHM profile:
# native scene must share whatever RMW the other host nodes use so it actually
# sees the camera / pointcloud topics they publish.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

if [[ -f "$PKG/rbnx-build/native-env.sh" ]]; then
    source "$PKG/rbnx-build/native-env.sh"
    "$SCENE_NATIVE_PYTHON" "$PKG/scripts/prepare_native.py" --check
fi
PY="${SCENE_NATIVE_PYTHON:-python3}"


ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

# Codegen stubs (proto_gen + robonix_mcp_types) + robonix-api + the package.
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:$PKG:${PYTHONPATH:-}"
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PYTHONPATH"
fi

export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-$PKG/rbnx-build/data/ultralytics}"

W="$PKG/docker/_weights"
export SCENE_YOLO_WORLD_WEIGHTS="${SCENE_YOLO_WORLD_WEIGHTS:-$W/yolov8l-world.pt}"
export SCENE_MOBILE_SAM_WEIGHTS="${SCENE_MOBILE_SAM_WEIGHTS:-$W/mobile_sam.pt}"
export SCENE_CLIP_MODEL="${SCENE_CLIP_MODEL:-ViT-B-32}"
export SCENE_CLIP_PRETRAINED="${SCENE_CLIP_PRETRAINED:-$W/open_clip_pytorch_model.bin}"
export SCENE_ONNX_DIR="${SCENE_ONNX_DIR:-$W/onnx}"
export SCENE_ENGINE_CACHE_DIR="${SCENE_ENGINE_CACHE_DIR:-$HOME/.cache/robonix/scene-engines}"
# Startup checks only: export/compilation must precede Atlas registration.
"$PY" "$PKG/scripts/prepare_models.py" --check

# Host-persisted scene state (object-memory DB + scene-graph caches).
mkdir -p "$PKG/rbnx-build/data/robonix"
export SCENE_GRAPH_CACHE_DIR="${SCENE_GRAPH_CACHE_DIR:-$PKG/rbnx-build/data/robonix/scene_graph/cache}"
export SCENE_ANNOTATIONS_DIR="${SCENE_ANNOTATIONS_DIR:-$PKG/rbnx-build/data/robonix/scene_annotations}"
mkdir -p "$SCENE_ANNOTATIONS_DIR"

# Service knobs (same defaults the docker run wires via -e).
export SCENE_WEB_PORT="${SCENE_WEB_PORT:-50107}"
# Loopback by default: the web UI has no authentication and its annotation
# endpoints write map data. Set SCENE_WEB_HOST=0.0.0.0 to reach it from another
# machine.
export SCENE_WEB_HOST="${SCENE_WEB_HOST-127.0.0.1}"
export SCENE_LOG_LEVEL="${SCENE_LOG_LEVEL:-INFO}"
echo "[scene/native] python=$PY ros=$ROS_DISTRO rmw=${RMW_IMPLEMENTATION:-<default>} web=$SCENE_WEB_HOST:$SCENE_WEB_PORT"
exec "$PY" -m scene_service.service
