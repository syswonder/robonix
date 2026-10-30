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

PY="${SCENE_NATIVE_PYTHON:-python3}"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

# Codegen stubs (proto_gen + robonix_mcp_types) + robonix-api + the package.
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:$PKG:${PYTHONPATH:-}"
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PYTHONPATH"
fi

# Keep legacy model identifiers for exported map metadata; runtime loads ONNX.
W="$PKG/docker/_weights"
export SCENE_CLIP_MODEL="${SCENE_CLIP_MODEL:-ViT-B-32}"
export SCENE_CLIP_PRETRAINED="${SCENE_CLIP_PRETRAINED:-$W/open_clip_pytorch_model.bin}"
export SCENE_ONNX_DIR="${SCENE_ONNX_DIR:-$W/onnx}"
ONNX_DIR="$SCENE_ONNX_DIR"
export SCENE_ENGINE_CACHE_DIR="${SCENE_ENGINE_CACHE_DIR:-$HOME/.cache/robonix/scene-engines}"
if [[ ! -s "$ONNX_DIR/yolo-world-cut.onnx" ]]; then
    echo "[scene/native] error: no ONNX exports under $ONNX_DIR — perception cannot start" >&2
    echo "               run system/scene/scripts/export_models.py on the build machine." >&2
    exit 1
fi
if ! "$PY" -c "import tensorrt, cuda.bindings.runtime" 2>/dev/null; then
    echo "[scene/native] error: '$PY' lacks tensorrt/cuda-bindings — perception cannot start" >&2
    echo "               install the engine runtime (docker/requirements/scene-perception-engines.txt)." >&2
    exit 1
fi
echo "[scene/native] checking prebuilt engine cache ..."
"$PY" "$PKG/scripts/build_engines.py" --models-dir "$ONNX_DIR" --check || exit 1

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
