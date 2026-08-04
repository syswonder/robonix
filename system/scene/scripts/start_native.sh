#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# scene native start — run scene_service directly on the host (no docker).
#
# Used on Jetson (and any host with ROS 2 + a CUDA-capable python that already
# has torch/torchvision/ultralytics). Selected by the jetson-native
# manifest (`ROBONIX_SCENE_FORCE=native bash scripts/start.sh`, which execs
# this). The build phase (build_native.sh) created rbnx-build/venv with
# --system-site-packages so the heavy CUDA wheels come from the host JetPack
# stack and only the light pure-python deps live in the venv.
#
# Unlike the docker path this does NOT force a specific RMW / SHM profile:
# native scene must share whatever RMW the other host nodes use so it actually
# sees the camera / pointcloud topics they publish.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# Native Scene runs on the host Python. The lite profile deliberately works on
# hosts without torch; metric profiles validate their model dependencies only
# when the detector is actually started.
PY="${SCENE_NATIVE_PYTHON:-python3}"

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

# Codegen stubs (proto_gen + robonix_mcp_types) + robonix-api + the package.
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:$PKG:${PYTHONPATH:-}"
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PYTHONPATH"
fi

PERCEPTION_PROFILE="${SCENE_PERCEPTION_PROFILE:-full}"
if [[ "$PERCEPTION_PROFILE" == "lite" ]]; then
    echo "[scene/native] profile=lite: skipping all perception model checks"
else
    # Model weights: build.sh freezes YOLO-World's prompt embeddings into a
    # native build artifact. Robot boot must never invoke the YOLO text encoder
    # or fetch model weights.
    BAKED_YOLO="$PKG/rbnx-build/data/models/yolov8l-world-baked.safetensors"
    BAKED_YOLO_JETSON="$PKG/rbnx-build/data/models/yolov8s-worldv2-baked.safetensors"
    export SCENE_YOLO_WORLD_WEIGHTS_FULL="${SCENE_YOLO_WORLD_WEIGHTS_FULL:-$BAKED_YOLO}"
    export SCENE_YOLO_WORLD_WEIGHTS_JETSON="${SCENE_YOLO_WORLD_WEIGHTS_JETSON:-$BAKED_YOLO_JETSON}"
    export SCENE_YOLO_WORLD_WEIGHTS="${SCENE_YOLO_WORLD_WEIGHTS:-$SCENE_YOLO_WORLD_WEIGHTS_FULL}"
    export SCENE_MOBILE_SAM_WEIGHTS="${SCENE_MOBILE_SAM_WEIGHTS:-$PKG/rbnx-build/data/models/mobile_sam.fp16.safetensors}"
    export SCENE_CLIP_MODEL="${SCENE_CLIP_MODEL:-ViT-B-32}"
    export SCENE_CLIP_PRETRAINED="${SCENE_CLIP_PRETRAINED:-$PKG/rbnx-build/data/models/open_clip_vit_b32.fp16.safetensors}"
    export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-$PKG/rbnx-build/data/ultralytics}"
    export SCENE_TENSORRT_CACHE_DIR="${SCENE_TENSORRT_CACHE_DIR:-$PKG/rbnx-build/data/model-cache}"
    mkdir -p "$SCENE_TENSORRT_CACHE_DIR"
    for model_path in \
        "$SCENE_YOLO_WORLD_WEIGHTS" \
        "${SCENE_YOLO_WORLD_WEIGHTS%.safetensors}.classes.json" \
        "$SCENE_YOLO_WORLD_WEIGHTS_JETSON" \
        "${SCENE_YOLO_WORLD_WEIGHTS_JETSON%.safetensors}.classes.json" \
        "$SCENE_MOBILE_SAM_WEIGHTS" \
        "$SCENE_CLIP_PRETRAINED"; do
        if [[ ! -s "$model_path" ]]; then
            echo "[scene/native] error: missing model weight $model_path" >&2
            echo "               run system/scene/scripts/build.sh before starting scene." >&2
            exit 1
        fi
    done
fi

# Host-persisted scene state (object-memory DB + scene-graph caches).
mkdir -p "$PKG/rbnx-build/data/robonix"
export SCENE_GRAPH_CACHE_DIR="${SCENE_GRAPH_CACHE_DIR:-$PKG/rbnx-build/data/robonix/scene_graph/cache}"
export SCENE_ANNOTATIONS_DIR="${SCENE_ANNOTATIONS_DIR:-$PKG/rbnx-build/data/robonix/scene_annotations}"
mkdir -p "$SCENE_ANNOTATIONS_DIR"

# Service knobs (same defaults the docker run wires via -e).
export SCENE_WEB_PORT="${SCENE_WEB_PORT:-50107}"
export SCENE_WEB_HOST="${SCENE_WEB_HOST-0.0.0.0}"
export SCENE_LOG_LEVEL="${SCENE_LOG_LEVEL:-INFO}"

echo "[scene/native] python=$PY ros=$ROS_DISTRO rmw=${RMW_IMPLEMENTATION:-<default>} web=$SCENE_WEB_HOST:$SCENE_WEB_PORT"
exec "$PY" -m scene_service.service
