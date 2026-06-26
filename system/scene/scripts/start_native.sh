#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# scene native start — run scene_service directly on the host (no docker).
#
# Used on Jetson (and any host with ROS 2 + a CUDA-capable python that already
# has torch/torchvision/ultralytics/open3d). Selected by the jetson-native
# manifest (`ROBONIX_SCENE_FORCE=native bash scripts/start.sh`, which execs
# this). The build phase (build_native.sh) created rbnx-build/venv with
# --system-site-packages so the heavy CUDA wheels come from the host JetPack
# stack and only the light pure-python deps live in the venv.
#
# Unlike the docker path this does NOT force FastRTPS/no-shm: native scene must
# share the host's RMW (the car runs CycloneDDS) to actually see the camera /
# pointcloud topics published by the other native nodes.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

# Native scene runs on the host python (the build installed scene's light deps
# into the user site next to the host JetPack torch — no venv; see build.sh).
PY="${SCENE_NATIVE_PYTHON:-python3}"
"$PY" -c "import torch" 2>/dev/null || {
    echo "[scene/native] error: '$PY' has no torch — run the jetson-native build" >&2
    echo "               and install the JetPack CUDA stack first (see scene README)." >&2
    exit 1
}

ROS_DISTRO="${ROS_DISTRO:-humble}"
# shellcheck disable=SC1091
set +u; source "/opt/ros/${ROS_DISTRO}/setup.bash"; set -u

# Codegen stubs (proto_gen + robonix_mcp_types) + robonix-api + the package.
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:$PKG:${PYTHONPATH:-}"
if ROBONIX_API="$(rbnx path robonix-api 2>/dev/null)"; then
    export PYTHONPATH="$ROBONIX_API:$PYTHONPATH"
fi

# Model weights: build.sh fetches yolo + mobile_sam to docker/_weights (shared
# with the docker path); CLIP downloads to a host HF cache on first run.
W="$PKG/docker/_weights"
export SCENE_YOLO_WORLD_WEIGHTS="${SCENE_YOLO_WORLD_WEIGHTS:-$W/yolov8l-world.pt}"
export SCENE_MOBILE_SAM_WEIGHTS="${SCENE_MOBILE_SAM_WEIGHTS:-$W/mobile_sam.pt}"
export SCENE_CLIP_MODEL="${SCENE_CLIP_MODEL:-ViT-B-32}"
export SCENE_CLIP_PRETRAINED="${SCENE_CLIP_PRETRAINED:-laion2b_s34b_b79k}"
export HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}"
export HF_HOME="${HF_HOME:-$PKG/rbnx-build/data/hf}"

# Host-persisted scene state (object-memory DB + scene-graph caches).
mkdir -p "$PKG/rbnx-build/data/robonix"
export SCENE_GRAPH_CACHE_DIR="${SCENE_GRAPH_CACHE_DIR:-$PKG/rbnx-build/data/robonix/scene_graph/cache}"

# Service knobs (same defaults the docker run wires via -e).
export SCENE_WEB_PORT="${SCENE_WEB_PORT:-50107}"
export SCENE_LOG_LEVEL="${SCENE_LOG_LEVEL:-INFO}"

echo "[scene/native] python=$PY ros=$ROS_DISTRO rmw=${RMW_IMPLEMENTATION:-<default>} web=$SCENE_WEB_PORT"
exec "$PY" -m scene_service.service
