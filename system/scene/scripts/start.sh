#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scene service start phase — `rbnx boot` calls this via package_manifest.yaml.
#
# Runs the `robonix-scene` container on the host network so it shares
# the host DDS bus with whatever sim/robot container publishes the
# observation topics. `--ipc=host` is load-bearing: FastRTPS's default
# SHM transport keys are namespaced by /dev/shm, so without sharing
# host IPC scene saw publishers but received zero messages.
#
# Trap discipline: when boot SIGTERMs our PGID, this script's TERM
# trap stops + removes the container. `docker run --rm` alone is not
# enough; docker leaks containers if the host wrapper dies before
# sending stop.

set -euo pipefail

CT="${ROBONIX_SCENE_CONTAINER:-robonix_scene}"
IMG="${ROBONIX_SCENE_IMAGE:-robonix-scene}"

cleanup() {
    docker stop "$CT" >/dev/null 2>&1 || true
    kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Drop a stopped container from a previous run.
docker rm -f "$CT" >/dev/null 2>&1 || true

mkdir -p rbnx-build/data

declare -a EXTRA_MOUNTS=()
if [[ -n "${RBNX_CONFIG_FILE:-}" ]]; then
    EXTRA_MOUNTS+=(-v "${RBNX_CONFIG_FILE}:${RBNX_CONFIG_FILE}:ro")
fi

# GPU passthrough: ConceptGraphs perception (YOLO-World + MobileSAM +
# CLIP) wants CUDA. Auto-detect via nvidia-smi; opt out by setting
# ROBONIX_FORCE_CPU=1. Without this flag the container sees CPU only
# and CLIP/YOLO run ~5x slower.
declare -a GPU_ARGS=()
if [[ "${ROBONIX_FORCE_CPU:-0}" != "1" ]] && command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
    GPU_ARGS=(--gpus all)
fi

exec docker run --rm \
    --name "$CT" \
    --network host \
    --ipc=host \
    "${GPU_ARGS[@]}" \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-com.robonix.system.scene}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e SCENE_WEB_PORT="${SCENE_WEB_PORT:-50107}" \
    -e SCENE_LOG_LEVEL="${SCENE_LOG_LEVEL:-INFO}" \
    -e SCENE_CG_FORCE_CPU="${SCENE_CG_FORCE_CPU:-}" \
    -e SCENE_PERCEPTION_WAIT_S="${SCENE_PERCEPTION_WAIT_S:-30}" \
    -e SCENE_OPEN_VOCAB_CLASSES="${SCENE_OPEN_VOCAB_CLASSES:-}" \
    -e VLM_BASE_URL="${VLM_BASE_URL:-}" \
    -e VLM_API_KEY="${VLM_API_KEY:-}" \
    -e VLM_MODEL="${VLM_MODEL:-}" \
    -e SCENE_GRAPH_ENABLED="${SCENE_GRAPH_ENABLED:-true}" \
    -e SCENE_GRAPH_INTERVAL_SEC="${SCENE_GRAPH_INTERVAL_SEC:-30}" \
    -e SCENE_GRAPH_CACHE_DIR="${SCENE_GRAPH_CACHE_DIR:-/data/robonix/scene_graph/cache}" \
    -e SCENE_GRAPH_MIN_OBSERVATIONS="${SCENE_GRAPH_MIN_OBSERVATIONS:-2}" \
    -e SCENE_GRAPH_MAX_OBJECTS="${SCENE_GRAPH_MAX_OBJECTS:-80}" \
    -e SCENE_GRAPH_MAX_LLM_RELATIONS_PER_CYCLE="${SCENE_GRAPH_MAX_LLM_RELATIONS_PER_CYCLE:-20}" \
    -e RBNX_CONFIG_FILE="${RBNX_CONFIG_FILE:-}" \
    -e CUDA_VISIBLE_DEVICES="${CUDA_VISIBLE_DEVICES:-}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -v "$(pwd)":/scene \
    -v "$(rbnx path robonix-api)":/robonix-api:ro \
    "${EXTRA_MOUNTS[@]}" \
    "$IMG"
