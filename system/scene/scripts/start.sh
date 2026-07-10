#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scene service start phase — `rbnx boot` calls this via package_manifest.yaml.
#
# Runs the `robonix-scene` container on the host network so it shares
# the host ROS 2 graph with whatever sim/robot container publishes the
# observation topics. The runtime RMW is passed through so scene uses
# the same transport as the rest of the deploy.
#
# Trap discipline: when boot SIGTERMs our PGID, this script's TERM
# trap stops + removes the container. `docker run --rm` alone is not
# enough; docker leaks containers if the host wrapper dies before
# sending stop.

set -euo pipefail

# ── Platform dispatch (same scheme as mapping_rbnx) ─────────────────────────
#   ROBONIX_SCENE_FORCE=native|docker     explicit hard pin
#   else auto: ROBONIX_SCENE_PLATFORM=jetson_orin → native, otherwise docker
# native → scripts/start_native.sh (host venv + host JetPack torch, no docker).
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
is_native_platform() { case "$1" in jetson_orin|jetson*|orin*) return 0 ;; *) return 1 ;; esac }
case "${ROBONIX_SCENE_FORCE:-}" in
    native) MODE=native ;;
    docker) MODE=docker ;;
    "") if is_native_platform "${ROBONIX_SCENE_PLATFORM:-}"; then MODE=native; else MODE=docker; fi ;;
    *) echo "[scene/start] ROBONIX_SCENE_FORCE=${ROBONIX_SCENE_FORCE} not in {native,docker}" >&2; exit 2 ;;
esac
echo "[scene/start] mode=${MODE} (FORCE=${ROBONIX_SCENE_FORCE:-} PLATFORM=${ROBONIX_SCENE_PLATFORM:-})"
if [[ "$MODE" == "native" ]]; then
    exec bash "${PKG}/scripts/start_native.sh"
fi

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
# Host-persisted scene state. Mounted at /data/robonix in the container so the
# object-memory DB — and the scene-graph JSON caches, which otherwise die with
# the --rm container — survive across boots. CI sets SCENE_DATA_DIR to keep
# semantic/object state isolated per run.
SCENE_HOST_DATA_DIR="${SCENE_DATA_DIR:-$(pwd)/rbnx-build/data/robonix}"
mkdir -p "$SCENE_HOST_DATA_DIR"

declare -a EXTRA_MOUNTS=()
if [[ -n "${RBNX_CONFIG_FILE:-}" ]]; then
    EXTRA_MOUNTS+=(-v "${RBNX_CONFIG_FILE}:${RBNX_CONFIG_FILE}:ro")
fi

declare -a ZENOH_ARGS=()
if [[ -n "${ROBONIX_ZENOH_ROUTER:-}" ]]; then
    ZENOH_ARGS=(-e "ROBONIX_ZENOH_ROUTER=${ROBONIX_ZENOH_ROUTER}")
fi
if [[ -n "${ROBONIX_ZENOH_MODE:-}" ]]; then
    ZENOH_ARGS+=(-e "ROBONIX_ZENOH_MODE=${ROBONIX_ZENOH_MODE}")
fi
if [[ -n "${ROBONIX_ZENOH_LISTEN:-}" ]]; then
    ZENOH_ARGS+=(-e "ROBONIX_ZENOH_LISTEN=${ROBONIX_ZENOH_LISTEN}")
fi

# GPU passthrough: ConceptGraphs perception (YOLO-World + MobileSAM +
# CLIP) wants CUDA. Auto-detect via nvidia-smi; opt out by setting
# ROBONIX_FORCE_CPU=1. Without this flag the container sees CPU only
# and CLIP/YOLO run ~5x slower.
declare -a GPU_ARGS=()
if [[ "${ROBONIX_FORCE_CPU:-0}" != "1" ]]; then
    # NVIDIA_DRIVER_CAPABILITIES=all is REQUIRED: with just `--gpus all` (or
    # `--runtime nvidia`) and the capability unset, the NVIDIA runtime injects
    # only "utility" — nvidia-smi works but the CUDA compute libs are NOT
    # mounted, so torch.cuda.is_available() is False and ConceptGraphs silently
    # falls back to CPU (~5x slower). Requesting all caps (compute+utility+
    # graphics) makes CUDA actually available.
    if is_native_platform "${ROBONIX_SCENE_PLATFORM:-}" || [[ -e /etc/nv_tegra_release ]]; then
        # Jetson / L4T: the container gets the GPU via the NVIDIA container
        # runtime (which bind-mounts the host CUDA libs); `--gpus all` is x86.
        GPU_ARGS=(--runtime nvidia -e NVIDIA_DRIVER_CAPABILITIES=all)
    elif command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
        GPU_ARGS=(--gpus all -e NVIDIA_DRIVER_CAPABILITIES=all)
    fi
    # Forward CUDA_VISIBLE_DEVICES ONLY when explicitly set (e.g. to pin one
    # GPU). The old unconditional `-e CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES:-}`
    # passed it EMPTY when unset on the host — which tells CUDA "no GPUs" and
    # disabled the GPU even though --gpus all had mounted it (torch.cuda → False
    # while nvidia-smi still worked). Omitting it lets all mounted GPUs show.
    if [[ ${#GPU_ARGS[@]} -gt 0 && -n "${CUDA_VISIBLE_DEVICES:-}" ]]; then
        GPU_ARGS+=(-e "CUDA_VISIBLE_DEVICES=${CUDA_VISIBLE_DEVICES}")
    fi
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
    -e SCENE_CG_OBJ_MIN_POINTS="${SCENE_CG_OBJ_MIN_POINTS:-}" \
    -e SCENE_CG_MAX_MERGE_DIST_M="${SCENE_CG_MAX_MERGE_DIST_M:-}" \
    -e SCENE_CG_CROSS_CLASS_CENTROID_MAX_M="${SCENE_CG_CROSS_CLASS_CENTROID_MAX_M:-}" \
    -e SCENE_CG_CROSS_CLASS_IOU_THRESH="${SCENE_CG_CROSS_CLASS_IOU_THRESH:-}" \
    -e SCENE_CG_CROSS_CLASS_OVERLAP_THRESH="${SCENE_CG_CROSS_CLASS_OVERLAP_THRESH:-}" \
    -e SCENE_CG_MERGE_OVERLAP_THRESH="${SCENE_CG_MERGE_OVERLAP_THRESH:-}" \
    -e SCENE_CG_MERGE_VISUAL_SIM_THRESH="${SCENE_CG_MERGE_VISUAL_SIM_THRESH:-}" \
    -e SCENE_CG_SAME_CLASS_MERGE_DIST_M="${SCENE_CG_SAME_CLASS_MERGE_DIST_M:-}" \
    -e SCENE_CG_MERGE_CLASS_GROUPS="${SCENE_CG_MERGE_CLASS_GROUPS:-}" \
    -e SCENE_OBJECT_TTL_SEC="${SCENE_OBJECT_TTL_SEC:-}" \
    -e SCENE_PERCEPTION_WAIT_S="${SCENE_PERCEPTION_WAIT_S:-30}" \
    -e SCENE_OPEN_VOCAB_CLASSES="${SCENE_OPEN_VOCAB_CLASSES:-}" \
    -e VLM_BASE_URL="${VLM_BASE_URL:-}" \
    -e VLM_API_KEY="${VLM_API_KEY:-}" \
    -e VLM_MODEL="${VLM_MODEL:-}" \
    -e VLM_REASONING_EFFORT="${VLM_REASONING_EFFORT:-}" \
    -e SCENE_GRAPH_ENABLED="${SCENE_GRAPH_ENABLED:-true}" \
    -e SCENE_GRAPH_CAPTION_ENABLED="${SCENE_GRAPH_CAPTION_ENABLED:-true}" \
    -e SCENE_GRAPH_RELATION_ENABLED="${SCENE_GRAPH_RELATION_ENABLED:-true}" \
    -e SCENE_GRAPH_INTERVAL_SEC="${SCENE_GRAPH_INTERVAL_SEC:-30}" \
    -e SCENE_GRAPH_CACHE_DIR="${SCENE_GRAPH_CACHE_DIR:-/data/robonix/scene_graph/cache}" \
    -e SCENE_GRAPH_MIN_OBSERVATIONS="${SCENE_GRAPH_MIN_OBSERVATIONS:-2}" \
    -e SCENE_GRAPH_MAX_OBJECTS="${SCENE_GRAPH_MAX_OBJECTS:-80}" \
    -e SCENE_GRAPH_MAX_LLM_RELATIONS_PER_CYCLE="${SCENE_GRAPH_MAX_LLM_RELATIONS_PER_CYCLE:-20}" \
    -e SCENE_OBJECT_MEMORY_ENABLED="${SCENE_OBJECT_MEMORY_ENABLED:-true}" \
    -e SCENE_OBJECT_MEMORY_DB="${SCENE_OBJECT_MEMORY_DB:-/data/robonix/scene_memory/objects.db}" \
    -e SCENE_MAP_ID="${SCENE_MAP_ID:-default}" \
    -e RBNX_CONFIG_FILE="${RBNX_CONFIG_FILE:-}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
    "${ZENOH_ARGS[@]}" \
    -v "$(pwd)":/scene \
    -v "$SCENE_HOST_DATA_DIR":/data/robonix \
    -v "$(rbnx path robonix-api)":/robonix-api:ro \
    "${EXTRA_MOUNTS[@]}" \
    "$IMG"
