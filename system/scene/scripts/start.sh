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
RUNTIME_PROTO_TMP=""

cleanup() {
    timeout 15s docker stop "$CT" >/dev/null 2>&1 || true
    if [[ -n "$RUNTIME_PROTO_TMP" ]]; then
        rm -rf -- "$RUNTIME_PROTO_TMP"
    fi
}
trap cleanup EXIT INT TERM

# Drop a stopped container from a previous run.
docker rm -f "$CT" >/dev/null 2>&1 || true

# Python protobuf generated code is only forward-compatible within protobuf's
# documented runtime window.  `rbnx codegen` intentionally uses the host's
# active python3, which can be newer than this image (for example gencode
# 7.35.0 with a 6.33.6 runtime).  Never suppress protobuf's version check and
# never guess a package pin here.  Instead, regenerate Scene's Python stubs
# with the exact grpc_tools/protobuf stack that will import them, without
# network access, then validate every generated module before Scene starts.
# Keep these stubs separate from the host build output: other packages may
# legitimately use a different Python runtime.
prepare_runtime_proto_gen() {
    local proto_staging="$PKG/rbnx-build/proto-staging"
    local runtime_proto
    local runtime_proto_gen="$PKG/rbnx-build/codegen/scene_proto_gen"

    runtime_proto="$(rbnx path runtime-proto)" || {
        echo "[scene/start] cannot resolve Robonix runtime proto directory" >&2
        return 1
    }
    [[ -d "$runtime_proto" && -f "$runtime_proto/atlas.proto" ]] || {
        echo "[scene/start] missing runtime atlas.proto: $runtime_proto" >&2
        return 1
    }
    [[ -d "$proto_staging" ]] \
        && find "$proto_staging" -maxdepth 1 -type f -name '*.proto' -print -quit \
            | grep -q . || {
        echo "[scene/start] missing staged package protos; run rbnx build first" >&2
        return 1
    }

    mkdir -p "$PKG/rbnx-build/codegen"
    RUNTIME_PROTO_TMP="$(mktemp -d "${runtime_proto_gen}.tmp.XXXXXX")"

    docker run --rm \
        --network none \
        --entrypoint sh \
        --user "$(id -u):$(id -g)" \
        -e HOME=/tmp \
        -v "$runtime_proto:/runtime-proto:ro" \
        -v "$proto_staging:/proto-staging:ro" \
        -v "$RUNTIME_PROTO_TMP:/proto-gen" \
        "$IMG" -ec '
            python3 -m grpc_tools.protoc \
                -I/runtime-proto \
                -I/proto-staging \
                --python_out=/proto-gen \
                --grpc_python_out=/proto-gen \
                /runtime-proto/*.proto \
                /proto-staging/*.proto
            PYTHONPATH=/proto-gen python3 -c '\''import importlib, pathlib; p = pathlib.Path("/proto-gen"); modules = sorted({f.stem for f in p.glob("*_pb2.py")} | {f.stem for f in p.glob("*_pb2_grpc.py")}); assert modules; [importlib.import_module(name) for name in modules]'\''
        '

    rm -rf -- "$runtime_proto_gen"
    mv -- "$RUNTIME_PROTO_TMP" "$runtime_proto_gen"
    RUNTIME_PROTO_TMP=""
    echo "[scene/start] runtime-compatible protobuf stubs ready"
}

prepare_runtime_proto_gen

mkdir -p rbnx-build/data
# Host-persisted scene state. Mounted at /data/robonix in the container so the
# object-memory DB — and the scene-graph JSON caches, which otherwise die with
# the --rm container — survive across boots. CI sets SCENE_DATA_DIR to keep
# semantic/object state isolated per run.
SCENE_HOST_DATA_DIR="${SCENE_DATA_DIR:-$(pwd)/rbnx-build/data/robonix}"
mkdir -p "$SCENE_HOST_DATA_DIR"

declare -a EXTRA_MOUNTS=()
# Deprecated compatibility only. Canonical deployments send
# system.scene.config through Driver(CMD_INIT) and leave this unset.
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

declare -a MAP_ID_ARGS=()
if [[ -n "${SCENE_MAP_ID:-}" ]]; then
    MAP_ID_ARGS=(-e "SCENE_MAP_ID=${SCENE_MAP_ID}")
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
    --entrypoint /scene/docker/entrypoint.sh \
    --network host \
    --ipc=host \
    ${GPU_ARGS[@]+"${GPU_ARGS[@]}"} \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}" \
    -e ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}" \
    -e ROBONIX_DRIVER_CONTRACT_ID="${ROBONIX_DRIVER_CONTRACT_ID-robonix/lifecycle/driver}" \
    -e ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK="${ROBONIX_DRIVER_ALLOW_OLD_ARTIFACT_FALLBACK:-}" \
    -e ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-com.robonix.system.scene}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e SCENE_WEB_PORT="${SCENE_WEB_PORT:-50107}" \
    -e SCENE_WEB_HOST="${SCENE_WEB_HOST-0.0.0.0}" \
    -e SCENE_CAMERA_FRAME="${SCENE_CAMERA_FRAME:-}" \
    -e SCENE_BASE_FRAME="${SCENE_BASE_FRAME:-}" \
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
    -e SCENE_ANNOTATIONS_DIR="${SCENE_ANNOTATIONS_DIR:-/data/robonix/scene_annotations}" \
    -e SCENE_MAP_LOAD_TIMEOUT_S="${SCENE_MAP_LOAD_TIMEOUT_S:-240}" \
    -e SCENE_GRAPH_MIN_OBSERVATIONS="${SCENE_GRAPH_MIN_OBSERVATIONS:-2}" \
    -e SCENE_GRAPH_MAX_OBJECTS="${SCENE_GRAPH_MAX_OBJECTS:-80}" \
    -e SCENE_GRAPH_MAX_LLM_RELATIONS_PER_CYCLE="${SCENE_GRAPH_MAX_LLM_RELATIONS_PER_CYCLE:-20}" \
    -e SCENE_OBJECT_MEMORY_ENABLED="${SCENE_OBJECT_MEMORY_ENABLED:-true}" \
    -e SCENE_OBJECT_MEMORY_DB="${SCENE_OBJECT_MEMORY_DB:-/data/robonix/scene_memory/objects.db}" \
    ${MAP_ID_ARGS[@]+"${MAP_ID_ARGS[@]}"} \
    -e RBNX_CONFIG_FILE="${RBNX_CONFIG_FILE:-}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
    -e CYCLONEDDS_URI="${CYCLONEDDS_URI:-}" \
    ${ZENOH_ARGS[@]+"${ZENOH_ARGS[@]}"} \
    -v "$(pwd)":/scene \
    -v "$PKG/rbnx-build/codegen/scene_proto_gen:/scene/rbnx-build/codegen/proto_gen:ro" \
    -v "$SCENE_HOST_DATA_DIR":/data/robonix \
    -v "$(rbnx path robonix-api)":/robonix-api:ro \
    ${EXTRA_MOUNTS[@]+"${EXTRA_MOUNTS[@]}"} \
    "$IMG"
