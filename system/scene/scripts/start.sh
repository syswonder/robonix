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

INFERENCE="${SCENE_CG_INFERENCE:-torch}"
INFERENCE="$(printf '%s' "$INFERENCE" | tr '[:upper:]' '[:lower:]' | xargs)"
export SCENE_CG_INFERENCE="$INFERENCE"
CG_FORCE_CPU="$(printf '%s' "${SCENE_CG_FORCE_CPU:-}" | tr '[:upper:]' '[:lower:]')"
if [[ "${ROBONIX_FORCE_CPU:-0}" == "1" || "$CG_FORCE_CPU" == "1" || "$CG_FORCE_CPU" == "true" || "$CG_FORCE_CPU" == "yes" ]]; then
    export ROBONIX_FORCE_CPU=1
    if [[ "$INFERENCE" == "tensorrt" ]]; then
        echo "[scene/start] TensorRT requires GPU passthrough; unset ROBONIX_FORCE_CPU" >&2
        exit 2
    fi
    export SCENE_CG_FORCE_CPU=1
fi


PACKAGED=0
if [[ -f "$PKG/rbnx-build/docker-env.json" ]]; then
    python3 "$PKG/scripts/prepare_docker.py" --check
    source "$PKG/rbnx-build/docker-runtime.sh"
    PACKAGED=1
else
    DEFAULT_IMG=robonix-scene
    if [[ "$INFERENCE" != "tensorrt" && "${SCENE_PERCEPTION_BACKEND:-}" != "concept_graphs" ]] \
       && docker image inspect robonix-scene-dualmap >/dev/null 2>&1; then
        DEFAULT_IMG=robonix-scene-dualmap
    fi
    IMG="${ROBONIX_SCENE_IMAGE:-$DEFAULT_IMG}"
fi
CT="${ROBONIX_SCENE_CONTAINER:-robonix_scene}"
RUNTIME_PROTO_TMP=""

case "$INFERENCE" in
    torch) ;;
    tensorrt)
        export SCENE_PERCEPTION_BACKEND="${SCENE_PERCEPTION_BACKEND:-concept_graphs}"
        TRT_GPU=(--gpus all)
        if is_native_platform "${ROBONIX_SCENE_PLATFORM:-}" || [[ -e /etc/nv_tegra_release ]]; then
            TRT_GPU=(--runtime nvidia)
        fi
        if [[ "$PACKAGED" == "0" ]]; then
        # Check before installing cleanup traps or touching an existing service.
        docker run --rm "${TRT_GPU[@]}" -e NVIDIA_DRIVER_CAPABILITIES=all \
            -e SCENE_CG_INFERENCE=tensorrt -e SCENE_ONNX_DIR=/scene-models/onnx \
            -e SCENE_ENGINE_CACHE_DIR=/scene-models/engines \
            -e SCENE_PROFILE="${SCENE_PROFILE:-lite}" \
            -e SCENE_PERCEPTION_BACKEND="$SCENE_PERCEPTION_BACKEND" \
            -e SCENE_CLIP_MODEL="${SCENE_CLIP_MODEL:-}" \
            -e SCENE_CG_FORCE_CPU="${SCENE_CG_FORCE_CPU:-}" \
            -v "$PKG:/scene:ro" -v "$PKG/rbnx-build/tensorrt:/scene-models:ro" \
            --entrypoint python3 "$IMG" /scene/scripts/prepare_models.py --check
        fi
        ;;
    *) echo "[scene/start] SCENE_CG_INFERENCE must be torch or tensorrt" >&2; exit 2 ;;
esac


cleanup() {
    if [[ -n "${RUN_DIR:-}" ]]; then
        if [[ -s "$RUN_DIR/cid" ]]; then
            timeout 15s docker stop "$(cat "$RUN_DIR/cid")" >/dev/null 2>&1 || true
        fi
        rm -rf -- "$RUN_DIR"
    fi
    if [[ -n "$RUNTIME_PROTO_TMP" ]]; then
        rm -rf -- "$RUNTIME_PROTO_TMP"
    fi
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

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

if [[ "$PACKAGED" == "0" ]]; then
    prepare_runtime_proto_gen
fi

mkdir -p rbnx-build/data
# Host-persisted scene state. Mounted at /data/robonix in the container so the
# object-memory DB — and the scene-graph JSON caches, which otherwise die with
# the --rm container — survive across boots. CI sets SCENE_DATA_DIR to keep
# semantic/object state isolated per run.
SCENE_HOST_DATA_DIR="${SCENE_DATA_DIR:-$(pwd)/rbnx-build/data/robonix}"
mkdir -p "$SCENE_HOST_DATA_DIR"

# Every optional argument array below is expanded as ${ARR[@]+"${ARR[@]}"}.
# bash 3.2, still the default on macOS, treats "${ARR[@]}" on an empty array as
# an unbound variable under set -u and aborts the script; bash 4.4+ does not.
# start.sh was dying there before it could launch the service container, which
# is why the runtime-protobuf contract test only ever saw the codegen run.
declare -a EXTRA_MOUNTS=()
if [[ "$PACKAGED" == "1" ]]; then
    EXTRA_MOUNTS+=(-v "$SCENE_DOCKER_CACHE:/scene-models:ro")
elif [[ "$INFERENCE" == "tensorrt" ]]; then
    EXTRA_MOUNTS+=(-v "$PKG/rbnx-build/tensorrt:/scene-models:ro")
fi

if [[ -n "${RBNX_CONFIG_FILE:-}" ]]; then
    EXTRA_MOUNTS+=(-v "${RBNX_CONFIG_FILE}:${RBNX_CONFIG_FILE}:ro")
fi
# The ConceptGraphs export (SCENE_EXPORT_CG_PICKLE) is written inside the
# container; mount the directory so the file lands on the host at the same
# path the caller named.
if [[ -n "${SCENE_EXPORT_CG_PICKLE:-}" ]]; then
  mkdir -p "${SCENE_EXPORT_CG_PICKLE}"
  EXTRA_MOUNTS+=(-v "${SCENE_EXPORT_CG_PICKLE}:${SCENE_EXPORT_CG_PICKLE}")
fi
# Packaged images use their staged checkpoints; legacy images can bind local overrides.
for variable in SCENE_YOLO_WORLD_WEIGHTS SCENE_MOBILE_SAM_WEIGHTS SCENE_CLIP_PRETRAINED; do
  if [[ "$PACKAGED" == 0 && -n "${!variable:-}" ]]; then
    if [[ -f "${!variable}" ]]; then
      checkpoint="$(realpath "${!variable}")"
      EXTRA_MOUNTS+=(-v "$checkpoint:$checkpoint:ro" -e "$variable=$checkpoint")
    else
      EXTRA_MOUNTS+=(-e "$variable=${!variable}")
    fi
  fi
done
# DualMap backend: a host vocabulary file is mounted read-only at the same path so
# SCENE_DUALMAP_CLASSES means the same thing inside the container.
if [[ -n "${SCENE_DUALMAP_CLASSES:-}" ]]; then
  [[ -f "${SCENE_DUALMAP_CLASSES}" ]] || { echo "[scene/start] SCENE_DUALMAP_CLASSES=${SCENE_DUALMAP_CLASSES} is not a file" >&2; exit 2; }
  EXTRA_MOUNTS+=(-v "${SCENE_DUALMAP_CLASSES}:${SCENE_DUALMAP_CLASSES}:ro")
fi
# A DualMap checkout other than the one baked into the image (a patched or
# newer copy on the host) has to be visible inside the container at the same
# path the adapter is told to use.
if [[ -n "${SCENE_DUALMAP_ROOT:-}" ]]; then
  [[ -d "${SCENE_DUALMAP_ROOT}/utils" ]] || { echo "[scene/start] SCENE_DUALMAP_ROOT=${SCENE_DUALMAP_ROOT} is not a DualMap checkout" >&2; exit 2; }
  EXTRA_MOUNTS+=(-v "${SCENE_DUALMAP_ROOT}:${SCENE_DUALMAP_ROOT}:ro")
fi
# Legacy and x86 images can mount Full weights; packaged Jetson images use
# the checkpoints staged during their build.
if [[ "$PACKAGED" == 0 && -n "${SCENE_MODELS_DIR:-}" ]]; then
  [[ -d "${SCENE_MODELS_DIR}" ]] || { echo "[scene/start] SCENE_MODELS_DIR=${SCENE_MODELS_DIR} is not a directory" >&2; exit 2; }
  EXTRA_MOUNTS+=(-v "${SCENE_MODELS_DIR}:/opt/models/full:ro")
fi

# Scribe's log directory: rbnx sets SCRIBE_LOG_DIR and pipes every package's
# output into `<tag>.log` under it. Without this, scene cannot read back what
# it just wrote and the web UI's log view has nothing to show. Read-only, and
# at the same path inside as out so the variable needs no translation.
if [[ -n "${SCRIBE_LOG_DIR:-}" && -d "${SCRIBE_LOG_DIR}" ]]; then
    EXTRA_MOUNTS+=(-v "${SCRIBE_LOG_DIR}:${SCRIBE_LOG_DIR}:ro")
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
if [[ "${ROBONIX_FORCE_CPU:-0}" == "1" ]]; then
    GPU_ARGS=(-e NVIDIA_VISIBLE_DEVICES=void)
fi
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

declare -a SOURCE_MOUNTS=()
if [[ "$PACKAGED" == "0" ]]; then
    SOURCE_MOUNTS=(-v "$PKG:/scene"
        -v "$PKG/rbnx-build/codegen/scene_proto_gen:/scene/rbnx-build/codegen/proto_gen:ro"
        -v "$(rbnx path robonix-api):/robonix-api:ro")
fi
# All image/cache/protobuf checks have passed before replacing a container.
docker rm -f "$CT" >/dev/null 2>&1 || true
RUN_DIR="$(mktemp -d "$PKG/rbnx-build/.scene-run.XXXXXX")"

# SCENE_WEB_HOST below defaults to loopback. The container runs with
# --network host, so what the service binds is what the host exposes, and the
# web UI has no authentication while its annotation endpoints write map data.
# Set SCENE_WEB_HOST=0.0.0.0 to reach it from another machine.
docker run --rm --cidfile "$RUN_DIR/cid" \
    --name "$CT" \
    --entrypoint /scene/docker/entrypoint.sh \
    --network host \
    --ipc=host \
    ${GPU_ARGS[@]+"${GPU_ARGS[@]}"} \
    -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
    -e ROBONIX_PROVIDER_BIND_HOST="${ROBONIX_PROVIDER_BIND_HOST:-0.0.0.0}" \
    -e ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-}" \
    -e ROBONIX_CAPABILITY_ID="${ROBONIX_CAPABILITY_ID:-com.robonix.system.scene}" \
    -e ROBONIX_PKG_HOST_DIR="$(pwd)" \
    -e SCENE_WEB_PORT="${SCENE_WEB_PORT:-50107}" \
    -e SCENE_WEB_HOST="${SCENE_WEB_HOST-127.0.0.1}" \
    -e SCENE_LOG_LEVEL="${SCENE_LOG_LEVEL:-INFO}" \
    -e SCRIBE_LOG_DIR="${SCRIBE_LOG_DIR:-}" \
    -e SCENE_OBJECT_VIEWS_DIR="${SCENE_OBJECT_VIEWS_DIR:-/data/robonix/scene_object_views}" \
    -e SCENE_OBJECT_VIEWS_MAX="${SCENE_OBJECT_VIEWS_MAX:-}" \
    -e SCENE_CAPTION_LANG="${SCENE_CAPTION_LANG:-zh}" \
    -e SCENE_PROFILE="${SCENE_PROFILE:-}" \
    -e SCENE_CG_INFERENCE="$INFERENCE" \
    -e SCENE_ONNX_DIR=/scene-models/onnx \
    -e SCENE_ENGINE_CACHE_DIR=/scene-models/engines \
    -e SCENE_PERCEPTION_BACKEND="${SCENE_PERCEPTION_BACKEND:-}" \
    -e SCENE_DUALMAP_CLASSES="${SCENE_DUALMAP_CLASSES:-}" \
    -e SCENE_DUALMAP_ROOT="${SCENE_DUALMAP_ROOT:-}" \
    -e PYTORCH_CUDA_ALLOC_CONF="${PYTORCH_CUDA_ALLOC_CONF:-expandable_segments:True}" \
    -e SCENE_CG_FORCE_CPU="${SCENE_CG_FORCE_CPU:-}" \
    -e SCENE_CG_OBJ_MIN_POINTS="${SCENE_CG_OBJ_MIN_POINTS:-}" \
    -e SCENE_CG_MIN_POINTS="${SCENE_CG_MIN_POINTS:-}" \
    -e SCENE_CG_FLOOR_Z_M="${SCENE_CG_FLOOR_Z_M:-}" \
    -e SCENE_CG_PER_DETECTION_DBSCAN="${SCENE_CG_PER_DETECTION_DBSCAN:-}" \
    -e SCENE_CG_OBJ_MAX_POINTS="${SCENE_CG_OBJ_MAX_POINTS:-}" \
    -e SCENE_CG_MERGE_THRESHOLD="${SCENE_CG_MERGE_THRESHOLD:-}" \
    -e SCENE_CG_VOXEL_SIZE="${SCENE_CG_VOXEL_SIZE:-}" \
    -e SCENE_CLIP_MODEL="${SCENE_CLIP_MODEL:-}" \
    -e SCENE_DETECT_PERIOD_S="${SCENE_DETECT_PERIOD_S:-}" \
    -e SCENE_DETECT_CONFIDENCE="${SCENE_DETECT_CONFIDENCE:-}" \
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
    -e SCENE_CAMERA_FRAME="${SCENE_CAMERA_FRAME:-}" \
    -e SCENE_BASE_FRAME="${SCENE_BASE_FRAME:-}" \
    -e SCENE_EXPORT_CG_PICKLE="${SCENE_EXPORT_CG_PICKLE:-}" \
    -e SCENE_EXPORT_CG_EXP="${SCENE_EXPORT_CG_EXP:-}" \
    ${MAP_ID_ARGS[@]+"${MAP_ID_ARGS[@]}"} \
    -e RBNX_CONFIG_FILE="${RBNX_CONFIG_FILE:-}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
    -e CYCLONEDDS_URI="${CYCLONEDDS_URI:-}" \
    ${ZENOH_ARGS[@]+"${ZENOH_ARGS[@]}"} \
    ${SOURCE_MOUNTS[@]+"${SOURCE_MOUNTS[@]}"} \
    -v "$SCENE_HOST_DATA_DIR":/data/robonix \
    ${EXTRA_MOUNTS[@]+"${EXTRA_MOUNTS[@]}"} \
    "$IMG" &
wait "$!"
