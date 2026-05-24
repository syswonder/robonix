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

# ── Dispatch: docker (default) vs native ──────────────────────────────
# Native mode bypasses docker entirely and runs scene_service in a host
# Python venv. Triggered when:
#   ROBONIX_SCENE_FORCE=native              (explicit operator override)
#   SCENE_NATIVE_PYTHON=<path/to/venv/python>   (operator picked a venv)
# Native is required on Jetson/Tegra (CSV-mode container runtime rejects
# `--gpus all`) and is preferred whenever the host already has scene's
# heavy CV deps (torch / open_clip / ultralytics / open3d) installed —
# saves ~6 min of cold container start.
#
# Opt back into docker by leaving both vars unset, or
# ROBONIX_SCENE_FORCE=docker.
case "${ROBONIX_SCENE_FORCE:-}" in
    native)
        exec bash "$(dirname "$0")/start_native.sh" "$@"
        ;;
    docker)
        : # fall through to docker block
        ;;
    "")
        if [[ -n "${SCENE_NATIVE_PYTHON:-}" ]]; then
            exec bash "$(dirname "$0")/start_native.sh" "$@"
        fi
        ;;
    *)
        echo "[scene/start] ERR: unknown ROBONIX_SCENE_FORCE='${ROBONIX_SCENE_FORCE}' (expected: native | docker | unset)" >&2
        exit 2
        ;;
esac

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
#
# Jetson vs x86 split:
#   Jetson (Tegra) uses the CSV-mode nvidia-container-runtime which
#   REJECTS direct `--gpus all` invocation:
#     "invoking the NVIDIA Container Runtime Hook directly (e.g.
#      specifying the docker --gpus flag) is not supported. Please
#      use the NVIDIA Container Runtime (e.g. specify the
#      --runtime=nvidia flag) instead."
#   On Tegra we therefore switch to `--runtime=nvidia` (libnvidia-
#   container.so on Jetson auto-exposes the iGPU + dla + cuda libs
#   to the container; no extra flags needed).
#
#   x86 desktop nvidia uses the cdi-mode runtime where `--gpus all`
#   is the standard way to scope which GPUs are exposed.
declare -a GPU_ARGS=()
if [[ "${ROBONIX_FORCE_CPU:-0}" != "1" ]] && command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
    if [ -f /etc/nv_tegra_release ]; then
        GPU_ARGS=(--runtime=nvidia)
    else
        GPU_ARGS=(--gpus all)
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
    -e SCENE_PERCEPTION_WAIT_S="${SCENE_PERCEPTION_WAIT_S:-30}" \
    -e SCENE_OPEN_VOCAB_CLASSES="${SCENE_OPEN_VOCAB_CLASSES:-}" \
    -e VLM_BASE_URL="${VLM_BASE_URL:-}" \
    -e VLM_API_KEY="${VLM_API_KEY:-}" \
    -e VLM_MODEL="${VLM_MODEL:-}" \
    -e RBNX_CONFIG_FILE="${RBNX_CONFIG_FILE:-}" \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -v "$(pwd)":/scene \
    -v "$(rbnx path robonix-api)":/robonix-api:ro \
    "${EXTRA_MOUNTS[@]}" \
    "$IMG"
