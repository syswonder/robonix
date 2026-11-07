#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scene service build phase.
#
# v2 — scene runs in its own docker image (`robonix-scene`) joined to
# the host DDS bus. Build phase here:
#   1. rbnx codegen → rbnx-build/codegen/{proto_gen, robonix_mcp_types}
#      (still done on host because robonix-codegen is a Rust binary)
#   2. docker build the scene image, baking in scene's Python deps
#      (no host venv needed at runtime)
#
# CLEAN=1 forces a full rebuild incl. nuking rbnx-build and
# `docker build --no-cache`.
#
# HTTP proxy:
#   If you need proxy during docker build, export proxy variables on the host:
#
#     export http_proxy=http://127.0.0.1:7890
#     export https_proxy=http://127.0.0.1:7890
#
#   This script uses `docker build --network=host`, so 127.0.0.1 inside
#   the build container refers to the host network namespace on Linux.
#   Therefore we intentionally do NOT rewrite 127.0.0.1/localhost to
#   host.docker.internal here.
#
#   To force-disable proxy even when your shell has proxy env vars:
#
#     RBNX_BUILD_PROXY=0 ./this-script.sh
#
#   Proxy forwarding is opt-in; use existing proxy variables with:
#
#     RBNX_BUILD_PROXY=auto ./this-script.sh

set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
REPO_ROOT="$(cd "$PKG/../.." && pwd)"
# shellcheck disable=SC1091
source "$REPO_ROOT/scripts/docker_base_image.sh"
cd "$PKG"

BUILD="rbnx-build"
CLEAN="${RBNX_BUILD_CLEAN:-}"
IMG="${ROBONIX_SCENE_IMAGE:-robonix-scene}"
# Deployment target (same scheme as mapping_rbnx). Chosen by the per-target
# package manifest's `build:` line:
#   x86-docker     x86_64 + docker, ROS2 + cu128 torch in image  [default]
#   jetson-docker  arm64 Jetson + docker, L4T base (docker/Dockerfile.jetson)
#   jetson-native  host ROS 2, isolated application environment and GPU checks.
TARGET="${RBNX_BUILD_TARGET:-x86-docker}"
INFERENCE="${SCENE_CG_INFERENCE:-torch}"
INFERENCE="$(printf '%s' "$INFERENCE" | tr '[:upper:]' '[:lower:]' | xargs)"
export SCENE_CG_INFERENCE="$INFERENCE"
case "$INFERENCE" in
    torch) ;;
    tensorrt)
        export SCENE_PERCEPTION_BACKEND="${SCENE_PERCEPTION_BACKEND:-concept_graphs}"
        ;;
    *) echo "[build] SCENE_CG_INFERENCE must be torch or tensorrt" >&2; exit 2 ;;
esac


# ROS distro the scene image is built against. Robonix does not bind to a
# single ROS release: pick it here and the Dockerfile threads it through
# the base image, the ros-<distro>-* apt packages, and the runtime
# setup.bash. Supported: humble (default, verified), iron, jazzy, rolling.
# Set it before the first build, e.g.
#   ROBONIX_SCENE_ROS_DISTRO=iron rbnx build -p system/scene
# (`rbnx build` inherits the shell environment, so the variable reaches this
# script unchanged).
ROS_DISTRO_BUILD="${ROBONIX_SCENE_ROS_DISTRO:-humble}"
UPSTREAM_ROS_BASE_IMAGE="ros:${ROS_DISTRO_BUILD}-ros-base"
DEFAULT_ROS_BASE_IMAGE="robonix-ros:${ROS_DISTRO_BUILD}-ros-base"
ROS_BASE_IMAGE="${ROBONIX_SCENE_ROS_BASE_IMAGE:-$DEFAULT_ROS_BASE_IMAGE}"
# Empty means use the Docker/BuildKit daemon's bundled Dockerfile frontend.
# A non-empty image reference is forwarded through BuildKit's special
# pre-parse BUILDKIT_SYNTAX argument (for a pinned digest or registry mirror).
SCENE_BUILDKIT_SYNTAX="${ROBONIX_SCENE_BUILDKIT_SYNTAX:-}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"

if [[ "$TARGET" == "jetson-native" ]]; then
    # Serialize dependency installation through model preparation/publication.
    exec 9>"$BUILD/.native-build.lock"
    flock 9
    SETUP_PYTHON="${SCENE_NATIVE_PYTHON:-/usr/bin/python3}"
    if [[ ! -x "$SETUP_PYTHON" && "$SETUP_PYTHON" == "$PKG/$BUILD/native/"* ]]; then
        SETUP_PYTHON=/usr/bin/python3
    fi
    "$SETUP_PYTHON" "$PKG/scripts/prepare_native.py" --setup
    source "$BUILD/native-candidate.sh"
    export PATH="$(dirname "$SCENE_NATIVE_PYTHON"):$PATH"
    export RBNX_CODEGEN_PYTHON="$SCENE_NATIVE_PYTHON"
    PIP_IDX="${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"
    for req in scene-base scene-perception-native; do
        "$SCENE_NATIVE_PYTHON" -m pip install -r "docker/requirements/${req}.txt" --index-url "$PIP_IDX"
    done
    "$SCENE_NATIVE_PYTHON" -m pip install --no-deps -r docker/requirements/scene-viewer.txt --index-url "$PIP_IDX"
    bash scripts/prepare_viewer.sh "$(dirname "$(dirname "$SCENE_NATIVE_PYTHON")")/rerun-web-viewer"
    if [[ "${SCENE_PERCEPTION_BACKEND:-}" == dualmap ]]; then
        "$SCENE_NATIVE_PYTHON" -m pip install -r docker/requirements/scene-perception-dualmap.txt --index-url "$PIP_IDX"
    fi
    if [[ "$INFERENCE" == "tensorrt" ]]; then
        for req in scene-perception-export scene-perception-engines; do
            "$SCENE_NATIVE_PYTHON" -m pip install -r "docker/requirements/$req.txt" --index-url "$PIP_IDX"
        done
    fi

fi


# ── 1. Codegen (.proto + grpc stubs + MCP dataclasses → rbnx-build/codegen/) ─
# MapLifecycle needs a generated ROS message overlay so Scene can follow
# Mapping's map identity broadcasts on both Native and Docker deployments.
FLAGS=(--mcp --ros2)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[build] rbnx codegen ${FLAGS[*]}"
# RBNX_CODEGEN_POLICY_EXEMPT: Scene's host codegen stages IDL/ROS artifacts.
# Docker runtime Python stubs are regenerated and import-validated offline by
# scripts/start.sh with the exact interpreter and libraries baked into $IMG.
rbnx codegen -p "$PKG" "${FLAGS[@]}"
if [[ "$TARGET" == "jetson-native" ]]; then
    "$SCENE_NATIVE_PYTHON" - "$ROS_DISTRO" <<'PY'
import sys
from scripts.prepare_native import prepare_ros_overlay
prepare_ros_overlay(sys.argv[1])
PY
fi

# ── 1.5 Pre-fetch model weights onto host ──────────────────────────────────
# Download large weights on the host with curl retries and resume support.
# COPY the cached files into the image.
#
# RBNX_GH_MIRROR:
#   Prefix prepended to github.com URLs.
#   Default: https://ghfast.top/
#   Set to empty string to disable mirror and hit GitHub directly:
#
#     RBNX_GH_MIRROR= ./this-script.sh
#
WEIGHTS_DIR="$PKG/docker/_weights"
MODEL_CACHE_DIR="${ROBONIX_MODEL_CACHE_DIR:-$WEIGHTS_DIR}"
mkdir -p "$WEIGHTS_DIR" "$MODEL_CACHE_DIR"

GH_MIRROR="${RBNX_GH_MIRROR-https://ghfast.top/}"
HF_MIRROR="${RBNX_HF_MIRROR-${HF_ENDPOINT:-https://hf-mirror.com}}"
DOWNLOAD_CONNECT_TIMEOUT="${RBNX_MODEL_DOWNLOAD_CONNECT_TIMEOUT:-20}"
DOWNLOAD_MAX_TIME="${RBNX_MODEL_DOWNLOAD_MAX_TIME:-600}"
DOWNLOAD_RETRIES="${RBNX_MODEL_DOWNLOAD_RETRIES:-3}"
DOWNLOAD_SPEED_TIME="${RBNX_MODEL_DOWNLOAD_SPEED_TIME:-60}"
DOWNLOAD_SPEED_LIMIT="${RBNX_MODEL_DOWNLOAD_SPEED_LIMIT:-1024}"

copy_cached_weight() {
    local src="$1"
    local dest="$2"
    mkdir -p "$(dirname "$dest")"
    if [[ "$src" == "$dest" || "$src" -ef "$dest" ]]; then
        return 0
    fi
    cp -f "$src" "$dest"
}

fetch_weight_candidates() {
    local dest="$1"
    shift
    local name
    name="$(basename "$dest")"
    local cached="$MODEL_CACHE_DIR/$name"

    if [[ -s "$dest" ]]; then
        echo "[build] weight already present: $name"
        return 0
    fi
    if [[ -s "$cached" ]]; then
        echo "[build] using cached weight: $cached"
        copy_cached_weight "$cached" "$dest"
        return 0
    fi

    local source_url
    local tmp="$cached.part"
    for source_url in "$@"; do
        [[ -z "$source_url" ]] && continue
        echo "[build] downloading $name from $source_url"
        if curl -fL \
                -C - \
                --connect-timeout "$DOWNLOAD_CONNECT_TIMEOUT" \
                --max-time "$DOWNLOAD_MAX_TIME" \
                --retry "$DOWNLOAD_RETRIES" \
                --retry-all-errors \
                --retry-delay 5 \
                --speed-time "$DOWNLOAD_SPEED_TIME" \
                --speed-limit "$DOWNLOAD_SPEED_LIMIT" \
                -o "$tmp" \
                "$source_url"; then
            mv -f "$tmp" "$cached"
            copy_cached_weight "$cached" "$dest"
            return 0
        fi
        echo "[build] download candidate failed; keeping partial file for resume" >&2
    done

    rm -f "$dest"
    echo "[build] error: failed to download $name" >&2
    echo "[build]        partial download retained at $tmp" >&2
    echo "[build]        set ROBONIX_MODEL_CACHE_DIR to a directory containing $name to build offline" >&2
    exit 1
}

fetch_weight() {
    local url="$1"
    local dest="$2"
    local primary="$url"
    local candidates=()

    if [[ -n "$GH_MIRROR" ]]; then
        primary="${GH_MIRROR%/}/$url"
    fi
    candidates+=("$primary")
    [[ "$primary" != "$url" ]] && candidates+=("$url")
    fetch_weight_candidates "$dest" "${candidates[@]}"
}

fetch_hf_weight() {
    local repo="$1"
    local filename="$2"
    local dest="$3"
    local direct="https://huggingface.co/${repo}/resolve/main/${filename}"
    local candidates=()

    if [[ -n "$HF_MIRROR" ]]; then
        local mirrored="${HF_MIRROR%/}/${repo}/resolve/main/${filename}"
        candidates+=("$mirrored")
    fi
    if [[ "${candidates[0]:-}" != "$direct" ]]; then
        candidates+=("$direct")
    fi
    fetch_weight_candidates "$dest" "${candidates[@]}"
}

SCENE_PROFILE="$(printf '%s' "${SCENE_PROFILE:-lite}" | tr '[:upper:]' '[:lower:]' | xargs)"
export SCENE_PROFILE
MODEL_PATHS_TEXT="$(PYTHONPATH="$PKG:${PYTHONPATH:-}" python3 - "$WEIGHTS_DIR" <<'PYMODELS'
import sys
from scene_service.inference_config import model_config, model_defaults
for models in (model_config(weights_dir=sys.argv[1]), model_defaults(weights_dir=sys.argv[1])):
    for key in ("yolo", "sam", "clip_pretrained"):
        print(models[key])
PYMODELS
)"
mapfile -t MODEL_PATHS <<< "$MODEL_PATHS_TEXT"
for variable in SCENE_YOLO_WORLD_WEIGHTS SCENE_MOBILE_SAM_WEIGHTS SCENE_CLIP_PRETRAINED SCENE_DUALMAP_EXPORT_CLIP; do
    if [[ -n "${!variable:-}" && ! -s "${!variable}" ]]; then
        echo "[build] error: $variable must name an existing local checkpoint: ${!variable}" >&2
        exit 1
    fi
done

fetch_weight \
    "https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8l-world.pt" \
    "${MODEL_PATHS[0]}"

# The x86 image also carries Lite weights for its default runtime configuration.
if [[ "$SCENE_PROFILE" != full || "${SCENE_PERCEPTION_BACKEND:-}" == dualmap || "$TARGET" == x86-docker ]]; then
    SAM_DEST="${MODEL_PATHS[1]}"
    CLIP_DEST="${MODEL_PATHS[2]}"
    if [[ "$SCENE_PROFILE" == full && "${SCENE_PERCEPTION_BACKEND:-}" != dualmap ]]; then
        SAM_DEST="$WEIGHTS_DIR/mobile_sam.pt"
        CLIP_DEST="$WEIGHTS_DIR/open_clip_pytorch_model.bin"
    fi
    if [[ "$TARGET" == x86-docker && "${SCENE_PERCEPTION_BACKEND:-}" == dualmap ]]; then
        CLIP_DEST="$WEIGHTS_DIR/open_clip_pytorch_model.bin"
    fi
    fetch_weight \
        "https://github.com/ChaoningZhang/MobileSAM/raw/master/weights/mobile_sam.pt" \
        "$SAM_DEST"
    if [[ "${SCENE_PERCEPTION_BACKEND:-}" == dualmap && "$TARGET" != x86-docker ]]; then
        fetch_hf_weight "apple/MobileCLIP-S2-OpenCLIP" "open_clip_pytorch_model.bin" "$CLIP_DEST"
    else
        fetch_hf_weight "laion/CLIP-ViT-B-32-laion2B-s34B-b79K" "open_clip_pytorch_model.bin" "$CLIP_DEST"
    fi
fi
if [[ "$SCENE_PROFILE" == full && "${SCENE_PERCEPTION_BACKEND:-}" != dualmap ]]; then
    fetch_weight \
        "https://github.com/ultralytics/assets/releases/download/v8.3.0/sam_l.pt" \
        "${MODEL_PATHS[1]}"
    fetch_hf_weight \
        "laion/CLIP-ViT-H-14-laion2B-s32B-b79K" \
        "open_clip_pytorch_model.bin" \
        "${MODEL_PATHS[2]}"
fi

if [[ "${SCENE_PERCEPTION_BACKEND:-}" == dualmap && "$TARGET" != x86-docker ]]; then
    FASTSAM_DEST="$(python3 -c 'from scene_service.inference_config import model_config; print(model_config()["fastsam"])')"
    fetch_weight "https://github.com/ultralytics/assets/releases/download/v8.3.0/FastSAM-s.pt" "$FASTSAM_DEST"
    REPLICA_CLIP_DEST="$(python3 -c 'from scene_service.inference_config import model_config; print(model_config()["replica_clip_pretrained"])')"
    fetch_hf_weight "laion/CLIP-ViT-B-32-laion2B-s34B-b79K" "open_clip_pytorch_model.bin" "$REPLICA_CLIP_DEST"
fi

if [[ "$TARGET" == x86-docker ]]; then
    FULL_CONTEXT="$PKG/docker/_build/full-models"
    rm -rf "$FULL_CONTEXT"
    mkdir -p "$FULL_CONTEXT"
    copy_cached_weight "${MODEL_PATHS[0]}" "$WEIGHTS_DIR/yolov8l-world.pt"
    if [[ "$SCENE_PROFILE" == full && "${SCENE_PERCEPTION_BACKEND:-}" != dualmap ]]; then
        copy_cached_weight "${MODEL_PATHS[1]}" "$FULL_CONTEXT/$(basename "${MODEL_PATHS[4]}")"
        copy_cached_weight "${MODEL_PATHS[2]}" "$FULL_CONTEXT/$(basename "${MODEL_PATHS[5]}")"
    else
        copy_cached_weight "${MODEL_PATHS[1]}" "${MODEL_PATHS[4]}"
        copy_cached_weight "${MODEL_PATHS[2]}" "${MODEL_PATHS[5]}"
    fi
fi

if [[ "$TARGET" == "jetson-docker" ]]; then
    python3 "$PKG/scripts/prepare_docker.py" --build
    exit 0
fi

# ── 2a. Native dependencies inside the selected Scene environment ────────
if [[ "$TARGET" == "jetson-native" ]]; then
    PY="$SCENE_NATIVE_PYTHON"
    # Validate the host-staged open_clip checkpoint without contacting the
    # Hugging Face metadata API. Also pre-warm Ultralytics YOLO-World's text
    # encoder path:
    # YOLOWorld.set_classes() uses openai/clip, not open_clip's HF cache.
    # If this is left to start_native, first boot can
    # spend minutes downloading ViT-B-32.pt and miss the scene/object test flow.
    YCD="${YOLO_CONFIG_DIR:-$PKG/rbnx-build/data/ultralytics}"
    UWD="$YCD/weights"
    mkdir -p "$YCD" "$UWD"
    "$PY" -c "from scene_service.inference_config import model_config; from scene_service.ingest.perception_models import create_clip_model; m=model_config(); create_clip_model(m['clip_model'], m['clip_pretrained'])" \
        || {
            echo "[build] error: open_clip could not load the selected model; check SCENE_PROFILE and model paths" >&2
            exit 1
        }
    YOLO_CONFIG_DIR="$YCD" "$PY" <<PY
from ultralytics.utils import SETTINGS

SETTINGS.update({
    "weights_dir": "$UWD",
    "datasets_dir": "$YCD/datasets",
    "runs_dir": "$YCD/runs",
    "sync": False,
})
PY
    YOLO_CONFIG_DIR="$YCD" SCENE_YOLO_WORLD_WEIGHTS="${MODEL_PATHS[0]}" HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}" "$PY" <<PY
import os
from pathlib import Path
from ultralytics.utils import WEIGHTS_DIR

expected = Path("$UWD")
if Path(WEIGHTS_DIR) != expected:
    raise RuntimeError(f"Ultralytics WEIGHTS_DIR={WEIGHTS_DIR!s}, expected {expected!s}")

from ultralytics import YOLO

classes = [
    "chair", "table", "desk", "couch", "sofa", "bookshelf", "shelf",
    "cabinet", "drawer", "whiteboard",
    "monitor", "laptop", "keyboard", "mouse", "computer tower",
    "monitor stand", "headphones", "webcam", "router", "power strip",
    "cup", "mug", "water bottle", "thermos", "paper cup",
    "backpack", "handbag", "book", "notebook", "pen", "pencil",
    "phone", "tablet",
    "box", "cardboard box", "tray", "basket", "trash bin",
    "tool", "screwdriver", "wrench", "tape",
    "plant", "potted plant", "lamp", "clock", "picture frame",
    "snack", "fruit", "apple", "banana",
    "door", "doorway", "fire extinguisher", "person",
]

model = YOLO(os.environ["SCENE_YOLO_WORLD_WEIGHTS"])
model.set_classes(classes)
PY
    # Compile before service startup: Atlas registration has a short deadline.
    if [[ "$INFERENCE" == "tensorrt" ]]; then
        YOLO_CONFIG_DIR="$YCD" "$PY" "$PKG/scripts/prepare_models.py"
    fi
    "$PY" "$PKG/scripts/prepare_native.py" --verify
    echo "[build] done (jetson-native)."
    exit 0
fi

# ── 2. Docker image (scene's Python deps + ROS Humble base) ────────────────
if ! command -v docker >/dev/null 2>&1; then
    echo "[build] error: target $TARGET needs docker on PATH" >&2
    exit 1
fi
SCENE_DOCKERFILE="docker/Dockerfile"

DOCKER_BUILD_FLAGS=(
    --network=host
    --build-arg "SCENE_CG_INFERENCE=$INFERENCE"
    --pull=false
    --build-arg "ROS_DISTRO=${ROS_DISTRO_BUILD}"
    --build-arg "ROS_BASE_IMAGE=${ROS_BASE_IMAGE}"
    --build-arg "GITHUB_PREFIX=${GH_MIRROR:+${GH_MIRROR%/}/}"
)
if [[ -n "$SCENE_BUILDKIT_SYNTAX" ]]; then
    DOCKER_BUILD_FLAGS+=(--build-arg "BUILDKIT_SYNTAX=${SCENE_BUILDKIT_SYNTAX}")
    echo "[build] Dockerfile frontend override: ${SCENE_BUILDKIT_SYNTAX}"
else
    echo "[build] Dockerfile frontend: builder-bundled (no frontend registry lookup)"
fi
[[ "$CLEAN" == "1" ]] && DOCKER_BUILD_FLAGS+=(--no-cache)
echo "[build] ROS distro: ${ROS_DISTRO_BUILD} (set ROBONIX_SCENE_ROS_DISTRO to change)"
echo "[build] ROS base image: ${ROS_BASE_IMAGE} (set ROBONIX_SCENE_ROS_BASE_IMAGE to override)"

# Keep the default base image local. A Dockerfile FROM that points at a remote
# tag makes BuildKit query registry metadata on every rebuild, even when the
# layers are cached. The local alias removes that registry hit from normal
# rebuilds while still preserving an explicit override for mirrors/digests.
if [[ "$ROS_BASE_IMAGE" == "$DEFAULT_ROS_BASE_IMAGE" ]]; then
    robonix_ensure_local_base_image "$ROS_BASE_IMAGE" "$UPSTREAM_ROS_BASE_IMAGE"
fi

# Proxy → docker build-args.
#
# Important:
#   We pass HTTP_PROXY_HOST / HTTPS_PROXY_HOST instead of Docker's special
#   HTTP_PROXY / HTTPS_PROXY build args.
#
#   This avoids BuildKit treating them as global proxy args implicitly.
#   The Dockerfile can then opt in only for selected heavy network layers
#   such as apt/pip/git/HuggingFace.
#
#   Because we use --network=host, keep 127.0.0.1 unchanged.
#
# Default to no proxy: most CI/self-hosted environments work better when
# configured mirrors are reached directly. Picking up the host's http_proxy
# would tunnel large wheel downloads (torch, ~1 GB) through a local proxy and
# can stall them. Opt back in with RBNX_BUILD_PROXY=1 only when the host
# genuinely needs a proxy for egress.
USE_PROXY="${RBNX_BUILD_PROXY:-0}"

_http_proxy="${HTTP_PROXY:-${http_proxy:-}}"
_https_proxy="${HTTPS_PROXY:-${https_proxy:-}}"
_no_proxy="${NO_PROXY:-${no_proxy:-}}"

_docker_http=""
_docker_https=""

case "$USE_PROXY" in
    0|false|False|FALSE|no|No|NO)
        echo "[build] proxy disabled by RBNX_BUILD_PROXY=$USE_PROXY"
        ;;
    1|true|True|TRUE|yes|Yes|YES|auto)
        _docker_http="$_http_proxy"
        _docker_https="$_https_proxy"

        if [[ -n "$_docker_http" || -n "$_docker_https" ]]; then
            echo "[build] proxy enabled:"
            echo "[build]   HTTP_PROXY_HOST=${_docker_http:-<unset>}"
            echo "[build]   HTTPS_PROXY_HOST=${_docker_https:-<unset>}"

            if [[ "$_docker_http" == *host.docker.internal* || "$_docker_https" == *host.docker.internal* ]]; then
                echo "[build] warning: proxy URL contains host.docker.internal while using --network=host." >&2
                echo "[build] warning: for this script, prefer http://127.0.0.1:7890 on Linux." >&2
            fi

            [[ -n "$_docker_http" ]] && DOCKER_BUILD_FLAGS+=(--build-arg "HTTP_PROXY_HOST=${_docker_http}")
            [[ -n "$_docker_https" ]] && DOCKER_BUILD_FLAGS+=(--build-arg "HTTPS_PROXY_HOST=${_docker_https}")
            [[ -n "$_no_proxy" ]] && DOCKER_BUILD_FLAGS+=(--build-arg "NO_PROXY_HOST=${_no_proxy}")
        else
            echo "[build] proxy env not set; building without proxy"
        fi
        ;;
    *)
        echo "[build] error: invalid RBNX_BUILD_PROXY=$USE_PROXY" >&2
        echo "[build] expected one of: auto, 1, 0, true, false, yes, no" >&2
        exit 1
        ;;
esac

echo "[build] docker build -f $SCENE_DOCKERFILE -t $IMG docker/  (target=$TARGET)"
docker build "${DOCKER_BUILD_FLAGS[@]}" -f "$SCENE_DOCKERFILE" -t "$IMG" docker/

TRT_IMG="$IMG"
if [[ "$TARGET" == "x86-docker" && "${SCENE_PERCEPTION_BACKEND:-}" != "concept_graphs" ]]; then
    echo "[build] DualMap layer -> robonix-scene-dualmap"
    rm -rf docker/_build/dualmap
    python3 scripts/prepare_dualmap.py --root docker/_build/dualmap
    if [[ -n "${SCENE_DUALMAP_EXPORT_CLIP:-}" ]]; then
        copy_cached_weight "$SCENE_DUALMAP_EXPORT_CLIP" docker/_build/dualmap/.scene-replica-clip.bin
        chmod a+r docker/_build/dualmap/.scene-replica-clip.bin
    fi
    docker build "${DOCKER_BUILD_FLAGS[@]}" -f docker/Dockerfile.dualmap \
        --build-arg BASE_IMAGE="$IMG" \
        --build-arg HF_ENDPOINT="$HF_MIRROR" \
        -t robonix-scene-dualmap docker/
    TRT_IMG=robonix-scene-dualmap
fi

if [[ "$INFERENCE" == "tensorrt" ]]; then
    # GPU compilation runs after image construction, before service replacement.
    TRT_GPU=(--gpus all)
    mkdir -p "$BUILD/tensorrt"
    docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e USER=scene -e LOGNAME=scene "${TRT_GPU[@]}" -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
        -e SCENE_CG_INFERENCE=tensorrt -e SCENE_ONNX_DIR=/scene-models/onnx \
        -e SCENE_ENGINE_CACHE_DIR=/scene-models/engines \
        -e SCENE_PROFILE="${SCENE_PROFILE:-lite}" \
        -e SCENE_PERCEPTION_BACKEND="$SCENE_PERCEPTION_BACKEND" \
        -e SCENE_CLIP_MODEL="${SCENE_CLIP_MODEL:-}" \
        -e SCENE_CG_FORCE_CPU="${SCENE_CG_FORCE_CPU:-}" \
        -v "$PKG:/scene:ro" -v "$PKG/$BUILD/tensorrt:/scene-models" \
        --entrypoint bash "$TRT_IMG" -ec '
            mkdir -p /tmp/Ultralytics
            cp /opt/ultralytics/Ultralytics/settings.json /tmp/Ultralytics/
            export YOLO_CONFIG_DIR=/tmp
            exec "$@"
        ' scene-prepare python3 /scene/scripts/prepare_models.py
fi


# colcon-build the ros2_idl overlay (map interface package for the
# lifecycle broadcast) inside the image we just built, so the install
# tree lands on the host bind mount at the SAME path the runtime
# container sees (/scene/rbnx-build/...). Skipped when codegen was
# skipped — scene then runs with static map binding only.
IDL="$PKG/rbnx-build/codegen/ros2_idl"
if [[ -d "$IDL/src/map" ]]; then
    echo "[build] colcon build ros2_idl (map interface pkg) in $IMG"
    # --user: build/ install/ land on the HOST bind mount — as root they
    # would survive a later non-root `rm -rf rbnx-build` (RBNX_BUILD_CLEAN).
    # HOME=/tmp gives colcon a writable home for its metadata as that uid.
    docker run --rm --entrypoint bash --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -v "$PKG":/scene "$IMG" -lc \
        "source /opt/ros/\${ROS_DISTRO:-humble}/setup.bash && \
         cd /scene/rbnx-build/codegen/ros2_idl && \
         colcon build --packages-up-to map"
else
    echo "[build] WARNING: ros2_idl/src/map missing — dynamic map binding disabled"
fi

echo "[build] done."
