#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scene service build phase.
#
# v2 — scene runs in its own docker image (`robonix-scene`) joined to
# the host DDS bus. Build phase here:
#   1. exact-version Python codegen →
#      rbnx-build/codegen/{proto_gen, robonix_mcp_types}
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
#   To use proxy automatically when env vars exist:
#
#     ./this-script.sh

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
#   jetson-native  arm64 Jetson + host ROS2 + host JetPack torch — no docker;
#                  builds rbnx-build/venv (--system-site-packages) for the
#                  light pure-python deps only.
TARGET="${RBNX_BUILD_TARGET:-x86-docker}"

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

# ── 1. Codegen (.proto + grpc stubs + MCP dataclasses → rbnx-build/codegen/) ─
# --ros2 also emits rbnx-build/codegen/ros2_idl: the generated ROS 2
# interface overlay carrying map/msg/MapLifecycle (mapping's latched map
# identity broadcast, which scene's map binding subscribes to). It is
# colcon-built inside the scene image after the docker build below and
# sourced by docker/entrypoint.sh; without it scene falls back to static
# map binding (SCENE_MAP_ID / config) with a warning.
FLAGS=(--mcp --ros2)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[build] rbnx codegen ${FLAGS[*]}"
bash "$PKG/scripts/run_python_codegen.sh" "$PKG" "${FLAGS[@]}"

# ── 1.5 Pre-fetch model weights onto host ──────────────────────────────────
# Pulled out of the docker build because direct CDN connections from some
# drop mid-stream on multi-hundred-MB transfers; an out-of-band download
# with curl --retry-all-errors is much more robust, and the resulting
# files become a cache-key-stable COPY into the image.
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
    if [[ "$src" == "$dest" ]]; then
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

fetch_weight \
    "https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8l-world.pt" \
    "$WEIGHTS_DIR/yolov8l-world.pt"

fetch_weight \
    "https://github.com/ChaoningZhang/MobileSAM/raw/master/weights/mobile_sam.pt" \
    "$WEIGHTS_DIR/mobile_sam.pt"

# open_clip normally resolves this symbolic pretrained name through the
# Hugging Face metadata API during the Docker build. Fetch the model file on
# the host instead, using the same retry/cache path as the other scene weights,
# then COPY it into both Docker variants. RBNX_HF_MIRROR controls the mirror;
# the canonical huggingface.co URL is always the fallback.
OPEN_CLIP_WEIGHTS="$WEIGHTS_DIR/open_clip_pytorch_model.bin"
fetch_hf_weight \
    "laion/CLIP-ViT-B-32-laion2B-s34B-b79K" \
    "open_clip_pytorch_model.bin" \
    "$OPEN_CLIP_WEIGHTS"

# ── 2a. jetson-native: host venv, no docker ────────────────────────────────
# The heavy CUDA wheels (torch / torchvision / ultralytics / open3d) come from
# the host JetPack stack via --system-site-packages; we only pip-install the
# light pure-python deps on top. Mirrors mapping_rbnx's native target.
if [[ "$TARGET" == "jetson-native" ]]; then
    # No venv on purpose. A --system-site-packages venv can't see the host
    # JetPack torch (it's typically a `pip install --user` in ~/.local), and
    # forcing the user-site onto PYTHONPATH drags shadowing shims (e.g. enum34)
    # ahead of the stdlib and breaks the interpreter. The host python already
    # has the correct sys.path order AND sees the JetPack torch — so install
    # scene's light pure-python deps into the user site alongside it.
    PY=python3
    PIP_IDX="${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}"
    echo "[build] jetson-native: host python ($("$PY" --version 2>&1)), pip install --user"
    "$PY" -c "import torch" 2>/dev/null || {
        echo "[build] error: host python has no torch — install the JetPack CUDA" >&2
        echo "        stack first (see scene README 'Jetson native prerequisites'):" >&2
        echo "        pip install --user --index-url https://pypi.jetson-ai-lab.dev/jp6/cu126 torch torchvision" >&2
        exit 1
    }
    "$PY" -m pip install --user --upgrade pip --index-url "$PIP_IDX" || true
    # scene-base contains the provider's RPC/MCP runtime. A failed install is
    # fatal: otherwise rbnx writes a successful build stamp and the native
    # process can die on generated imports before it registers with Atlas.
    BASE_REQ="docker/requirements/scene-base.txt"
    echo "[build] pip install --user -r $BASE_REQ"
    "$PY" -m pip install --user -r "$BASE_REQ" --index-url "$PIP_IDX"

    # Perception layers may degrade independently of the provider's core
    # lifecycle/MCP runtime, so retain their existing warning behavior.
    for req in scene-perception-core scene-perception-heavy; do
        f="docker/requirements/${req}.txt"
        [[ -f "$f" ]] || continue
        echo "[build] pip install --user -r $f"
        "$PY" -m pip install --user -r "$f" --index-url "$PIP_IDX" \
            || echo "[build] warning: some deps in $f failed (perception may degrade)"
    done
    # concept-graphs (perception backbone) — editable, no deps.
    CG="$PKG/rbnx-build/concept-graphs"
    GH="${RBNX_GH_MIRROR-https://ghfast.top/}"
    if [[ ! -d "$CG/.git" ]]; then
        echo "[build] cloning concept-graphs"
        git clone --depth 1 --branch ali-dev \
            "${GH%/}/https://github.com/concept-graphs/concept-graphs.git" "$CG" \
        || git clone --depth 1 --branch ali-dev \
            https://github.com/concept-graphs/concept-graphs.git "$CG"
    fi
    "$PY" -m pip install --user --no-deps -e "$CG" || echo "[build] warning: concept-graphs install failed"

    # Verify the actual native interpreter after every dependency install, not
    # only the isolated generator venv. Keep PYTHONPATH package-local and reject
    # stale modules from another package before writing a successful build
    # stamp; optional pip resolution must not have displaced the runtime pins.
    PROTO_ROOT="$PKG/rbnx-build/codegen/proto_gen"
    MCP_ROOT="$PKG/rbnx-build/codegen/robonix_mcp_types"
    PYTHONPATH="$PROTO_ROOT:$MCP_ROOT" \
        "$PY" "$PKG/scripts/verify_python_codegen.py" \
        "$PROTO_ROOT" "$MCP_ROOT" \
        "protobuf=6.33.6" \
        "grpcio=1.80.0"

    # Pre-fetch the CLIP weights into the host HF cache (start_native points
    # HF_HOME here). Also pre-warm Ultralytics YOLO-World's text encoder path:
    # YOLOWorld.set_classes() uses openai/clip via Ultralytics' weights_dir,
    # not open_clip's HF cache. If this is left to start_native, first boot can
    # spend minutes downloading ViT-B-32.pt and miss the scene/object test flow.
    YCD="$PKG/rbnx-build/data/ultralytics"
    UWD="$YCD/weights"
    mkdir -p "$YCD" "$UWD"
    SCENE_CLIP_PRETRAINED="$OPEN_CLIP_WEIGHTS" \
        "$PY" -c "import os, open_clip; open_clip.create_model_and_transforms('ViT-B-32', pretrained=os.environ['SCENE_CLIP_PRETRAINED'])" \
        || {
            echo "[build] error: open_clip could not load $OPEN_CLIP_WEIGHTS" >&2
            exit 1
        }
    YOLO_CONFIG_DIR="$YCD" "$PY" <<PY \
        || echo "[build] warning: failed to persist Ultralytics settings"
from ultralytics.utils import SETTINGS

SETTINGS.update({
    "weights_dir": "$UWD",
    "datasets_dir": "$YCD/datasets",
    "runs_dir": "$YCD/runs",
    "sync": False,
})
PY
    YOLO_CONFIG_DIR="$YCD" HF_ENDPOINT="${HF_ENDPOINT:-https://hf-mirror.com}" "$PY" <<PY \
        || echo "[build] warning: ultralytics CLIP prefetch failed (scene start will fail fast until build succeeds)"
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

model = YOLO("$WEIGHTS_DIR/yolov8l-world.pt")
model.set_classes(classes)
PY
    ULTRALYTICS_CLIP="$UWD/clip/ViT-B-32.pt"
    HF_CLIP="$HFD/clip/ViT-B-32.pt"
    if [[ ! -s "$ULTRALYTICS_CLIP" && -s "$HF_CLIP" ]]; then
        mkdir -p "$(dirname "$ULTRALYTICS_CLIP")"
        ln "$HF_CLIP" "$ULTRALYTICS_CLIP" 2>/dev/null \
            || cp "$HF_CLIP" "$ULTRALYTICS_CLIP"
        echo "[build] staged Ultralytics CLIP weight: $ULTRALYTICS_CLIP"
    fi
    if [[ ! -s "$ULTRALYTICS_CLIP" ]]; then
        echo "[build] error: missing $ULTRALYTICS_CLIP after prefetch" >&2
        exit 1
    fi
    "$PY" -c "import torch,torchvision,ultralytics; print('[build] torch',torch.__version__,'cuda',torch.cuda.is_available())" || true
    echo "[build] done (jetson-native)."
    exit 0
fi

# ── 2. Docker image (scene's Python deps + ROS Humble base) ────────────────
if ! command -v docker >/dev/null 2>&1; then
    echo "[build] error: target $TARGET needs docker on PATH" >&2
    exit 1
fi
# jetson-docker uses the L4T-based Dockerfile; x86-docker the default one.
SCENE_DOCKERFILE="docker/Dockerfile"
[[ "$TARGET" == "jetson-docker" ]] && SCENE_DOCKERFILE="docker/Dockerfile.jetson"

DOCKER_BUILD_FLAGS=(
    --network=host
    --pull=false
    --build-arg "ROS_DISTRO=${ROS_DISTRO_BUILD}"
    --build-arg "ROS_BASE_IMAGE=${ROS_BASE_IMAGE}"
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
