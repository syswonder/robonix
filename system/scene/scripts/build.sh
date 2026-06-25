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
#   To use proxy automatically when env vars exist:
#
#     ./this-script.sh

set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

BUILD="rbnx-build"
CLEAN="${RBNX_BUILD_CLEAN:-}"
IMG="${ROBONIX_SCENE_IMAGE:-robonix-scene}"

# ROS distro the scene image is built against. Robonix does not bind to a
# single ROS release: pick it here and the Dockerfile threads it through
# the base image, the ros-<distro>-* apt packages, and the runtime
# setup.bash. Supported: humble (default, verified), iron, jazzy, rolling.
# Set it before the first build, e.g.
#   ROBONIX_SCENE_ROS_DISTRO=iron rbnx build -p system/scene
# (`rbnx build` inherits the shell environment, so the variable reaches this
# script unchanged).
ROS_DISTRO_BUILD="${ROBONIX_SCENE_ROS_DISTRO:-humble}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"

# ── 1. Codegen (.proto + grpc stubs + MCP dataclasses → rbnx-build/codegen/) ─
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

# ── 1.5 Pre-fetch model weights onto host ──────────────────────────────────
# Pulled out of the docker build because github CDN connections from CN
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
mkdir -p "$WEIGHTS_DIR"

GH_MIRROR="${RBNX_GH_MIRROR-https://ghfast.top/}"

fetch_weight() {
    local url="$1"
    local dest="$2"

    if [[ -s "$dest" ]]; then
        echo "[build] weight already present: $(basename "$dest")"
        return 0
    fi

    local primary="$url"
    if [[ -n "$GH_MIRROR" ]]; then
        primary="${GH_MIRROR%/}/$url"
    fi

    echo "[build] downloading $(basename "$dest") from $primary"

    if curl -fL \
            --connect-timeout 30 \
            --retry 5 \
            --retry-all-errors \
            --retry-delay 5 \
            -o "$dest" \
            "$primary"; then
        return 0
    fi

    rm -f "$dest"

    if [[ "$primary" != "$url" ]]; then
        echo "[build] mirror failed; falling back to direct: $url" >&2

        if curl -fL \
                --connect-timeout 30 \
                --retry 5 \
                --retry-all-errors \
                --retry-delay 5 \
                -o "$dest" \
                "$url"; then
            return 0
        fi

        rm -f "$dest"
    fi

    echo "[build] error: failed to download $url" >&2
    exit 1
}

fetch_weight \
    "https://github.com/ultralytics/assets/releases/download/v8.1.0/yolov8l-world.pt" \
    "$WEIGHTS_DIR/yolov8l-world.pt"

fetch_weight \
    "https://github.com/ChaoningZhang/MobileSAM/raw/master/weights/mobile_sam.pt" \
    "$WEIGHTS_DIR/mobile_sam.pt"

# ── 2. Docker image (scene's Python deps + ROS Humble base) ────────────────
if ! command -v docker >/dev/null 2>&1; then
    echo "[build] error: docker not found on PATH" >&2
    exit 1
fi

DOCKER_BUILD_FLAGS=(--network=host --build-arg "ROS_DISTRO=${ROS_DISTRO_BUILD}")
[[ "$CLEAN" == "1" ]] && DOCKER_BUILD_FLAGS+=(--no-cache)
echo "[build] ROS distro: ${ROS_DISTRO_BUILD} (set ROBONIX_SCENE_ROS_DISTRO to change)"

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
# Default to no proxy: this repo targets domestic (CN) networks where the
# aliyun / tuna mirrors are reached directly and fast. Picking up the host's
# http_proxy (the old `auto` default) would tunnel large wheel downloads
# (torch, ~1 GB) through a local proxy and stall them. Opt back in with
# RBNX_BUILD_PROXY=1 only when the host genuinely needs a proxy for egress.
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

echo "[build] docker build -t $IMG docker/"
docker build "${DOCKER_BUILD_FLAGS[@]}" -t "$IMG" docker/

echo "[build] done."