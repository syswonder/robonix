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
# HTTP proxy (Clash): export http_proxy/https_proxy on the host, e.g.
# http://127.0.0.1:7890. For `docker build`, loopback/localhost in those URLs
# is rewritten to host.docker.internal (+ --add-host=...:host-gateway). Clash
# should allow LAN and listen on 0.0.0.0.

set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

BUILD="rbnx-build"
GEN="$BUILD/codegen"
CLEAN="${RBNX_BUILD_CLEAN:-}"
IMG="${ROBONIX_SCENE_IMAGE:-robonix-scene}"

if [[ "$CLEAN" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
mkdir -p "$BUILD/data"

# ── 1. Codegen (.proto + grpc stubs + MCP dataclasses → rbnx-build/codegen/) ─
FLAGS=(--mcp --out-dir "$GEN")
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)
echo "[build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"

# ── 1.5 Pre-fetch model weights onto host ──────────────────────────────────
# Pulled out of the docker build because github CDN connections from CN
# drop mid-stream on multi-hundred-MB transfers; an out-of-band download
# with curl --retry-all-errors is much more robust, and the resulting
# files become a cache-key-stable COPY into the image.
WEIGHTS_DIR="$PKG/docker/_weights"
mkdir -p "$WEIGHTS_DIR"
fetch_weight() {
    local url="$1"
    local dest="$2"
    if [[ -s "$dest" ]]; then
        echo "[build] weight already present: $(basename "$dest")"
        return 0
    fi
    echo "[build] downloading $(basename "$dest") from $url"
    curl -fL --connect-timeout 30 --retry 5 --retry-all-errors --retry-delay 5 \
        -o "$dest" "$url" || {
        echo "[build] error: failed to download $url" >&2
        rm -f "$dest"
        exit 1
    }
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

DOCKER_BUILD_FLAGS=()
[[ "$CLEAN" == "1" ]] && DOCKER_BUILD_FLAGS+=(--no-cache)

# Proxy → docker build-args (see file-header comment).
_http_proxy="${HTTP_PROXY:-${http_proxy:-}}"
_https_proxy="${HTTPS_PROXY:-${https_proxy:-}}"
_no_proxy="${NO_PROXY:-${no_proxy:-}}"

proxy_for_docker_build() {
    local url="$1"
    [[ -z "$url" ]] && { printf ''; return; }
    case "$url" in
        *127.0.0.1*) url="${url//127.0.0.1/host.docker.internal}" ;;
        *localhost*) url="${url//localhost/host.docker.internal}" ;;
    esac
    printf '%s' "$url"
}

_docker_http="$(proxy_for_docker_build "$_http_proxy")"
_docker_https="$(proxy_for_docker_build "$_https_proxy")"

if [[ -n "$_docker_http" || -n "$_docker_https" ]]; then
    echo "[build] docker build: build-arg HTTP_PROXY=${_docker_http:-<unset>} HTTPS_PROXY=${_docker_https:-<unset>}"
    DOCKER_BUILD_FLAGS+=(--add-host=host.docker.internal:host-gateway)
    [[ -n "$_docker_http" ]] && DOCKER_BUILD_FLAGS+=(--build-arg "HTTP_PROXY=${_docker_http}")
    [[ -n "$_docker_https" ]] && DOCKER_BUILD_FLAGS+=(--build-arg "HTTPS_PROXY=${_docker_https}")
    [[ -n "$_no_proxy" ]] && DOCKER_BUILD_FLAGS+=(--build-arg "NO_PROXY=${_no_proxy}")
fi

# Skip the rebuild when the image already exists AND its layers are
# all cached — `docker build` is idempotent so this branch is just an
# optimisation; safe to remove if it ever surprises someone.
if [[ "$CLEAN" != "1" ]] && docker image inspect "$IMG" >/dev/null 2>&1; then
    echo "[build] image $IMG already present; rebuilding incrementally"
fi

echo "[build] docker build -t $IMG docker/"
docker build "${DOCKER_BUILD_FLAGS[@]}" -t "$IMG" docker/

echo "[build] done."
