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

# ── 2. Docker image (scene's Python deps + ROS Humble base) ────────────────
if ! command -v docker >/dev/null 2>&1; then
    echo "[build] error: docker not found on PATH" >&2
    exit 1
fi

DOCKER_BUILD_FLAGS=()
[[ "$CLEAN" == "1" ]] && DOCKER_BUILD_FLAGS+=(--no-cache)

# Skip the rebuild when the image already exists AND its layers are
# all cached — `docker build` is idempotent so this branch is just an
# optimisation; safe to remove if it ever surprises someone.
if [[ "$CLEAN" != "1" ]] && docker image inspect "$IMG" >/dev/null 2>&1; then
    echo "[build] image $IMG already present; rebuilding incrementally"
fi

echo "[build] docker build -t $IMG docker/"
docker build "${DOCKER_BUILD_FLAGS[@]}" -t "$IMG" docker/

echo "[build] done."
