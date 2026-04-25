#!/usr/bin/env bash
# Build script for vlm_service
# Called by: rbnx build
# Working directory: package root

set -euo pipefail

echo "Building vlm_service ..."

if [ "${RBNX_BUILD_CLEAN:-}" = "1" ]; then
    echo "Clean build requested — removing rbnx-build/"
    rm -rf rbnx-build
fi

mkdir -p rbnx-build

echo "Build complete."
