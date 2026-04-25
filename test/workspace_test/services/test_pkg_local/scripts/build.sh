#!/usr/bin/env bash
# Build script for test_pkg_local
# Called by: rbnx build
# Working directory: package root

set -euo pipefail

echo "Building test_pkg_local ..."

if [ "${RBNX_BUILD_CLEAN:-}" = "1" ]; then
    echo "Clean build requested — removing rbnx-build/"
    rm -rf rbnx-build
fi

mkdir -p rbnx-build

echo "Build complete."
