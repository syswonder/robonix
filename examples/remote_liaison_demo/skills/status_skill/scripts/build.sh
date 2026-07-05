#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"

mkdir -p rbnx-build/data
rbnx codegen -p "$PKG" --mcp
echo "[status_skill/build] done."
