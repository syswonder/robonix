#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Build script for the zero_copy_demo package.
# Run `rbnx setup` once from the robonix source root first.
set -euo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
RUST_ROOT="$(rbnx path rust)"

FLAGS=()
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)

echo "[build] building librobonix_buffer.so..."
(cd "$RUST_ROOT" && cargo build -p robonix-buffer --release)

echo "[build] installing Python package (editable)..."
if command -v uv >/dev/null 2>&1; then
    (cd "$PKG_ROOT" && uv pip install -e .)
else
    (cd "$PKG_ROOT" && pip install -e .)
fi

rbnx codegen -p "$PKG_ROOT" "${FLAGS[@]}"

touch "$PKG_ROOT/rbnx-build/.rbnx-built"
echo "[build] done."
