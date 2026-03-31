#!/usr/bin/env bash
set -euo pipefail

# Build script for the zero_copy_demo package.
# Called by `rbnx build -p zero_copy_demo`.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
RUST_ROOT="$(cd "$PKG_ROOT/../../../.." && pwd)"

echo "[build] Building librobonix_buffer.so..."
(cd "$RUST_ROOT" && cargo build -p robonix-buffer --release)

echo "[build] Installing Python package (editable)..."
if command -v uv &>/dev/null; then
    (cd "$PKG_ROOT" && uv pip install -e .)
else
    (cd "$PKG_ROOT" && pip install -e .)
fi

mkdir -p "$PKG_ROOT/rbnx-build"
touch "$PKG_ROOT/rbnx-build/.rbnx-built"
echo "[build] Done."
