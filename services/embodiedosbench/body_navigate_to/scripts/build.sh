#!/usr/bin/env bash
# Build: venv with robonix-api and grpcio, then codegen for this package's contracts.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
: "${ROBONIX_ROOT:?set ROBONIX_ROOT to the Robonix checkout}"
BUILD="$PKG/rbnx-build"; VENV="$BUILD/venv"
mkdir -p "$BUILD" "$PKG/capabilities/lib"
# Codegen does not follow symlinks, so the shared IDL is copied in.
rm -rf "$PKG/capabilities/lib/body" && cp -R "$PKG/../lib/body" "$PKG/capabilities/lib/body"
uv venv --allow-existing "$VENV"
uv pip install --python "$VENV/bin/python" --quiet "$ROBONIX_ROOT/pylib/robonix-api" "grpcio>=1.75" "protobuf>=6.31,<7" "grpcio-tools==1.75.1"
RBNX_CODEGEN_PYTHON="$VENV/bin/python" PATH="$VENV/bin:$PATH" rbnx codegen -p "$PKG" --mcp
