#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
BUILD="$PKG/rbnx-build"
VENV="$BUILD/venv"

if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
    echo "[build] clean: removing $BUILD"
    rm -rf "$BUILD"
fi
if ! command -v uv >/dev/null 2>&1; then
    echo "[build] error: 'uv' not found on PATH. Install: https://docs.astral.sh/uv/" >&2
    exit 1
fi

uv venv --allow-existing --python "${AUDIO_DRIVER_PYTHON:-python3}" "$VENV"
ROBONIX_API="$(rbnx path robonix-api)"
uv pip install --python "$VENV/bin/python" --quiet \
    "$ROBONIX_API" -r "$PKG/requirements.txt" \
    "grpcio==1.80.0" "grpcio-tools==1.76.0" "protobuf==6.33.6"

RBNX_CODEGEN_PYTHON="$VENV/bin/python" \
    PATH="$VENV/bin:$PATH" \
    rbnx codegen -p "$PKG"

CODEGEN_ROOT="$BUILD/codegen"
PYTHONPATH="$ROBONIX_API:$PKG:$CODEGEN_ROOT/proto_gen:$CODEGEN_ROOT/robonix_mcp_types:${PYTHONPATH:-}" \
    "$VENV/bin/python" -c 'import audio_pb2, std_msgs_pb2; import audio_driver.main'
echo "[build] done."
