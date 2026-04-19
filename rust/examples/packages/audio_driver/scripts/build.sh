#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
PROTO_SRC="$PKG/proto"
OUT_DIR="$PKG/proto_gen"

mkdir -p "$OUT_DIR"

python -m grpc_tools.protoc \
    -I"$PROTO_SRC" \
    --python_out="$OUT_DIR" \
    --grpc_python_out="$OUT_DIR" \
    "$PROTO_SRC/audio_driver.proto"

echo "[build] done."
