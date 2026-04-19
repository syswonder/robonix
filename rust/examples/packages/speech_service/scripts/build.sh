#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
PROTO_SRC="$PKG/proto"
PROTO_DEPS="$PKG/../../../crates/robonix-interfaces/robonix_proto"
OUT_DIR="$PKG/proto_gen"

FLAGS=()
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)

# Step 1: rbnx codegen (generates IDL-based protos: asr.proto, tts.proto, etc.)
rbnx codegen -p "$PKG" "${FLAGS[@]}" || true

# Step 2: Generate gRPC service stubs from speech_service.proto
mkdir -p "$OUT_DIR"
python -m grpc_tools.protoc \
    -I"$OUT_DIR" \
    -I"$PROTO_DEPS" \
    -I"$PROTO_SRC" \
    --python_out="$OUT_DIR" \
    --grpc_python_out="$OUT_DIR" \
    "$PROTO_SRC/speech_service.proto"

echo "[build] done."
