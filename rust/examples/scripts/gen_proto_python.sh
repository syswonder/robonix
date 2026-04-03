#!/usr/bin/env bash
# Regenerate rust/examples/proto_gen from rust/proto and crates/robonix-interfaces/robonix_proto.
# Requires: pip install grpcio-tools  (provides python3 -m grpc_tools.protoc)
set -euo pipefail

RUST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
OUT="${RUST_ROOT}/examples/proto_gen"
PROTO_RUNTIME="${RUST_ROOT}/proto"
PROTO_RI="${RUST_ROOT}/crates/robonix-interfaces/robonix_proto"

if ! python3 -m grpc_tools.protoc --version >/dev/null 2>&1; then
  echo "[gen_proto_python] grpc_tools not found. Install: pip install grpcio-tools"
  exit 1
fi

mkdir -p "$OUT"
mapfile -t PROTO_FILES < <(find "$PROTO_RI" -name '*.proto' | LC_ALL=C sort)

echo "[gen_proto_python] output: $OUT"
python3 -m grpc_tools.protoc \
  -I "$PROTO_RUNTIME" \
  -I "$PROTO_RI" \
  --python_out="$OUT" \
  --grpc_python_out="$OUT" \
  "$PROTO_RUNTIME/robonix_runtime.proto" \
  "${PROTO_FILES[@]}"

echo "[gen_proto_python] done."
