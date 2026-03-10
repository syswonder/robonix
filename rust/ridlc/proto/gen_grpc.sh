#!/usr/bin/env bash
# Generate Python gRPC stubs from proto/robonix_runtime.proto.
# Requires: pip install grpcio-tools
# Usage: from repo root, ./proto/gen_grpc.sh [OUT_DIR]
# Default OUT_DIR is proto/gen (so imports become from robonix_runtime_pb2 import ... from that dir)
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${1:-${SCRIPT_DIR}/gen}"
mkdir -p "$OUT_DIR"
python3 -m grpc_tools.protoc \
  -I"$SCRIPT_DIR" \
  --python_out="$OUT_DIR" \
  --grpc_python_out="$OUT_DIR" \
  "$SCRIPT_DIR/robonix_runtime.proto"
echo "Generated in $OUT_DIR:"
ls -la "$OUT_DIR"
# Fix imports if proto package is robonix.runtime (generated files may use absolute_import from package)
echo "Use: PYTHONPATH=\"$OUT_DIR\" or copy gen/* into your project and \"from robonix_runtime_pb2 import ...\""
