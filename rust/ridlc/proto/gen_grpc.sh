#!/usr/bin/env bash
# Generate Python gRPC stubs from proto/robonix_runtime.proto.
# Requires: pip install grpcio-tools
# Usage: from repo root, ./proto/gen_grpc.sh [OUT_DIR]
# Default OUT_DIR is proto/gen (so imports become from robonix_runtime_pb2 import ... from that dir)
#
# The generated pb2 from newer protobuf releases may import
# `google.protobuf.runtime_version`, which is unavailable in the older system
# protobuf shipped with ROS 2 Humble. Strip that guard so the checked-in stubs
# stay runnable in the ROS test environment.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OUT_DIR="${1:-${SCRIPT_DIR}/gen}"
mkdir -p "$OUT_DIR"
python3 -m grpc_tools.protoc \
  -I"$SCRIPT_DIR" \
  --python_out="$OUT_DIR" \
  --grpc_python_out="$OUT_DIR" \
  "$SCRIPT_DIR/robonix_runtime.proto"
python3 - "$OUT_DIR/robonix_runtime_pb2.py" <<'PY'
from pathlib import Path
import re
import sys

path = Path(sys.argv[1])
src = path.read_text()
src = src.replace("from google.protobuf import runtime_version as _runtime_version\n", "")
src = re.sub(
    r"_runtime_version\.ValidateProtobufRuntimeVersion\([\s\S]*?\)\n",
    "",
    src,
    count=1,
)
path.write_text(src)
PY
echo "Generated in $OUT_DIR:"
ls -la "$OUT_DIR"
# Fix imports if proto package is robonix.runtime (generated files may use absolute_import from package)
echo "Use: PYTHONPATH=\"$OUT_DIR\" or copy gen/* into your project and \"from robonix_runtime_pb2 import ...\""
