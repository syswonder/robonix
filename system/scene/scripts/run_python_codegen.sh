#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Generate Scene's Python protobuf stubs in a reproducible environment.
# Generated `_pb2.py` modules encode the protobuf generator version, while
# `_pb2_grpc.py` modules enforce a minimum grpcio runtime. Using whichever
# grpcio-tools happens to be installed on the host can therefore produce
# stubs that the Scene image or Jetson host cannot import.
set -euo pipefail

if [[ "$#" -lt 1 ]]; then
    echo "usage: $0 <package-root> [rbnx-codegen-args...]" >&2
    exit 2
fi

PKG="$(cd "$1" && pwd)"
shift
VENV="$PKG/rbnx-build/python-codegen-venv"
PROTOBUF_VERSION="6.33.6"
GRPC_TOOLS_VERSION="1.76.0"
GRPCIO_VERSION="1.80.0"

if ! command -v uv >/dev/null 2>&1; then
    echo "[scene/codegen] error: 'uv' not found on PATH. Install: https://docs.astral.sh/uv/" >&2
    exit 1
fi

compatible=0
if [[ -x "$VENV/bin/python" ]]; then
    if "$VENV/bin/python" - "$PROTOBUF_VERSION" "$GRPC_TOOLS_VERSION" "$GRPCIO_VERSION" <<'PY'
from importlib.metadata import version
import sys

expected_protobuf, expected_tools, expected_grpc = sys.argv[1:]
if version("protobuf") != expected_protobuf:
    raise SystemExit(1)
if version("grpcio-tools") != expected_tools:
    raise SystemExit(1)
if version("grpcio") != expected_grpc:
    raise SystemExit(1)

import grpc_tools.protoc  # noqa: F401, E402
PY
    then
        compatible=1
    fi
fi

if [[ "$compatible" != "1" ]]; then
    echo "[scene/codegen] preparing protobuf $PROTOBUF_VERSION / grpcio-tools $GRPC_TOOLS_VERSION / grpcio $GRPCIO_VERSION"
    rm -rf "$VENV"
    uv venv "$VENV"
    uv pip install --python "$VENV/bin/python" \
        "protobuf==$PROTOBUF_VERSION" \
        "grpcio-tools==$GRPC_TOOLS_VERSION" \
        "grpcio==$GRPCIO_VERSION"
fi

echo "[scene/codegen] rbnx codegen -p $PKG $*"
PATH="$VENV/bin:$PATH" rbnx codegen -p "$PKG" "$@"

# Import the generated modules with the exact generator/runtime versions. Do
# not inherit the caller's PYTHONPATH: generic module names such as map_pb2 can
# otherwise resolve from another package and hide missing Scene output. The
# verifier also checks every module's __file__ against these two package roots.
PROTO_ROOT="$PKG/rbnx-build/codegen/proto_gen"
MCP_ROOT="$PKG/rbnx-build/codegen/robonix_mcp_types"
PYTHONPATH="$PROTO_ROOT:$MCP_ROOT" \
    "$VENV/bin/python" "$PKG/scripts/verify_python_codegen.py" \
    "$PROTO_ROOT" "$MCP_ROOT" \
    "protobuf=$PROTOBUF_VERSION" \
    "grpcio-tools=$GRPC_TOOLS_VERSION" \
    "grpcio=$GRPCIO_VERSION"
