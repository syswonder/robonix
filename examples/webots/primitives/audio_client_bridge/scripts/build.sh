#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Same shape as audio_driver/scripts/build.sh — wraps `rbnx codegen` for
# the proto stubs. No native deps to compile; the bridge is pure Python
# (asyncio + websockets) plus the codegen output.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
BUILD="$PKG/rbnx-build"
VENV="$BUILD/venv"

# Keep package dependencies independent of whichever Python environment ran
# `rbnx build`. In particular, user-site installs are invalid when CI invokes
# the build from a virtualenv and do not guarantee the same interpreter at
# package start time.
if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
  echo "[build] clean: removing $BUILD"
  rm -rf "$BUILD"
fi
if ! command -v uv >/dev/null 2>&1; then
  echo "[build] error: 'uv' not found on PATH. Install: https://docs.astral.sh/uv/" >&2
  exit 1
fi
uv venv --allow-existing \
  --python "${AUDIO_CLIENT_BRIDGE_PYTHON:-python3}" "$VENV"
ROBONIX_API="$(rbnx path robonix-api)"
uv pip install --python "$VENV/bin/python" --quiet \
  "$ROBONIX_API" 'websockets>=12,<16' \
  "grpcio==1.80.0" "grpcio-tools==1.76.0" "protobuf==6.33.6"

RBNX_CODEGEN_PYTHON="$VENV/bin/python" \
  PATH="$VENV/bin:$PATH" \
  rbnx codegen -p "$PKG"

# Import the actual provider, not only its bridge-specific dependency.  This
# catches missing protobuf/grpc/robonix-api runtime dependencies during build
# instead of letting Soma discover them when the package starts.
CODEGEN_ROOT="$BUILD/codegen"
PYTHONPATH="$ROBONIX_API:$PKG:$CODEGEN_ROOT/proto_gen:$CODEGEN_ROOT/robonix_mcp_types:${PYTHONPATH:-}" \
  "$VENV/bin/python" -c 'import audio_pb2; import audio_client_bridge.main'

echo "[build] done."
