#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Same shape as audio_driver/scripts/build.sh — wraps `rbnx codegen` for
# the proto stubs. No native deps to compile; the bridge is pure Python
# (asyncio + websockets) plus the codegen output.
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
FLAGS=()
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)

rbnx codegen -p "$PKG" "${FLAGS[@]}"

# Keep package dependencies independent of whichever Python environment ran
# `rbnx build`. In particular, user-site installs are invalid when CI invokes
# the build from a virtualenv and do not guarantee the same interpreter at
# package start time.
VENV="$PKG/rbnx-build/venv"
uv venv --allow-existing --system-site-packages \
  --python "${AUDIO_CLIENT_BRIDGE_PYTHON:-python3}" "$VENV"
ROBONIX_API="$(rbnx path robonix-api)"
uv pip install --python "$VENV/bin/python" --quiet "$ROBONIX_API" 'websockets>=12,<16'

# Import the actual provider, not only its bridge-specific dependency.  This
# catches missing protobuf/grpc/robonix-api runtime dependencies during build
# instead of letting Soma discover them when the package starts.
PYTHONPATH="$ROBONIX_API:$PKG:${PYTHONPATH:-}" \
  "$VENV/bin/python" -c 'import audio_client_bridge.main'

echo "[build] done."
