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
uv venv --system-site-packages --python "${AUDIO_CLIENT_BRIDGE_PYTHON:-python3}" "$VENV"
uv pip install --python "$VENV/bin/python" --quiet 'websockets>=12,<16'
"$VENV/bin/python" -c 'import websockets'

echo "[build] done."
