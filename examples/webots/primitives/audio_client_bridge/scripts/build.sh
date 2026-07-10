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

# `websockets` is the one runtime dep this package needs that ships
# outside the robonix workspace. Install into the host Python; it's a
# small pure-Python wheel and avoids spinning up another venv just for
# one library.
if ! python3 -c "import websockets" >/dev/null 2>&1; then
    echo "[build] installing websockets (PyPI)"
    python3 -m pip install --user --quiet websockets >/dev/null
fi

echo "[build] done."
