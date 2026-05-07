#!/usr/bin/env bash
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
# `--out-dir rbnx-build/codegen` so generated proto_gen/ lands where
# scripts/start.sh's PYTHONPATH points. Without this the codegen output
# goes to <pkg>/proto_gen/ (the rbnx-codegen default) while the runtime
# reads <pkg>/rbnx-build/codegen/proto_gen/ — new contracts then fail
# to register with a "no generated Servicer found" warning.
FLAGS=(--out-dir "$PKG/rbnx-build/codegen")
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)

# rbnx codegen (generates proto stubs from ROS IDL + contracts)
rbnx codegen -p "$PKG" "${FLAGS[@]}"

echo "[build] done."
