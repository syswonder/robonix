#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# tiago_chassis build — codegen only. The driver runs inside the sim
# docker container (which already has rclpy + numpy + fastmcp + grpc),
# so no per-package venv. The codegen output (rbnx-build/codegen/) is
# bind-mounted into the container via sim/compose.yaml's ../primitives
# volume; the driver picks it up via _ensure_proto_gen / _ensure_mcp_types.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

CLEAN="${RBNX_BUILD_CLEAN:-}"
FLAGS=(--mcp)
[[ "$CLEAN" == "1" ]] && FLAGS+=(--clean)

echo "[tiago_chassis/build] rbnx codegen ${FLAGS[*]}"
rbnx codegen -p "$PKG" "${FLAGS[@]}"
echo "[tiago_chassis/build] done."
