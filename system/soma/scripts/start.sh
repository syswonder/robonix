#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# soma is small (urdf parser + 3 mcp tools + 1 ros pub + 1 robot_state_publisher
# subprocess); no docker. Just exec the venv'd python.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
mkdir -p rbnx-build/data
exec rbnx-build/venv/bin/python -m soma_service.service \
    > rbnx-build/data/soma.log 2>&1
