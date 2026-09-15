#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -eo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
set -u
export PYTHONPATH="$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
exec "$PKG/rbnx-build/venv/bin/python" -m vlm_verifier.service
