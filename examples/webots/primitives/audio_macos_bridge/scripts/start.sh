#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start the macOS-bridge audio primitive on the Linux host.
# `mac_server/server.py` must already be running on the target macOS
# box (different repo / box; see this package's README).
set -eo pipefail

PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

export PYTHONPATH="$(rbnx path robonix-py):$PKG_ROOT/rbnx-build/codegen/proto_gen:$PKG_ROOT/rbnx-build/codegen/robonix_mcp_types:$PKG_ROOT:${PYTHONPATH:-}"

exec python3 -m audio_macos_bridge.node
