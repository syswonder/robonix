#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# audio_driver runtime — runs on host (no sim container). Uses arecord /
# aplay against the host's ALSA stack to expose
# robonix/primitive/audio/{mic,speaker,driver} on atlas.
set -eo pipefail

PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

# Codegen output + the runtime venv (`rbnx-build/`) and the package
# itself need to be on PYTHONPATH so `python3 -m audio_driver.node`
# finds the generated `audio_pb2` stubs and the package code.
export PYTHONPATH="$(rbnx path robonix-py):$PKG_ROOT/rbnx-build/codegen/proto_gen:$PKG_ROOT/rbnx-build/codegen/robonix_mcp_types:$PKG_ROOT:${PYTHONPATH:-}"

exec python3 -m audio_driver.node
