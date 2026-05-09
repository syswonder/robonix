#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# audio_driver runtime — runs on host (no sim container). Uses arecord /
# aplay against the host's ALSA stack to expose
# robonix/primitive/audio/{mic,speaker,driver} on atlas.
set -eo pipefail

PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

# robonix_api ships as a source dir; expose it on PYTHONPATH so
# `from robonix_api import Capability` resolves. The Capability
# constructor then walks up to add `rbnx-build/codegen/{proto_gen,
# robonix_mcp_types}` itself — packages don't manage codegen paths.
export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"

exec python3 -m audio_driver.main
