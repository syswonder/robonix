#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start the audio client bridge primitive on the Linux host.
# `client_audio_server/server.py` must already be running on the client
# machine (different repo / host; see this package's README).
set -eo pipefail

PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"

exec python3 -m audio_client_bridge.main
