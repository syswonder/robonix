#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start the audio client bridge primitive on the Linux host.
# `client_audio_server/server.py` must already be running on the client
# machine (different repo / host; see this package's README).
set -eo pipefail

PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"

PYTHON="$PKG_ROOT/rbnx-build/venv/bin/python"
if [[ ! -x "$PYTHON" ]]; then
  echo "audio_client_bridge is not built; run 'rbnx build' first" >&2
  exit 1
fi

exec "$PYTHON" -m audio_client_bridge.main
