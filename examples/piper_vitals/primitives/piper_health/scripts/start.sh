#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0

set -euo pipefail

PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"
export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:$PKG_ROOT/rbnx-build/codegen/proto_gen:${PYTHONPATH:-}"

exec python3 -m piper_health.driver
