#!/usr/bin/env bash
set -euo pipefail

PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
ROBONIX_ROOT="$(cd "$PKG/../../../.." && pwd)"
export ROBONIX_ATLAS="127.0.0.1:50051"
export PYTHONPATH="$PKG:$ROBONIX_ROOT/pylib/robonix-api:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
exec python3 -m remote_demo_skill.service
