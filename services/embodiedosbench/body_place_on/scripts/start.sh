#!/usr/bin/env bash
# Start the provider; EMBODIEDOSBENCH_ROOT and BENCH_EMBODIMENT_ADDR come from the deployment env.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
: "${ROBONIX_ROOT:?set ROBONIX_ROOT}"
: "${EMBODIEDOSBENCH_ROOT:?set EMBODIEDOSBENCH_ROOT to the bench/ directory}"
export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"
export PYTHONPATH="$PKG:$PKG/../common:$ROBONIX_ROOT/pylib/robonix-api:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"
exec "$PKG/rbnx-build/venv/bin/python" -m body_place_on.service
