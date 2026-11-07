#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
: "${NOVA5_RUNTIME_PYTHON:?Set NOVA5_RUNTIME_PYTHON to the Robonix Python environment}"
: "${NOVA5_REPO_ROOT:?Set NOVA5_REPO_ROOT to the Robonix repository}"
export ROBONIX_PROVIDER_BIND_HOST=127.0.0.1
export ROBONIX_ADVERTISE_HOST=127.0.0.1
export PYTHONPATH="$PKG:$PKG/rbnx-build/codegen/proto_gen:$PKG/rbnx-build/codegen/robonix_mcp_types:$NOVA5_REPO_ROOT/pylib/robonix-api"
exec "$NOVA5_RUNTIME_PYTHON" -m pick_return.driver
