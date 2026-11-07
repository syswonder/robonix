#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
: "${NOVA5_RUNTIME_PYTHON:?Set NOVA5_RUNTIME_PYTHON to the Robonix Python environment}"
RBNX_CODEGEN_PYTHON="$NOVA5_RUNTIME_PYTHON" rbnx codegen -p "$PKG" --mcp
