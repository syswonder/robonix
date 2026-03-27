#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
  rm -rf "$PKG/rbnx-build"
fi
EXAMPLES="$(cd "$PKG/../.." && pwd)"
PROTO_GEN="$EXAMPLES/proto_gen"
mkdir -p "$PKG/rbnx-build/ws/install"
cat >"$PKG/rbnx-build/ws/install/setup.bash" <<EOF
export PYTHONPATH="$PKG:$PROTO_GEN:\${PYTHONPATH:-}"
EOF
