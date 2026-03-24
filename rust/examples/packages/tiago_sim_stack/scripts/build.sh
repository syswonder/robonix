#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG"
if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
  rm -rf rbnx-build
fi
docker compose -f compose.yaml build
mkdir -p rbnx-build/ws/install
echo "# stub (compose stack; no host colcon)" >rbnx-build/ws/install/setup.bash
