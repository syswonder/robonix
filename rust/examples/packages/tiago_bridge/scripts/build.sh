#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
RUST_ROOT="$(cd "$PKG/../../.." && pwd)"
extra=()
if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
  extra+=(--clean)
fi
exec cargo run --manifest-path "$RUST_ROOT/Cargo.toml" -p robonix-cli -- ros2-workspace-build -p "$PKG" "${extra[@]}"
