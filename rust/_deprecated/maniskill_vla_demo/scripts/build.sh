#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Codegen is done by `rbnx codegen`; adds demo-local maniskill_env proto
# on top of the shared runtime stubs.
# TODO: package-local contracts (packages/maniskill_vla_demo/contracts/) +
# custom IDL (interfaces/lib/) union is not yet supported by `rbnx codegen`;
# when it is, drop the `PKG/proto/maniskill_env.proto` step below.
# Run `rbnx setup` once from the robonix source root first.
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"

# Locate rbnx: prefer compiled debug binary under the robonix rust workspace,
# fallback to whatever is on PATH.
if ! command -v rbnx >/dev/null 2>&1; then
  for candidate in \
    "$HOME/robonix_upstream/rust/target/debug/rbnx" \
    "$(cd "$PKG/../../.." 2>/dev/null && pwd)/target/debug/rbnx" \
    ; do
    if [[ -x "$candidate" ]]; then
      export PATH="$(dirname "$candidate"):$PATH"
      break
    fi
  done
fi

FLAGS=(--mcp)
[[ "${RBNX_BUILD_CLEAN:-}" == "1" ]] && FLAGS+=(--clean)

rbnx codegen -p "$PKG" "${FLAGS[@]}"

# Demo-only proto (maniskill_env.proto — not a robonix contract, stays local).
# Prefer the package's uv venv python (has grpcio-tools); fall back to system.
if [[ -x "$PKG/.venv/bin/python3" ]]; then
  PY="$PKG/.venv/bin/python3"
else
  PY="python3"
fi
"$PY" -m grpc_tools.protoc \
  -I "$PKG/proto" \
  --python_out="$PKG/maniskill_vla_demo" \
  --grpc_python_out="$PKG/maniskill_vla_demo" \
  "$PKG/proto/maniskill_env.proto"

echo "[build] done."
