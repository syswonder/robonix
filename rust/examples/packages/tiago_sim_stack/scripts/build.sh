#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
EXAMPLES="$(cd "$PKG/../.." && pwd)"
RUST_ROOT="$(cd "$EXAMPLES/.." && pwd)"
ROBONIX_MCP_CONTRACT_PKG="$(cd "$EXAMPLES/packages/robonix_mcp_contract" && pwd)"
MCP_TYPES="$PKG/tiago_bridge/robonix_mcp_types"
PROTO_GEN="$PKG/tiago_bridge/proto_gen"

cd "$PKG"
if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
  rm -rf rbnx-build
  rm -rf "$MCP_TYPES"
  rm -rf "$PROTO_GEN"
fi

# IDL data types → Python dataclasses for MCP tool schemas (robonix-codegen --lang mcp).
# Imported by tiago_bridge/node.py to build/validate contract-typed inputs and outputs.
echo "[build] generating robonix_mcp_types (robonix-codegen --lang mcp)..."
mkdir -p "$MCP_TYPES"
# Prefer system cargo if rustup is broken (e.g. misconfigured toolchain proxy).
if [[ -x /usr/bin/cargo ]]; then
  CARGO_BIN=/usr/bin/cargo
else
  CARGO_BIN="${CARGO:-cargo}"
fi
"$CARGO_BIN" run -p robonix-codegen --manifest-path "$RUST_ROOT/Cargo.toml" -- \
  --lang mcp \
  -I "$RUST_ROOT/crates/robonix-interfaces/lib" \
  -o "$MCP_TYPES"

# Runtime proto stubs are copied into the Docker image at /app/proto_gen.
# Keep this directory present (and fresh when grpc_tools is available) before compose build.
mkdir -p "$PROTO_GEN"
if python3 -m grpc_tools.protoc --version >/dev/null 2>&1; then
  echo "[build] generating proto_gen stubs (grpc_tools.protoc)..."
  RUNTIME_DIR="$RUST_ROOT/proto"
  INTERFACES_DIR="$RUST_ROOT/crates/robonix-interfaces/robonix_proto"
  python3 -m grpc_tools.protoc \
    -I "$RUNTIME_DIR" \
    -I "$INTERFACES_DIR" \
    --python_out="$PROTO_GEN" \
    --grpc_python_out="$PROTO_GEN" \
    "$RUNTIME_DIR"/*.proto \
    "$INTERFACES_DIR"/*.proto 2>/dev/null || true
else
  echo "[build] grpcio-tools not found; leaving existing proto_gen/ as-is"
fi

mkdir -p rbnx-build/ws/install
cat >"rbnx-build/ws/install/setup.bash" <<EOF
#!/usr/bin/env bash
export PYTHONPATH="$PKG/tiago_bridge:$MCP_TYPES:$PROTO_GEN:$ROBONIX_MCP_CONTRACT_PKG:\${PYTHONPATH:-}"
EOF

docker compose -f compose.yaml build
echo "[build] done."
