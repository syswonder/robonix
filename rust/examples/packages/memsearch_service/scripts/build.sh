#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
EXAMPLES="$(cd "$PKG/../.." && pwd)"
ROBONIX_MCP_CONTRACT_PKG="$(cd "$EXAMPLES/packages/robonix_mcp_contract" && pwd)"
RUST_ROOT="$(cd "$EXAMPLES/.." && pwd)"
PROTO_GEN="$PKG/proto_gen"
MCP_TYPES="$PKG/robonix_mcp_types"

if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
  rm -rf "$PKG/rbnx-build"
  rm -rf "$MCP_TYPES"
  rm -rf "$PROTO_GEN"
fi

INTERFACES_LIB="$RUST_ROOT/crates/robonix-interfaces/lib"
CONTRACTS_DIR="$RUST_ROOT/contracts"
INTERFACES_DIR="$RUST_ROOT/crates/robonix-interfaces/robonix_proto"
if [[ -x /usr/bin/cargo ]]; then
  CARGO_BIN=/usr/bin/cargo
else
  CARGO_BIN="${CARGO:-cargo}"
fi

echo "[build] regenerating crates/robonix-interfaces/robonix_proto (robonix-codegen --lang proto)..."
"$CARGO_BIN" run -p robonix-codegen --manifest-path "$RUST_ROOT/Cargo.toml" -- \
  --lang proto \
  -I "$INTERFACES_LIB" \
  --contracts "$CONTRACTS_DIR" \
  -o "$INTERFACES_DIR"

# IDL data types → Python dataclasses for MCP tool schemas (robonix-codegen --lang mcp).
echo "[build] generating robonix_mcp_types (robonix-codegen --lang mcp)..."
mkdir -p "$MCP_TYPES"
"$CARGO_BIN" run -p robonix-codegen --manifest-path "$RUST_ROOT/Cargo.toml" -- \
  --lang mcp \
  -I "$INTERFACES_LIB" \
  -o "$MCP_TYPES"

# Package-local *_pb2*.py (includes robonix_contracts_pb2_grpc for tooling); gRPC services only in robonix_contracts.proto.
mkdir -p "$PROTO_GEN"
if python3 -m grpc_tools.protoc --version >/dev/null 2>&1; then
  echo "[build] generating proto_gen stubs (grpc_tools.protoc)..."
  RUNTIME_DIR="$RUST_ROOT/proto"
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

mkdir -p "$PKG/rbnx-build/ws/install"
cat >"$PKG/rbnx-build/ws/install/setup.bash" <<EOF
export PYTHONPATH="$PKG:$PROTO_GEN:$MCP_TYPES:$ROBONIX_MCP_CONTRACT_PKG:\${PYTHONPATH:-}"
EOF
