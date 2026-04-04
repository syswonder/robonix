#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
EXAMPLES="$(cd "$PKG/../.." && pwd)"
RUST_ROOT="$(cd "$EXAMPLES/.." && pwd)"
ROBONIX_MCP_CONTRACT_PKG="$(cd "$EXAMPLES/packages/robonix_mcp_contract" && pwd)"
PROTO_GEN="$PKG/proto_gen"
MCP_TYPES="$PKG/robonix_mcp_types"

if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
  rm -rf "$PKG/rbnx-build"
  rm -rf "$PROTO_GEN"
  rm -rf "$MCP_TYPES"
fi

if ! python3 -m grpc_tools.protoc --version >/dev/null 2>&1; then
  echo "[build] grpc_tools not found — run: ./run.sh setup"
  exit 1
fi

INTERFACES_LIB="$RUST_ROOT/crates/robonix-interfaces/lib"
CONTRACTS_DIR="$RUST_ROOT/contracts"
INTERFACES_DIR="$RUST_ROOT/crates/robonix-interfaces/robonix_proto"
RUNTIME_DIR="$RUST_ROOT/proto"

# 0. Regenerate `robonix_proto/*.proto` from `lib/**` + `rust/contracts` (messages + contract srv shims; gRPC services only in `robonix_contracts.proto`).
echo "[build] regenerating crates/robonix-interfaces/robonix_proto (robonix-codegen --lang proto)..."
if [[ -x /usr/bin/cargo ]]; then
  CARGO_BIN=/usr/bin/cargo
else
  CARGO_BIN="${CARGO:-cargo}"
fi
"$CARGO_BIN" run -p robonix-codegen --manifest-path "$RUST_ROOT/Cargo.toml" -- \
  --lang proto \
  -I "$INTERFACES_LIB" \
  --contracts "$CONTRACTS_DIR" \
  -o "$INTERFACES_DIR"

echo "[build] generating proto_gen stubs (runtime + robonix interfaces; gRPC services only in robonix_contracts.proto)..."

# 1. Demo-local env/slam gRPC (maniskill_env.proto — not merged into robonix_contracts.proto)
python3 -m grpc_tools.protoc \
  -I "$PKG/proto" \
  --python_out="$PKG/maniskill_vla_demo" \
  --grpc_python_out="$PKG/maniskill_vla_demo" \
  "$PKG/proto/maniskill_env.proto"

# 2. Shared robonix runtime + interface protos → package-local proto_gen/.
mkdir -p "$PROTO_GEN"
python3 -m grpc_tools.protoc \
  -I "$RUNTIME_DIR" \
  -I "$INTERFACES_DIR" \
  --python_out="$PROTO_GEN" \
  --grpc_python_out="$PROTO_GEN" \
  "$RUNTIME_DIR"/*.proto \
  "$INTERFACES_DIR"/*.proto 2>/dev/null || true

# 3. IDL data types → Python dataclasses for MCP tool schemas (robonix-codegen --lang mcp).
#    Import these in MCP tool handlers instead of defining schemas by hand.
echo "[build] generating robonix_mcp_types (robonix-codegen --lang mcp)..."
mkdir -p "$MCP_TYPES"
"$CARGO_BIN" run -p robonix-codegen --manifest-path "$RUST_ROOT/Cargo.toml" -- \
  --lang mcp \
  -I "$INTERFACES_LIB" \
  -o "$MCP_TYPES"

mkdir -p "$PKG/rbnx-build/ws/install"
cat >"$PKG/rbnx-build/ws/install/setup.bash" <<EOF
#!/usr/bin/env bash
export PYTHONPATH="$PKG:$PROTO_GEN:$MCP_TYPES:$ROBONIX_MCP_CONTRACT_PKG:\${PYTHONPATH:-}"
EOF

echo "[build] done."
