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

echo "[build] generating proto_gen stubs (control-plane gRPC)..."

# 1. Local package proto (maniskill_env) → maniskill_vla_demo/ (importable as package module)
python3 -m grpc_tools.protoc \
  -I "$PKG/proto" \
  --python_out="$PKG/maniskill_vla_demo" \
  --grpc_python_out="$PKG/maniskill_vla_demo" \
  "$PKG/proto/maniskill_env.proto"

# 2. Shared robonix runtime + interface protos → package-local proto_gen/.
INTERFACES_DIR="$RUST_ROOT/crates/robonix-interfaces/robonix_proto"
RUNTIME_DIR="$RUST_ROOT/proto"
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
cargo run -p robonix-codegen --manifest-path "$RUST_ROOT/Cargo.toml" -- \
  --lang mcp \
  -I "$RUST_ROOT/crates/robonix-interfaces/lib" \
  -o "$MCP_TYPES"

mkdir -p "$PKG/rbnx-build/ws/install"
cat >"$PKG/rbnx-build/ws/install/setup.bash" <<EOF
#!/usr/bin/env bash
export PYTHONPATH="$PKG:$PROTO_GEN:$MCP_TYPES:$ROBONIX_MCP_CONTRACT_PKG:\${PYTHONPATH:-}"
EOF

echo "[build] done."
