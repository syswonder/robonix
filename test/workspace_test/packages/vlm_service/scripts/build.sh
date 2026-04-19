#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
EXAMPLES="$(cd "$PKG/../.." && pwd)"
RUST_ROOT_PRE="$(cd "$EXAMPLES/.." && pwd)"
RUST_ROOT="${RUST_ROOT:-$(cd "$RUST_ROOT_PRE/../rust" && pwd)}"
PROTO_GEN="$PKG/proto_gen"

if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
  rm -rf "$PKG/rbnx-build"
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

# Package-local proto_gen (robonix_contracts_pb2_grpc.SysModelVlmChat, etc.).
if python3 -m grpc_tools.protoc --version >/dev/null 2>&1; then
  echo "[build] generating proto_gen stubs..."
  mkdir -p "$PROTO_GEN"
  RUNTIME_DIR="$RUST_ROOT/proto"
  python3 -m grpc_tools.protoc \
    -I "$RUNTIME_DIR" \
    -I "$INTERFACES_DIR" \
    --python_out="$PROTO_GEN" \
    --grpc_python_out="$PROTO_GEN" \
    "$RUNTIME_DIR"/*.proto \
    "$INTERFACES_DIR"/*.proto 2>/dev/null || true
else
  echo "[build] grpcio-tools not found — leaving existing package-local proto_gen/"
fi

mkdir -p "$PKG/rbnx-build/ws/install"
cat >"$PKG/rbnx-build/ws/install/setup.bash" <<EOF
#!/usr/bin/env bash
export PYTHONPATH="$PKG:$PROTO_GEN:\${PYTHONPATH:-}"
EOF

echo "[build] done."
