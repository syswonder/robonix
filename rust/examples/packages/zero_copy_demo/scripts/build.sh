#!/usr/bin/env bash
set -euo pipefail

# Build script for the zero_copy_demo package.
# Called by `rbnx build -p zero_copy_demo`.

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
# Package is at rust/examples/packages/zero_copy_demo → workspace root is rust/ (three levels up).
RUST_ROOT="$(cd "$PKG_ROOT/../../.." && pwd)"
INTERFACES_LIB="$RUST_ROOT/crates/robonix-interfaces/lib"
CONTRACTS_DIR="$RUST_ROOT/contracts"
RUNTIME_DIR="$RUST_ROOT/proto"
INTERFACES_DIR="$RUST_ROOT/crates/robonix-interfaces/robonix_proto"
PROTO_GEN="$PKG_ROOT/proto_gen"

if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
    rm -rf "$PROTO_GEN"
    rm -rf "$PKG_ROOT/rbnx-build"
fi

echo "[build] Building librobonix_buffer.so..."
(cd "$RUST_ROOT" && cargo build -p robonix-buffer --release)

echo "[build] Installing Python package (editable)..."
if command -v uv &>/dev/null; then
    (cd "$PKG_ROOT" && uv pip install -e .)
else
    (cd "$PKG_ROOT" && pip install -e .)
fi

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

# Stubs include robonix_contracts_pb2*.py; gRPC service definitions only in robonix_contracts.proto.
echo "[build] generating package-local proto_gen stubs..."
mkdir -p "$PROTO_GEN"
if python3 -m grpc_tools.protoc --version >/dev/null 2>&1; then
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

mkdir -p "$PKG_ROOT/rbnx-build/ws/install"
cat >"$PKG_ROOT/rbnx-build/ws/install/setup.bash" <<EOF
#!/usr/bin/env bash
export PYTHONPATH="$PKG_ROOT:$PROTO_GEN:\${PYTHONPATH:-}"
EOF

touch "$PKG_ROOT/rbnx-build/.rbnx-built"
echo "[build] Done."
