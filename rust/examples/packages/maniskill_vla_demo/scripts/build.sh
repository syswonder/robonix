#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail
PKG="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
EXAMPLES="$(cd "$PKG/../.." && pwd)"
RUST_ROOT="$(cd "$EXAMPLES/.." && pwd)"
PROTO_GEN="$EXAMPLES/proto_gen"

if [[ "${RBNX_BUILD_CLEAN:-}" == "1" ]]; then
  rm -rf "$PKG/rbnx-build"
  rm -rf "$PKG/proto_stubs"
fi

if ! python3 -m grpc_tools.protoc --version >/dev/null 2>&1; then
  echo "[build] grpc_tools not found — run: ./run.sh setup"
  exit 1
fi

echo "[build] generating proto stubs..."

# 1. Local package proto (maniskill_env) → maniskill_vla_demo/ (importable as package module)
python3 -m grpc_tools.protoc \
  -I "$PKG/proto" \
  --python_out="$PKG/maniskill_vla_demo" \
  --grpc_python_out="$PKG/maniskill_vla_demo" \
  "$PKG/proto/maniskill_env.proto"

# 2. Shared robonix runtime + interface protos → proto_stubs/ (version-matched to active venv).
#    proto_gen/ used to hold these as committed files; now each package generates its own copy.
PROTO_STUBS="$PKG/proto_stubs"
mkdir -p "$PROTO_STUBS"
RUNTIME_PROTO="$RUST_ROOT/proto/robonix_runtime.proto"
INTERFACES_DIR="$RUST_ROOT/robonix-interfaces/robonix_proto"
RUNTIME_DIR="$RUST_ROOT/proto"

python3 -m grpc_tools.protoc \
  -I "$RUNTIME_DIR" \
  -I "$INTERFACES_DIR" \
  --python_out="$PROTO_STUBS" \
  --grpc_python_out="$PROTO_STUBS" \
  "$RUNTIME_PROTO" \
  "$INTERFACES_DIR/vlm.proto" \
  "$INTERFACES_DIR/builtin_interfaces.proto" \
  "$INTERFACES_DIR/std_msgs.proto" \
  "$INTERFACES_DIR/geometry_msgs.proto" \
  "$INTERFACES_DIR/robonix_msg.proto"

# 3. Also populate proto_gen/ so other packages that haven't been updated yet still work.
mkdir -p "$PROTO_GEN"
python3 -m grpc_tools.protoc \
  -I "$RUNTIME_DIR" \
  -I "$INTERFACES_DIR" \
  --python_out="$PROTO_GEN" \
  --grpc_python_out="$PROTO_GEN" \
  "$RUNTIME_DIR"/*.proto \
  "$INTERFACES_DIR"/*.proto 2>/dev/null || true

mkdir -p "$PKG/rbnx-build/ws/install"
cat >"$PKG/rbnx-build/ws/install/setup.bash" <<EOF
#!/usr/bin/env bash
# proto_stubs (version-matched to this package's venv) takes priority over proto_gen
export PYTHONPATH="$PKG:$PROTO_STUBS:$PROTO_GEN:\${PYTHONPATH:-}"
EOF

echo "[build] done."
