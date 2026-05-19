#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
#
# Build robonix-api sdist + wheel for PyPI / Test PyPI.
#
# Steps:
#   1. Run protoc against rust/crates/robonix-atlas/proto/atlas.proto
#      and write atlas_pb2.py / atlas_pb2_grpc.py into robonix_api/_generated/.
#   2. `python -m build` to produce dist/*.tar.gz + dist/*.whl.
#
# Does NOT upload. Run scripts/publish_testpypi.sh / scripts/publish_pypi.sh
# afterwards to publish.

set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_ROOT="$(cd "$HERE/.." && pwd)"
REPO_ROOT="$(cd "$PKG_ROOT/../.." && pwd)"
PROTO_DIR="$REPO_ROOT/rust/crates/robonix-atlas/proto"
PROTO_FILE="$PROTO_DIR/atlas.proto"
OUT_DIR="$PKG_ROOT/robonix_api/_generated"

if [[ ! -f "$PROTO_FILE" ]]; then
    echo "ERR: atlas.proto not found at $PROTO_FILE" >&2
    exit 1
fi

PYTHON="${PYTHON:-python3}"

if ! "$PYTHON" -c "import grpc_tools.protoc" 2>/dev/null; then
    echo "ERR: grpc_tools not installed in $PYTHON." >&2
    echo "     Install with: $PYTHON -m pip install grpcio-tools" >&2
    exit 1
fi
if ! "$PYTHON" -c "import build" 2>/dev/null; then
    echo "ERR: build not installed in $PYTHON." >&2
    echo "     Install with: $PYTHON -m pip install build" >&2
    exit 1
fi

echo "[build_dist] cleaning $OUT_DIR and dist/"
rm -rf "$OUT_DIR" "$PKG_ROOT/dist" "$PKG_ROOT/build" "$PKG_ROOT"/*.egg-info
mkdir -p "$OUT_DIR"

echo "[build_dist] generating atlas_pb2*.py from $PROTO_FILE"
"$PYTHON" -m grpc_tools.protoc \
    -I "$PROTO_DIR" \
    --python_out="$OUT_DIR" \
    --grpc_python_out="$OUT_DIR" \
    "$PROTO_FILE"

# grpc_tools emits atlas_pb2_grpc.py with `import atlas_pb2 as ...` at top
# level. That works because robonix_api/__init__.py appends _generated/ to
# sys.path on import. Mark _generated as a package so wheel data inclusion
# is clean.
touch "$OUT_DIR/__init__.py"
echo "[build_dist] generated files:"
ls -1 "$OUT_DIR"

echo "[build_dist] python -m build"
cd "$PKG_ROOT"
"$PYTHON" -m build

echo "[build_dist] done. Artifacts:"
ls -la "$PKG_ROOT/dist"
