#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail

SCENE_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
PYTHON="${PYTHON:-python3}"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

PROTO_ROOT="$TMP/proto_gen"
MCP_ROOT="$TMP/robonix_mcp_types"
STALE_ROOT="$TMP/stale"
mkdir -p "$PROTO_ROOT" "$MCP_ROOT" "$STALE_ROOT"

# Minimal generated-module fixture. Empty modules are sufficient to exercise
# import resolution and the verifier's package-root provenance checks.
touch \
    "$PROTO_ROOT/map_pb2.py" \
    "$PROTO_ROOT/robonix_contracts_pb2.py" \
    "$PROTO_ROOT/robonix_contracts_pb2_grpc.py" \
    "$MCP_ROOT/semantic_map_mcp.py"

PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH="$PROTO_ROOT:$MCP_ROOT" \
    "$PYTHON" "$SCENE_ROOT/scripts/verify_python_codegen.py" \
    "$PROTO_ROOT" "$MCP_ROOT" >/dev/null

# Removing the package-local MCP module must fail even if an inherited path
# offers a stale module with the same generic name.
rm "$MCP_ROOT/semantic_map_mcp.py"
touch "$STALE_ROOT/semantic_map_mcp.py"
ERROR_LOG="$TMP/stale-import.log"
if PYTHONDONTWRITEBYTECODE=1 \
   PYTHONPATH="$PROTO_ROOT:$MCP_ROOT:$STALE_ROOT" \
       "$PYTHON" "$SCENE_ROOT/scripts/verify_python_codegen.py" \
       "$PROTO_ROOT" "$MCP_ROOT" >"$ERROR_LOG" 2>&1; then
    echo "test_python_codegen: stale generated module unexpectedly passed" >&2
    exit 1
fi
grep -q "resolved outside expected root" "$ERROR_LOG"

echo "test_python_codegen: PASS"
