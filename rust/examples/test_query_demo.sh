#!/usr/bin/env bash
# Test query_demo: semantic_query server + client
# Prerequisite: robonix-server must be running (./start_server in another terminal)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUST_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$RUST_DIR"

echo "[test_query_demo] Ensure robonix-server is running (./start_server)"
echo "[test_query_demo] Building query_demo..."
cargo run --manifest-path robonix-cli/Cargo.toml -- build -p examples/query_demo 2>&1 | tail -5

echo "[test_query_demo] Starting semantic_server in background..."
cargo run --manifest-path robonix-cli/Cargo.toml -- start -p examples/query_demo -n semantic_server &
SERVER_PID=$!
trap "kill $SERVER_PID 2>/dev/null || true" EXIT

echo "[test_query_demo] Waiting 5s for server to register..."
sleep 5

echo "[test_query_demo] Running semantic_client..."
cargo run --manifest-path robonix-cli/Cargo.toml -- start -p examples/query_demo -n semantic_client

echo "[test_query_demo] OK"
