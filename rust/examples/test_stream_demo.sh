#!/usr/bin/env bash
# Test stream_demo: pose publisher + subscriber
# Prerequisite: robonix-server must be running (./start_server in another terminal)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUST_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$RUST_DIR"

echo "[test_stream_demo] Ensure robonix-server is running (./start_server)"
echo "[test_stream_demo] Building stream_demo..."
cargo run --manifest-path robonix-cli/Cargo.toml -- build -p examples/stream_demo 2>&1 | tail -5

echo "[test_stream_demo] Starting stream_server in background..."
cargo run --manifest-path robonix-cli/Cargo.toml -- start -p examples/stream_demo -n stream_server &
SERVER_PID=$!
trap "kill $SERVER_PID 2>/dev/null || true" EXIT

echo "[test_stream_demo] Waiting 5s for server to register..."
sleep 5

echo "[test_stream_demo] Running stream_client..."
cargo run --manifest-path robonix-cli/Cargo.toml -- start -p examples/stream_demo -n stream_client

echo "[test_stream_demo] OK"
