#!/usr/bin/env bash
# Test skill_demo: execute command server + client
# Prerequisite: robonix-server must be running (./start_server in another terminal)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUST_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$RUST_DIR"

echo "[test_skill_demo] Ensure robonix-server is running (./start_server)"
echo "[test_skill_demo] Building skill_demo..."
cargo run --manifest-path robonix-cli/Cargo.toml -- build -p examples/skill_demo 2>&1 | tail -5

echo "[test_skill_demo] Starting skill_server in background..."
cargo run --manifest-path robonix-cli/Cargo.toml -- start -p examples/skill_demo -n skill_server &
SERVER_PID=$!
trap "kill $SERVER_PID 2>/dev/null || true" EXIT

echo "[test_skill_demo] Waiting 5s for server to register..."
sleep 5

echo "[test_skill_demo] Running skill_client..."
cargo run --manifest-path robonix-cli/Cargo.toml -- start -p examples/skill_demo -n skill_client

echo "[test_skill_demo] OK"
