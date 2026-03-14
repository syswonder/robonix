#!/usr/bin/env bash
# Usage: ./test_demo.sh <demo> <server_node> <client_node>
# Example: ./test_demo.sh query_demo semantic_server semantic_client
# Prerequisite: robonix-server must be running (./start_server)
set -euo pipefail

cd "$(dirname "$0")/.."
RBNX="cargo run --manifest-path robonix-cli/Cargo.toml --"

echo "[$1] Ensure robonix-server is running (./start_server)"
$RBNX build -p examples/$1
$RBNX start -p examples/$1 -n $2 &
trap "kill $! 2>/dev/null || true" EXIT
sleep 5
$RBNX start -p examples/$1 -n $3
echo "[$1] OK"
