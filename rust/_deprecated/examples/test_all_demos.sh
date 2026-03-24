#!/usr/bin/env bash
# Run all demo tests: query_demo, skill_demo, stream_demo
# Prerequisite: robonix-server must be running (./start_server in another terminal)
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "=== Testing query_demo (semantic_query) ==="
./test_query_demo.sh
echo ""

echo "=== Testing skill_demo (execute command) ==="
./test_skill_demo.sh
echo ""

echo "=== Testing stream_demo (pose stream) ==="
./test_stream_demo.sh
echo ""

echo "=== All demos OK ==="
