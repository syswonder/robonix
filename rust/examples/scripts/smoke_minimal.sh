#!/usr/bin/env bash
# Starts robonix-server briefly and runs smoke_control_plane.py (no agent, no VLM).
set -euo pipefail

RUST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$RUST_ROOT"

if ! command -v python3 >/dev/null 2>&1; then
  echo "[smoke] python3 required"
  exit 1
fi

python3 -c "import grpc" 2>/dev/null || { echo "[smoke] pip install grpcio"; exit 1; }

export ROBONIX_SERVER="${ROBONIX_SERVER:-127.0.0.1:50051}"

SERVER_PID=""
if [[ "${SMOKE_USE_EXISTING_SERVER:-0}" != "1" ]]; then
  echo "[smoke] starting robonix-server..."
  cargo run -p robonix-server &
  SERVER_PID=$!
  sleep 2
fi

cleanup() {
  if [[ -n "$SERVER_PID" ]]; then
    kill "$SERVER_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

python3 examples/scripts/smoke_control_plane.py
EC=$?
echo "[smoke] exit=$EC"
exit "$EC"
