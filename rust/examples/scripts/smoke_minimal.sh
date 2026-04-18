#!/usr/bin/env bash
# Starts robonix-atlas briefly and runs smoke_control_plane.py (no pilot, no VLM).
set -euo pipefail

RUST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

if ! command -v python3 >/dev/null 2>&1; then
  echo "[smoke] python3 required"
  exit 1
fi

python3 -c "import grpc" 2>/dev/null || { echo "[smoke] pip install grpcio"; exit 1; }

export ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}"

ATLAS_PID=""
if [[ "${SMOKE_USE_EXISTING_ATLAS:-0}" != "1" ]]; then
  echo "[smoke] starting robonix-atlas..."
  cargo run --manifest-path "$RUST_ROOT/Cargo.toml" -p robonix-atlas &
  ATLAS_PID=$!
  sleep 2
fi

cleanup() {
  if [[ -n "$ATLAS_PID" ]]; then
    kill "$ATLAS_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

python3 "$RUST_ROOT/examples/scripts/smoke_control_plane.py"
EC=$?
echo "[smoke] exit=$EC"
exit "$EC"
