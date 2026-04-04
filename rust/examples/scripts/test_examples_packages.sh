#!/usr/bin/env bash
# Validate RFC002 manifests under examples/packages (no ROS / Docker required).
set -euo pipefail

RUST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$RUST_ROOT"

echo "[test] rbnx validate vlm_service"
cargo run -p robonix-cli -- validate "$RUST_ROOT/examples/packages/vlm_service"

echo "[test] rbnx validate tiago_sim_stack"
cargo run -p robonix-cli -- validate "$RUST_ROOT/examples/packages/tiago_sim_stack"

echo "[test] rbnx validate memsearch_service"
cargo run -p robonix-cli -- validate "$RUST_ROOT/examples/packages/memsearch_service"

echo "[test] rbnx validate zero_copy_demo"
cargo run -p robonix-cli -- validate "$RUST_ROOT/examples/packages/zero_copy_demo"

echo "[test] rbnx validate maniskill_vla_demo"
cargo run -p robonix-cli -- validate "$RUST_ROOT/examples/packages/maniskill_vla_demo"

echo "[test] done"
