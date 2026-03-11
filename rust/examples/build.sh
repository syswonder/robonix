#!/usr/bin/env bash
# 构建 python_ping_client package
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RUST_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PACKAGE_DIR="$(realpath "$SCRIPT_DIR/python_ping_client")"
ROBONIX_CLI="$RUST_DIR/robonix-cli/Cargo.toml"

cargo run --manifest-path "$ROBONIX_CLI" -- package validate "$PACKAGE_DIR"
cargo run --manifest-path "$ROBONIX_CLI" -- build -p "$PACKAGE_DIR"

# cargo run --manifest-path ../robonix-cli/Cargo.toml -- start -p ./python_ping_client -n call_ping