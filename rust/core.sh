#!/bin/bash

set -e

ROBONIX_WEB_ASSETS_DIR="$(pwd)/robonix-core/web" \
ROBONIX_WEB_PORT=8000 \
RUST_LOG=robonix_core=info robonix-core &
ROBONIX_PID=$!

# Wait for robonix-core, so we can catch Ctrl-C and clean up
wait $ROBONIX_PID