#!/bin/bash

set -e

export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

cleanup() {
    echo ""
    echo "Cleaning up: killing robonix-core, found pid(s): $(pgrep -x robonix-core | sort -n)"
    pgrep -x robonix-core | sort -n | xargs -r kill -9
    wait $ROBONIX_PID 2>/dev/null || true
    echo "Making sure robonix-core does not exist..."
    if pgrep -x robonix-core >/dev/null; then
        echo "robonix-core still exists"
        exit 1
    fi
    echo "Cleanup complete!"
    exit 0
}

trap cleanup SIGINT SIGTERM

ROBONIX_WEB_ASSETS_DIR="$(pwd)/robonix-core/web" \
ROBONIX_WEB_PORT=8000 \
RUST_LOG=robonix_core=info robonix-core &
ROBONIX_PID=$!

wait $ROBONIX_PID

trap cleanup SIGINT SIGTERM