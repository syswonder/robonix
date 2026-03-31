#!/bin/bash
# Build and run the RTAB-Map Docker container for ManiSkill VLA demo.
#
# Prerequisites:
#   - env_node must be running (provides RGB-D via gRPC)
#   - robonix-server must be running on ROBONIX_SERVER (default localhost:50051)
#   - (optional) Rerun viewer: rerun --serve-grpc
#
# Usage:
#   ./docker/run_rtabmap.sh
#
# Override env vars:
#   ROBONIX_SERVER=host:port  ENV_GRPC_ENDPOINT=host:port  RERUN_GRPC_URL=...

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

IMAGE_NAME="robonix-rtabmap:latest"

echo "==> Building Docker image: ${IMAGE_NAME}"
docker build \
    -f "$PKG_DIR/docker/Dockerfile.rtabmap" \
    -t "$IMAGE_NAME" \
    "$PKG_DIR"

echo "==> Running RTAB-Map bridge container"
# Use -i (keep stdin open) but NOT -t: when started from run.sh as a
# background job there is no TTY, so -t causes Docker to exit immediately
# with "the input device is not a TTY".
docker run --rm -i \
    --network host \
    --ipc host \
    -e ROBONIX_SERVER="${ROBONIX_SERVER:-localhost:50051}" \
    -e RERUN_GRPC_URL="${RERUN_GRPC_URL:-rerun+http://localhost:9877}" \
    -e ENV_GRPC_ENDPOINT="${ENV_GRPC_ENDPOINT:-}" \
    -e BRIDGE_FPS="${BRIDGE_FPS:-10}" \
    -e MAP_LOG_INTERVAL="${MAP_LOG_INTERVAL:-30}" \
    -e OMP_WAIT_POLICY=passive \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    "$IMAGE_NAME"
