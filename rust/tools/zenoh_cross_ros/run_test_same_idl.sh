#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Test 2: Same IDL, cross-distro (Humble -> Jazzy). Uses zenoh-bridge-ros2dds because
# rmw_zenoh does not support Humble<->Jazzy (type-hash mismatch in key expressions).

set -e
cd "$(dirname "$0")"

COMPOSE_FILE=docker-compose.same-idl-bridge.yaml
echo "=== Test 2: Same IDL via zenoh-bridge-ros2dds (Humble + Jazzy) ==="
echo "=== Building and starting bridge_jazzy + bridge_humble + humble + jazzy ==="
docker compose -f "$COMPOSE_FILE" up -d --build
trap "docker compose -f $COMPOSE_FILE down" EXIT

echo "=== Waiting for bridges and nodes ==="
sleep 10

echo "=== Starting Jazzy subscriber in background ==="
TMPOUT=$(mktemp)
trap "rm -f $TMPOUT; docker compose -f $COMPOSE_FILE down" EXIT
docker compose -f "$COMPOSE_FILE" exec jazzy bash -c 'source /opt/ros/jazzy/setup.bash && python3 /workspace/scripts/same_idl_sub.py' > "$TMPOUT" 2>&1 &
SUB_PID=$!
sleep 4

echo "=== Running Humble publisher (10 x geometry_msgs/Point on /point) ==="
docker compose -f "$COMPOSE_FILE" exec humble bash -c 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && python3 /workspace/scripts/same_idl_pub.py'

wait $SUB_PID 2>/dev/null || true
out=$(cat "$TMPOUT")
echo "$out"
if echo "$out" | grep -q "PASS:"; then
  echo "=== Same-IDL (bridge) test PASSED ==="
  exit 0
fi
echo "=== Same-IDL (bridge) test FAILED ==="
docker compose -f "$COMPOSE_FILE" logs humble jazzy bridge_humble bridge_jazzy 2>&1 | tail -80
exit 1
