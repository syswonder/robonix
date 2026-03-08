#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Run Zenoh cross-ROS test: build and start Humble + Jazzy + Zenoh, then
# run bridges and pub/sub; exit 0 if Jazzy receives messages (PASS).

set -e
cd "$(dirname "$0")"

echo "=== Building and starting containers (humble, jazzy) ==="
docker compose up -d --build
trap 'docker compose down' EXIT

echo "=== Waiting for containers and Zenoh listen ==="
sleep 6

echo "=== Starting Jazzy bridge (subscriber side) in background ==="
docker compose exec -d jazzy bash -c 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && python3 /workspace/scripts/jazzy_bridge.py'
sleep 4

echo "=== Starting Humble bridge (publisher side) in background ==="
docker compose exec -d humble bash -c 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && python3 /workspace/scripts/humble_bridge.py'
sleep 4

# Jazzy subscriber must be running BEFORE we publish, so it receives while humble_pub sends.
echo "=== Starting Jazzy subscriber in background (must be ready before publisher) ==="
TMPOUT=$(mktemp)
trap 'rm -f "$TMPOUT"; docker compose down' EXIT
docker compose exec jazzy bash -c 'source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash && python3 /workspace/scripts/jazzy_sub.py' > "$TMPOUT" 2>&1 &
SUB_PID=$!
sleep 2

echo "=== Running Humble publisher (sends 10 Point3D messages) ==="
docker compose exec humble bash -c 'source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && python3 /workspace/scripts/humble_pub.py'

wait $SUB_PID 2>/dev/null || true
out=$(cat "$TMPOUT")
echo "$out"
if echo "$out" | grep -q "PASS:"; then
  echo "=== Zenoh cross-ROS test PASSED ==="
  exit 0
fi
echo "=== Zenoh cross-ROS test FAILED (no PASS from jazzy_sub) ==="
echo "=== Container logs (for debugging) ==="
docker compose logs humble jazzy 2>&1 | tail -80
exit 1
