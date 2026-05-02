#!/usr/bin/env bash
# Re-export the Webots asset cache tarball used by Dockerfile.
# Run this on a machine where the container has successfully downloaded the
# full asset set (i.e. Webots has opened the world at least once).
# Usage:   ./update-webots-seed.sh [container_name]
# Default: robonix_tiago_sim_stack-ros2-bridge-1
set -euo pipefail
CONTAINER="${1:-robonix_tiago_sim_stack-ros2-bridge-1}"
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="${HERE}/webots_assets_seed.tar.gz"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "container not running: $CONTAINER" >&2
    echo "start the stack first (./run.sh start), wait for Webots to finish downloading," >&2
    echo "then rerun this script." >&2
    exit 1
fi

echo "[update-seed] exporting cache from $CONTAINER..."
docker exec "$CONTAINER" bash -c '
    cd /root/.cache/Cyberbotics/Webots && tar -czf - assets/
' > "$OUT"

echo "[update-seed] wrote $(ls -lh "$OUT" | awk "{print \$5}") → $OUT"
echo "[update-seed] file count: $(tar -tzf "$OUT" | grep -v "/$" | wc -l)"
