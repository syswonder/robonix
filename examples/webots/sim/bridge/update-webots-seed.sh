#!/usr/bin/env bash
# Re-export the Webots office cache for a robonix-assets Release.
# Run this on a machine where the container has successfully downloaded the
# full asset set (i.e. Webots has opened the world at least once).
# Usage:   ./update-webots-seed.sh [container_name] [output_path]
# Default: robonix_tiago_sim
set -euo pipefail
CONTAINER="${1:-robonix_tiago_sim}"
OUT="${2:-${TMPDIR:-/tmp}/webots-office-seed-v3.tar.gz}"

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
echo "[update-seed] sha256: $(sha256sum "$OUT" | awk '{print $1}')"
