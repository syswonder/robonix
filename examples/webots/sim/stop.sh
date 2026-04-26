#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Tear down everything `bash sim/start.sh` + `rbnx deploy` brought up.
# Run this when you want a clean slate (sim container + atlas/pilot/
# executor + every docker-exec'd driver inside the sim container).
set -euo pipefail

cd "$(dirname "$0")"

echo "[sim/stop] killing host-side robonix processes (atlas / pilot / executor / rbnx deploy)..."
pkill -9 -f "rbnx deploy|robonix-atlas|robonix-pilot|robonix-executor|rbnx start -p" 2>/dev/null || true

echo "[sim/stop] killing in-container drivers (chassis/camera/lidar/nav2 + nav2_bringup)..."
docker exec robonix_tiago_sim sh -c \
  'pkill -9 -f "_driver.driver|nav2_bringup|memsearch_service" 2>/dev/null || true' \
  2>/dev/null || true

echo "[sim/stop] docker compose down (sim container + volumes left intact)..."
docker compose -f compose.yaml down 2>/dev/null || true

echo "[sim/stop] done."
