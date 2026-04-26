#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail

if ! docker ps --format '{{.Names}}' | grep -qx robonix_tiago_sim; then
  echo "[tiago_lidar] error: sim container 'robonix_tiago_sim' is not running."
  echo "              Bring it up first:  bash examples/webots/sim/start.sh"
  exit 1
fi

exec docker exec -i \
  -e ROBONIX_ATLAS="${ROBONIX_ATLAS:-127.0.0.1:50051}" \
  -e TIAGO_LIDAR_MCP_PORT="${TIAGO_LIDAR_MCP_PORT:-50113}" \
  -e TIAGO_SCAN_TOPIC="${TIAGO_SCAN_TOPIC:-/scanner/scan}" \
  robonix_tiago_sim \
  bash -lc 'source /opt/ros/humble/setup.bash && \
            cd /robonix_pkgs/primitives/tiago_lidar && \
            exec python3 -m lidar_driver.driver'
