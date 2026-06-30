#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# tiago_lidar runtime — docker-exec into the pre-running sim container.
# See tiago_chassis/scripts/start.sh for trap-discipline rationale.
set -euo pipefail

# Sim container name — overridable via ROBONIX_SIM_CONTAINER for isolated
# CI / parallel deploys. Default keeps single-deploy behaviour. See
# tiago_chassis/scripts/start.sh for the full rationale.
SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"

if ! docker ps --format '{{.Names}}' | grep -qx "$SIM_CT"; then
  echo "[tiago_lidar] error: sim container '$SIM_CT' is not running."
  echo "              Bring it up first:  bash examples/webots/sim/start.sh"
  exit 1
fi

cleanup() {
  docker exec "$SIM_CT" pkill -9 -f 'lidar_driver|scan_normalize' 2>/dev/null || true
  kill -- "-$$" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# Webots's LaserScan publisher has known quirks (reversed angle_increment,
# scan_time/time_increment unset, stamp lags one sim step) that wreck
# downstream SLAM. The lidar primitive owns this fix locally —
# `scan_normalize.py` republishes the raw /scanner as a standards-compliant
# scan, and the primitive declares the NORMALISED topic to atlas. Mapping
# (and any other consumer) only ever sees clean scans; webots-specific
# compensation never leaks into generic services.
RAW_TOPIC="${TIAGO_SCAN_RAW_TOPIC:-/scanner}"
OUT_TOPIC="${TIAGO_SCAN_TOPIC:-/scanner_normalized}"

# Cross-host wiring for an isolated (bridge-network) sim — see tiago_chassis
# start.sh for the rationale. ROBONIX_SIM_ATLAS = atlas reachable from inside the
# container; ROBONIX_ADVERTISE_HOST = the container's own IP (no `ip` cmd inside
# to self-resolve). Both empty on a host-network sim, where 127.0.0.1 works.
SIM_IP="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$SIM_CT" 2>/dev/null || true)"

docker exec -i \
  -e ROBONIX_ATLAS="${ROBONIX_SIM_ATLAS:-${ROBONIX_ATLAS:-127.0.0.1:50051}}" \
  -e ROBONIX_ADVERTISE_HOST="${ROBONIX_ADVERTISE_HOST:-$SIM_IP}" \
  -e ROBONIX_PKG_HOST_DIR="$(cd "$(dirname "$0")/.." && pwd)" \
  -e TIAGO_SCAN_TOPIC="$OUT_TOPIC" \
  -e TIAGO_SCAN_RAW_TOPIC="$RAW_TOPIC" \
  -e PYTHONPATH="/robonix_pkgs/pylib/robonix-api" \
  "$SIM_CT" \
  bash -lc "
    set -eo pipefail
    source /opt/ros/humble/setup.bash
    OVL=/robonix_pkgs/primitives/tiago_lidar/rbnx-build/codegen/ros2_idl/install/setup.bash
    [ -f \"\$OVL\" ] && source \"\$OVL\" || true
    python3 /robonix_pkgs/primitives/tiago_lidar/scripts/scan_normalize.py \\
        --in $RAW_TOPIC --out $OUT_TOPIC &
    NORM_PID=\$!
    trap 'kill -TERM \$NORM_PID 2>/dev/null || true' EXIT
    cd /robonix_pkgs/primitives/tiago_lidar
    exec python3 -m lidar_driver.driver
  " &
wait $!
