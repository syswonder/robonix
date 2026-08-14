#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# tiago_lidar runtime — docker-exec into the pre-running sim container.
# Lifecycle cleanup is handled by Driver(CMD_SHUTDOWN) + manifest stop hook.
set -euo pipefail

# Sim container name — overridable via ROBONIX_SIM_CONTAINER for isolated
# CI / parallel deploys. Default keeps single-deploy behaviour.
SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"

if ! docker ps --format '{{.Names}}' | grep -qx "$SIM_CT"; then
  echo "[tiago_lidar] error: sim container '$SIM_CT' is not running."
  echo "              Bring it up first:  bash examples/webots/sim/start.sh"
  exit 1
fi

# Webots publishes scans with a reversed angle definition. The lidar primitive
# owns that orientation fix locally: `scan_normalize.py` reverses the samples
# and makes angle_increment positive. A Webots range image represents one
# simulation timestamp, so the relay passes header.stamp, scan_time, and
# time_increment through unchanged. Mapping and other consumers use the
# normalized topic without Webots-specific timing compensation.
RAW_TOPIC="${TIAGO_SCAN_RAW_TOPIC:-/scanner}"
OUT_TOPIC="${TIAGO_SCAN_TOPIC:-/scanner_normalized}"

# Cross-host wiring for an isolated (bridge-network) sim — see tiago_chassis
# start.sh for the rationale. Host-network sim containers do not have a bridge
# IP, so fall back to localhost unless Docker returns a valid bridge IPv4.
resolve_advertise_host() {
  if [ -n "${ROBONIX_ADVERTISE_HOST:-}" ]; then
    printf '%s\n' "$ROBONIX_ADVERTISE_HOST"
    return
  fi
  local network_mode inspected
  network_mode="$(docker inspect -f '{{.HostConfig.NetworkMode}}' "$SIM_CT" 2>/dev/null || true)"
  if [ "$network_mode" = "host" ]; then
    printf '%s\n' "127.0.0.1"
    return
  fi
  inspected="$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' "$SIM_CT" 2>/dev/null || true)"
  if [[ "$inspected" =~ ^([0-9]{1,3}\.){3}[0-9]{1,3}$ ]]; then
    printf '%s\n' "$inspected"
    return
  fi
  printf '%s\n' "127.0.0.1"
}

ADVERTISE_HOST="$(resolve_advertise_host)"

exec docker exec \
  -e ROBONIX_ATLAS="${ROBONIX_SIM_ATLAS:-${ROBONIX_ATLAS:-127.0.0.1:50051}}" \
  -e ROBONIX_ADVERTISE_HOST="$ADVERTISE_HOST" \
  -e ROBONIX_PKG_HOST_DIR="$(cd "$(dirname "$0")/.." && pwd)" \
  -e TIAGO_SCAN_TOPIC="$OUT_TOPIC" \
  -e TIAGO_SCAN_RAW_TOPIC="$RAW_TOPIC" \
  -e RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}" \
  -e PYTHONPATH="/robonix_pkgs/pylib/robonix-api:/robonix_pkgs/primitives/tiago_lidar/rbnx-build/codegen/proto_gen:/robonix_pkgs/primitives/tiago_lidar/rbnx-build/codegen/robonix_mcp_types" \
  "$SIM_CT" \
  bash -lc '''
    set -eo pipefail
    set +u
    source /opt/ros/humble/setup.bash >/dev/null
    OVL=/robonix_pkgs/primitives/tiago_lidar/rbnx-build/codegen/ros2_idl/install/setup.bash
    [ -f "$OVL" ] && source "$OVL" >/dev/null || true
    python3 /robonix_pkgs/primitives/tiago_lidar/scripts/scan_normalize.py \
        --in "$TIAGO_SCAN_RAW_TOPIC" --out "$TIAGO_SCAN_TOPIC" &
    NORM_PID=$!
    trap "kill -TERM \"$NORM_PID\" 2>/dev/null || true" EXIT
    cd /robonix_pkgs/primitives/tiago_lidar
    LOG=/tmp/tiago_lidar_driver.log
    : > "$LOG"
    python3 -m lidar_driver.driver >>"$LOG" 2>&1 &
    DRIVER_PID=$!
    tail --pid="$DRIVER_PID" -n +1 -F "$LOG" &
    TAIL_PID=$!
    set +e
    wait "$DRIVER_PID"
    STATUS=$?
    set -e
    kill "$TAIL_PID" 2>/dev/null || true
    wait "$TAIL_PID" 2>/dev/null || true
    exit "$STATUS"
  '''
