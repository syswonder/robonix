#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Stop one Webots simulator instance.
#
# This script deliberately does not emulate `rbnx shutdown`: it never searches
# for Atlas, Pilot, package drivers, Python services, or per-package containers.
# Run `rbnx shutdown` separately to stop the Robonix deployment that uses this
# simulator.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck source=runtime_state.sh
source "$SCRIPT_DIR/runtime_state.sh"
robonix_sim_init_runtime_state

rviz_pid="$(robonix_sim_read_rviz_pid "$ROBONIX_SIM_RVIZ_PID_FILE" 2>/dev/null || true)"
if [[ -n "$rviz_pid" ]] \
    && robonix_sim_rviz_pid_matches "$rviz_pid" "$SCRIPT_DIR/start_rviz.sh"; then
    echo "[sim/stop] stopping recorded rviz2 wrapper (pid $rviz_pid)..."
    kill "$rviz_pid" 2>/dev/null || true
else
    rviz_pid=""
fi

echo "[sim/stop] stopping compose project '$ROBONIX_SIM_PROJECT' (container '$ROBONIX_SIM_CONTAINER')..."
ROBONIX_SIM_CONTAINER="$ROBONIX_SIM_CONTAINER" \
ROBONIX_SIM_PROJECT="$ROBONIX_SIM_PROJECT" \
    docker compose \
        --project-name "$ROBONIX_SIM_PROJECT" \
        -f "$SCRIPT_DIR/compose.yaml" \
        down --remove-orphans

# Compose down normally releases docker exec and lets the wrapper exit. Use
# SIGKILL only when the exact recorded wrapper still exists after that.
if [[ -n "$rviz_pid" ]] \
    && robonix_sim_rviz_pid_matches "$rviz_pid" "$SCRIPT_DIR/start_rviz.sh"; then
    kill -KILL "$rviz_pid" 2>/dev/null || true
fi
rm -f -- "$ROBONIX_SIM_RVIZ_PID_FILE"

echo "[sim/stop] done. Robonix components, if any, are unchanged; use 'rbnx shutdown' separately."
