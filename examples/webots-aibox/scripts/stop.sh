#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Sweep one primitive's leftovers on the sim host. Normal teardown reaches
# the driver via Atlas Driver(CMD_SHUTDOWN) first; this stop hook is the
# belt-and-braces for the remote side, because a live remote `rbnx start`
# wrapper would respawn its driver otherwise. Kill order matters: wrapper
# first, then any orphaned driver inside the sim container.
set -euo pipefail

PKG="${1:?usage: stop.sh <primitive-package-name>}"
DIR="$(cd "$(dirname "$0")/.." && pwd)"
[[ -f "$DIR/remote.env" ]] || {
  echo "[webots-aibox] missing $DIR/remote.env — copy remote.env.example and fill in the machine values" >&2
  exit 1
}
# shellcheck disable=SC1091
source "$DIR/remote.env"
# Matches compose.yaml's container_name default; set SIM_CT in remote.env to
# override a custom ROBONIX_SIM_CONTAINER deployment.
SIM_CT="${SIM_CT:-robonix_tiago_sim}"

# Driver process patterns inside the sim container, mirroring the stop
# blocks in examples/webots/primitives/*/package_manifest.yaml. Bracketed
# first chars keep the patterns from matching their own command line.
case "$PKG" in
  tiago_chassis) DRIVER='[p]ython3 -m chassis_driver.driver' ;;
  tiago_camera)  DRIVER='[c]amera_driver|[s]tatic_transform_publisher.*head_front_camera' ;;
  tiago_lidar)   DRIVER='[l]idar_driver|[s]can_normalize' ;;
  tiago_health)  DRIVER='[p]ython3 -m tiago_health.driver' ;;
  *) echo "[webots-aibox] unknown primitive package: $PKG" >&2; exit 2 ;;
esac

# Best effort by design: the sim host may already be unreachable, in which
# case Atlas GC reclaims the registration and this must not block teardown.
ssh -o BatchMode=yes -o ConnectTimeout=10 "$HOST_SSH" "
  pkill -TERM -f '[r]bnx start -p .*$PKG' 2>/dev/null || true
  sleep 1
  pkill -KILL -f '[r]bnx start -p .*$PKG' 2>/dev/null || true
  docker exec '$SIM_CT' pkill -TERM -f '$DRIVER' 2>/dev/null || true
  sleep 1
  docker exec '$SIM_CT' pkill -KILL -f '$DRIVER' 2>/dev/null || true
" || true
