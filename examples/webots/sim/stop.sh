#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Tear down everything `bash sim/start.sh` + `rbnx boot` brought up:
# webots compose stack + rviz2 + rtabmap_viz (via `docker rm` on the
# mapping container) + every host process and in-container driver
# spawned by rbnx boot.
#
# Symmetric pair to sim/start.sh: every GUI / process that script
# brings up MUST be killed here, otherwise the next start hits port
# / container-name collisions.
set -euo pipefail

cd "$(dirname "$0")"
WEBOTS_DIR="$(cd .. && pwd -P)"

# Kill only the processes that belong to THIS webots deployment. A plain
# `pkill -f robonix-atlas` also took down every other Robonix stack on the
# host (the lab deployment under ~/robonix-lab/deploy was killed by every
# benchmark world and every CI run). A process is ours when its working
# directory or command line is under this checkout's examples/webots, or an
# ancestor's is (children of `rbnx boot` inherit the deployment cwd; python
# services started from their package dir still descend from it).
ours() {
    local pid="$1" depth=0 cwd cmd
    while [ -n "$pid" ] && [ "$pid" != 1 ] && [ "$depth" -lt 8 ]; do
        cwd="$(readlink "/proc/$pid/cwd" 2>/dev/null || true)"
        cmd="$(tr '\0' ' ' < "/proc/$pid/cmdline" 2>/dev/null || true)"
        case "$cwd" in "$WEBOTS_DIR"*) return 0 ;; esac
        case "$cmd" in *"$WEBOTS_DIR"*) return 0 ;; esac
        pid="$(awk '/^PPid:/ {print $2}' "/proc/$pid/status" 2>/dev/null || true)"
        depth=$((depth + 1))
    done
    return 1
}
kill_ours() {
    local pid
    for pid in $(pgrep -f "$1" 2>/dev/null || true); do
        [ "$pid" = "$$" ] && continue
        if ours "$pid"; then kill -9 "$pid" 2>/dev/null || true; fi
    done
}

echo "[sim/stop] killing host-side robonix processes of $WEBOTS_DIR (atlas / executor / soma / pilot / vitals / liaison / rbnx boot)..."
# Every binary spawned by `rbnx boot`'s system: block must be listed here,
# otherwise its TCP port leaks across boot cycles and the next boot fails
# with `listen address ':50081' is taken`. Add new ones to deploy.rs's
# system-bin table AND to this regex.
kill_ours "rbnx boot|rbnx deploy|rbnx start -p|robonix-atlas|robonix-executor|robonix-soma|robonix-pilot|robonix-vitals|robonix-liaison"

echo "[sim/stop] killing host-side python service zombies (speech / memsearch / scene / audio drivers / nav bridges)..."
# Host-side packages spawn long-lived Python processes in their own
# rbnx-build/venv. `rbnx shutdown` SIGTERMs the rbnx-cli wrapper, but if
# the wrapper dies first (or boot races), the Python child outlives it
# and keeps its GPU memory + gRPC port. Speech_service is the worst
# offender: every leaked instance pins ~1 GiB of CUDA on the FunASR
# model and a stale TCP port that the next boot can't rebind.
kill_ours "speech_service\\.service"
kill_ours "memsearch_service\\.service"
kill_ours "scene_service\\.service"
kill_ours "audio_driver\\.main|audio_driver\\.node"
kill_ours "audio_client_bridge\\.main|audio_client_bridge\\.node"
kill_ours "voiceprint_service\\.service"
kill_ours "simple_nav\\.atlas_bridge"
kill_ours "mapping_service\\.service"

# Quick GPU memory check — visible signal that the kills actually
# released CUDA. nvidia-smi may be absent on non-GPU hosts; that's fine.
if command -v nvidia-smi >/dev/null 2>&1; then
    used_free=$(nvidia-smi --query-gpu=memory.used,memory.free --format=csv,noheader 2>/dev/null | head -1 || true)
    if [ -n "${used_free:-}" ]; then
        echo "[sim/stop] GPU after host-side cleanup: ${used_free}"
    fi
fi

echo "[sim/stop] killing host-side rviz2 wrapper (docker exec into sim)..."
# sim/start.sh launches `bash sim/start_rviz.sh` in the background;
# that script does `docker exec robonix_tiago_sim ... rviz2`. Kill the
# wrapper before the compose-down below so docker doesn't have to GC
# a half-dead exec.
kill_ours "start_rviz.sh|rviz2 -d /tmp/rviz2_default.rviz"

echo "[sim/stop] killing in-container drivers + sim-side GUIs..."
# Container names must match what start.sh/compose.yaml actually created.
# CI runs give every container a per-run prefix so concurrent runs and
# interactive users coexist on the shared box; hardcoding the defaults here
# tore down someone else's stack and leaked our own.
SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"
MAPPING_CT="${ROBONIX_MAPPING_CONTAINER:-robonix_mapping}"
SCENE_CT="${ROBONIX_SCENE_CONTAINER:-robonix_scene}"
EXPLORE_CT="${ROBONIX_EXPLORE_CONTAINER:-robonix_explore}"

# Match the actual driver module names spawned by rbnx boot:
#   camera_driver.driver, chassis_driver.driver, lidar_driver.driver,
#   simple_nav.atlas_bridge.
# These leak across boot cycles when rbnx shutdown kills only the host
# wrappers — the python inside the sim container survives and holds
# its gRPC port (50212/50122/...), making the next boot fail with
# "address already in use".
docker exec "$SIM_CT" sh -c \
  'pkill -9 -f "_driver\\.driver|simple_nav\\.atlas_bridge|nav2_bringup|memsearch_service|rviz2 -d" 2>/dev/null || true' \
  2>/dev/null || true

echo "[sim/stop] removing per-package containers (mapping/scene/explore)..."
# These are spawned by rbnx boot (one container per service package).
# `rbnx shutdown` SIGTERMs the host wrapper, but if docker-stop raced
# or boot crashed, the containers leak and the next boot hits a name
# collision. Force-remove so the next start is clean. This also takes
# rtabmap_viz with it (it's a child of the mapping container).
for ct in "$MAPPING_CT" "$SCENE_CT" "$EXPLORE_CT"; do
    docker rm -f "$ct" >/dev/null 2>&1 || true
done

echo "[sim/stop] docker compose down (sim container + volumes left intact)..."
docker compose -f compose.yaml down 2>/dev/null || true

echo "[sim/stop] done."
