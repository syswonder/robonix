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

echo "[sim/stop] killing host-side robonix processes (atlas / executor / soma / pilot / vitals / liaison / rbnx boot)..."
# Every binary spawned by `rbnx boot`'s system: block must be listed here,
# otherwise its TCP port leaks across boot cycles and the next boot fails
# with `listen address ':50081' is taken`. Add new ones to deploy.rs's
# system-bin table AND to this regex.
pkill -9 -f "rbnx boot|rbnx deploy|rbnx start -p|robonix-atlas|robonix-executor|robonix-soma|robonix-pilot|robonix-vitals|robonix-liaison" 2>/dev/null || true

echo "[sim/stop] killing host-side python service zombies (speech / memsearch / scene / audio drivers / nav bridges)..."
# Host-side packages spawn long-lived Python processes in their own
# rbnx-build/venv. `rbnx shutdown` SIGTERMs the rbnx-cli wrapper, but if
# the wrapper dies first (or boot races), the Python child outlives it
# and keeps its GPU memory + gRPC port. Speech_service is the worst
# offender: every leaked instance pins ~1 GiB of CUDA on the FunASR
# model and a stale TCP port that the next boot can't rebind.
pkill -9 -f "speech_service\\.service" 2>/dev/null || true
pkill -9 -f "memsearch_service\\.service" 2>/dev/null || true
pkill -9 -f "scene_service\\.service" 2>/dev/null || true
pkill -9 -f "audio_driver\\.main|audio_driver\\.node" 2>/dev/null || true
pkill -9 -f "audio_client_bridge\\.main|audio_client_bridge\\.node" 2>/dev/null || true
pkill -9 -f "voiceprint_service\\.service" 2>/dev/null || true
pkill -9 -f "simple_nav\\.atlas_bridge" 2>/dev/null || true
pkill -9 -f "mapping_service\\.service" 2>/dev/null || true

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
pkill -9 -f "start_rviz.sh|rviz2 -d /tmp/rviz2_default.rviz" 2>/dev/null || true

echo "[sim/stop] killing in-container drivers + sim-side GUIs..."
# Match the actual driver module names spawned by rbnx boot:
#   camera_driver.driver, chassis_driver.driver, lidar_driver.driver,
#   simple_nav.atlas_bridge.
# These leak across boot cycles when rbnx shutdown kills only the host
# wrappers — the python inside the sim container survives and holds
# its gRPC port (50212/50122/...), making the next boot fail with
# "address already in use".
docker exec robonix_tiago_sim sh -c \
  'pkill -9 -f "_driver\\.driver|simple_nav\\.atlas_bridge|nav2_bringup|memsearch_service|rviz2 -d" 2>/dev/null || true' \
  2>/dev/null || true

echo "[sim/stop] removing per-package containers (mapping/scene/explore)..."
# These are spawned by rbnx boot (one container per service package).
# `rbnx shutdown` SIGTERMs the host wrapper, but if docker-stop raced
# or boot crashed, the containers leak and the next boot hits a name
# collision. Force-remove so the next start is clean. This also takes
# rtabmap_viz with it (it's a child of the mapping container).
for ct in robonix_mapping robonix_scene robonix_explore; do
    docker rm -f "$ct" >/dev/null 2>&1 || true
done

echo "[sim/stop] docker compose down (sim container + volumes left intact)..."
docker compose -f compose.yaml down 2>/dev/null || true

echo "[sim/stop] done."
