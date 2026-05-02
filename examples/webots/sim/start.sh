#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Bring up the Tiago Webots sim container. Run this BEFORE `rbnx boot`
# from examples/webots/ — robonix drivers are docker-exec'd into the
# container started here, so the container has to exist first.
#
# Auto-detects nvidia-smi to merge compose.gpu.yaml. To force CPU-only,
# unset CUDA_VISIBLE_DEVICES or set ROBONIX_FORCE_CPU=1.
#
# Re-running is safe: docker compose up reuses the running container.
# Stop with Ctrl-C, or from another terminal: `docker compose -f compose.yaml down`.
set -euo pipefail

cd "$(dirname "$0")"

CF=(-f compose.yaml)
if [[ "${ROBONIX_FORCE_CPU:-0}" != "1" ]] && command -v nvidia-smi &>/dev/null && nvidia-smi &>/dev/null; then
  CF+=(-f compose.gpu.yaml)
  echo "[sim/start] NVIDIA GPU detected — merging compose.gpu.yaml"
else
  echo "[sim/start] no GPU (or ROBONIX_FORCE_CPU=1) — CPU-only Webots"
fi

# X11 GUI: ensure docker can reach the local DISPLAY. xhost is harmless
# on systems without an X server (it just fails silently).
if command -v xhost &>/dev/null; then
  xhost +local:docker >/dev/null 2>&1 || true
fi

exec docker compose "${CF[@]}" up --build
