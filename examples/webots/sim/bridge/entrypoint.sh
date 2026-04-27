#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Sim ENVIRONMENT only — Webots + eaios_webots controller. Nav2 lives in
# the tiago_nav2 service package (started by `rbnx boot`); robonix
# drivers (tiago_chassis / tiago_camera / tiago_lidar) live in their
# respective primitive packages and are exec'd into THIS container by
# `rbnx boot` via `docker exec`. So this container is the host for
# both the simulator and (later) every robonix driver process.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash
set -u

WEBOTS_WARMUP_SEC="${WEBOTS_WARMUP_SEC:-25}"

ros2 launch eaios_webots robot_launch.py use_sim_time:=true &
_webots_launch_pid=$!
echo "[entrypoint] eaios_webots pid=${_webots_launch_pid}"
sleep "${WEBOTS_WARMUP_SEC}"

# Stay alive so `docker exec` from rbnx-driven driver packages can land
# inside this container. Ctrl-C (compose down) propagates SIGTERM here
# which `wait` will then forward to the launch process.
wait ${_webots_launch_pid}
