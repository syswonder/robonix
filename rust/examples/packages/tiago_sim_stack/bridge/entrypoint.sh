#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Webots (GUI) + eaios_webots launch → Nav2 bringup → tiago_bridge (foreground).
set -euo pipefail

source /opt/ros/humble/setup.bash
source /colcon_ws/install/setup.bash

WEBOTS_WARMUP_SEC="${WEBOTS_WARMUP_SEC:-25}"
NAV2_WARMUP_SEC="${NAV2_WARMUP_SEC:-15}"

ros2 launch eaios_webots robot_launch.py use_sim_time:=true &
_webots_launch_pid=$!
echo "[entrypoint] eaios_webots pid=${_webots_launch_pid}"
sleep "${WEBOTS_WARMUP_SEC}"

ros2 launch nav2_bringup bringup_launch.py \
  map:=/opt/robonix_nav2/config/my_map.yml \
  use_sim_time:=true \
  params_file:=/opt/robonix_nav2/config/nav2_params.yml &
_nav2_launch_pid=$!
echo "[entrypoint] nav2 pid=${_nav2_launch_pid}"
sleep "${NAV2_WARMUP_SEC}"

exec python3 -m tiago_bridge.node
