#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Run rviz2 inside the webots sim container so it sees the same DDS bus
# without needing host ros installed. Default config shows /map +
# /scanner + /tf + 2D Nav Goal tool publishing to /goal_pose (which
# simple_nav subscribes to). X11 routes through the same DISPLAY the
# sim already uses.
set -euo pipefail

SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"
RVIZ_CFG_HOST="$(cd "$(dirname "$0")" && pwd)/rviz2_default.rviz"
RVIZ_CFG_CT="/tmp/rviz2_default.rviz"

if ! docker ps --filter name="$SIM_CT" -q | grep -q .; then
    echo "[start_rviz] sim container '$SIM_CT' not running; bring it up first via sim/start.sh"
    exit 1
fi

docker cp "$RVIZ_CFG_HOST" "$SIM_CT":"$RVIZ_CFG_CT" >/dev/null

# Match the rest of the stack: FastRTPS UDP-only profile so cross-
# container DDS lines up with mapping/scene.
docker exec -i \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e XAUTHORITY=/root/.Xauthority \
    -e QT_X11_NO_MITSHM=1 \
    "$SIM_CT" bash -lc "
        set -eo pipefail
        source /opt/ros/humble/setup.bash
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        exec ros2 run rviz2 rviz2 -d $RVIZ_CFG_CT
    "
