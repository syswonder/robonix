#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Run rviz2 inside the webots sim container so it sees the same DDS bus
# without needing host ros installed. The config is the Nav2 view: /map +
# global/local costmaps + global/local plans + /scanner_normalized + TF +
# odometry, plus the Nav2 control panel and the "Nav2 Goal" tool.
#
# rviz runs with use_sim_time:=true so its clock tracks /clock — the
# map->odom TF (from rtabmap, on sim time) and the goal rviz stamps then
# share a timeline. Without it the goal carries a wall-clock stamp the
# planner can't transform ("Extrapolation … into the future"), so every
# plan aborts and the robot just spin-recovers in place.
#
# X11 routes through the same DISPLAY the sim already uses.
set -euo pipefail

SIM_CT="${ROBONIX_SIM_CONTAINER:-robonix_tiago_sim}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RVIZ_CFG_HOST="$SCRIPT_DIR/rviz2_default.rviz"
RVIZ_CFG_CT="/tmp/rviz2_default.rviz"

if ! docker ps --filter name="$SIM_CT" -q | grep -q .; then
    echo "[start_rviz] sim container '$SIM_CT' not running; bring it up first via sim/start.sh"
    exit 1
fi

docker cp "$RVIZ_CFG_HOST" "$SIM_CT":"$RVIZ_CFG_CT" >/dev/null
docker cp "$SCRIPT_DIR/goal_pose_relay.py" "$SIM_CT":/tmp/goal_pose_relay.py >/dev/null

# goal_pose_relay: rviz "2D Goal Pose" (/rviz_goal_pose) -> navigate_to_pose,
# zero-stamped, because rviz's goal tool stamps wall time even under
# use_sim_time and the planner can't transform that against the sim-clock
# TF. Launched detached (docker exec -d survives this shell); idempotent.
docker exec "$SIM_CT" bash -lc 'pkill -f goal_pose_relay.py 2>/dev/null; true' >/dev/null 2>&1 || true
sleep 0.3
docker exec -d "$SIM_CT" bash -lc "source /opt/ros/humble/setup.bash; export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}; exec python3 /tmp/goal_pose_relay.py --ros-args -p use_sim_time:=true -p input_topic:=/rviz_goal_pose > /tmp/goal_pose_relay.log 2>&1"

# Match the rest of the stack: use the same ROS 2 RMW as the sim,
# mapping, scene, and navigation containers.
docker exec -i \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e XAUTHORITY=/root/.Xauthority \
    -e QT_X11_NO_MITSHM=1 \
    "$SIM_CT" bash -lc "
        set -eo pipefail
        source /opt/ros/humble/setup.bash
        export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}
        exec ros2 run rviz2 rviz2 -d $RVIZ_CFG_CT --ros-args -p use_sim_time:=true
    "
