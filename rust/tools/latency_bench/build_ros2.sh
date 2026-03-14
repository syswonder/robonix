#!/usr/bin/env bash
# Build ROS2 latency_bench_msgs package for ROS2 transport benchmark.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

ROS_DISTRO="${ROS_DISTRO:-humble}"
if [[ -f "/opt/ros/$ROS_DISTRO/setup.bash" ]]; then
  set +u
  source "/opt/ros/$ROS_DISTRO/setup.bash"
  set -u
fi

WS="$SCRIPT_DIR/ros2_ws"
mkdir -p "$WS/src"
ln -sfn "$SCRIPT_DIR/latency_bench_msgs" "$WS/src/latency_bench_msgs" 2>/dev/null || true

cd "$WS"
colcon build --packages-select latency_bench_msgs --symlink-install

echo ""
echo "Build complete. Source with:"
echo "  source $WS/install/setup.bash"
