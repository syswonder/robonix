#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start semantic map service. Optional: pass config name to select map config.
# Usage: ./start_semantic_map.sh [config]
#   config: building (default), webots, or a filename e.g. my_map_config.yaml
set -e
CONFIG="${HOME:-/tmp}/.robonix/config.yaml"
[ -z "$ROBONIX_SDK_PATH" ] && [ -f "$CONFIG" ] && \
  ROBONIX_SDK_PATH=$(grep 'robonix_sdk_path' "$CONFIG" 2>/dev/null | sed 's/.*:[[:space:]]*//;s/[[:space:]]*$//' | tr -d "\"'")
[ -n "$ROBONIX_SDK_PATH" ] && { [ -f "$ROBONIX_SDK_PATH/install/setup.bash" ] && source "$ROBONIX_SDK_PATH/install/setup.bash"; _d=$(find "$ROBONIX_SDK_PATH/install" -type d -path "*/lib/python*/site-packages" 2>/dev/null | head -1); [ -n "$_d" ] && export PYTHONPATH="${_d}:${PYTHONPATH}"; }
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" && export PYTHONPATH="${PWD}:${PYTHONPATH}"
CONFIG_ARG=""
[ -n "$1" ] && CONFIG_ARG="--config $1"
exec python3 -m demo_service_provider.semantic_map_service $CONFIG_ARG
