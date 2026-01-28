#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
set -e
_log() { echo "[rbnx] $*" >&2; }
_log "start_wandering.sh: starting skl::wandering"

[ -z "$ROBONIX_SDK_PATH" ] && [ -f "${HOME:-/tmp}/.robonix/config.yaml" ] && \
  ROBONIX_SDK_PATH=$(grep 'robonix_sdk_path' "${HOME:-/tmp}/.robonix/config.yaml" 2>/dev/null | sed 's/.*:[[:space:]]*//;s/[[:space:]]*$//' | tr -d "\"'")
if [ -n "$ROBONIX_SDK_PATH" ]; then
  _log "ROBONIX_SDK_PATH=${ROBONIX_SDK_PATH}"
  [ -f "$ROBONIX_SDK_PATH/install/setup.bash" ] && source "$ROBONIX_SDK_PATH/install/setup.bash" && _log "sourced SDK setup.bash" || _log "SDK setup.bash not found, skip"
else
  _log "ROBONIX_SDK_PATH not set (no config or empty)"
fi
if [ -f /opt/ros/humble/setup.bash ]; then
  source /opt/ros/humble/setup.bash && _log "sourced ROS2 humble"
else
  _log "ROS2 /opt/ros/humble/setup.bash not found"
fi
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_log "PWD=${PWD}"

# Prefer source layout when skill module exists (avoids ros2 run when no colcon install)
if [ -f navigation_skills_provider/wandering_skill.py ]; then
  _log "layout=source, using python3 -m navigation_skills_provider.wandering_skill"
  export PYTHONPATH="${PWD}:${PYTHONPATH}"
  if [ -n "$ROBONIX_SDK_PATH" ]; then
    SDK_PY=$(find "$ROBONIX_SDK_PATH/install" -type d -path "*/dist-packages" 2>/dev/null | head -1)
    if [ -n "$SDK_PY" ]; then
      export PYTHONPATH="${SDK_PY}:${PYTHONPATH}"
      _log "PYTHONPATH includes SDK dist-packages: ${SDK_PY}"
    else
      _log "SDK dist-packages not found under ROBONIX_SDK_PATH/install, robonix_sdk may fail to import"
    fi
  fi
  _log "exec: python3 -m navigation_skills_provider.wandering_skill"
  exec python3 -m navigation_skills_provider.wandering_skill
fi
if [ -f install/setup.bash ]; then
  _log "layout=install, using ros2 run navigation_skills_provider wandering_skill"
  source install/setup.bash && exec ros2 run navigation_skills_provider wandering_skill
fi
_log "error: no navigation_skills_provider/wandering_skill.py and no install/setup.bash"
exit 1
