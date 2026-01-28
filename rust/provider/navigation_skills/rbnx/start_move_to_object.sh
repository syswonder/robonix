#!/bin/bash
# SPDX-License-Identifier: MulanPSL-2.0
set -e
[ -z "$ROBONIX_SDK_PATH" ] && [ -f ~/.robonix/config.yaml ] && \
  ROBONIX_SDK_PATH=$(grep 'robonix_sdk_path' ~/.robonix/config.yaml 2>/dev/null | sed 's/.*:[[:space:]]*//;s/[[:space:]]*$//')
[ -n "$ROBONIX_SDK_PATH" ] && [ -f "$ROBONIX_SDK_PATH/install/setup.bash" ] && source "$ROBONIX_SDK_PATH/install/setup.bash"
[ -f /opt/ros/humble/setup.bash ] && source /opt/ros/humble/setup.bash
cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)" && [ -f install/setup.bash ] && source install/setup.bash
export PYTHONPATH="${PWD}:${PYTHONPATH}"
exec ros2 run navigation_skills_provider move_to_object_skill 2>/dev/null || exec python3 -m navigation_skills_provider.move_to_object_skill
