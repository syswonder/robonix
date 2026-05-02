#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Scene container entrypoint. Sources ROS Humble, sets PYTHONPATH so
# the codegen output under /scene/rbnx-build/codegen/ is importable,
# then execs scene_service.service.
#
# `cd /scene` is critical: scene_service needs the package root on
# sys.path so `import scene_service.service` resolves. Bind-mounted
# from the host by the manifest's start: block — host edits show up
# instantly, no rebuild.

# `set -u` is incompatible with /opt/ros/humble/setup.bash (it
# references unset AMENT_TRACE_SETUP_FILES). Stick with -eo pipefail.
set -eo pipefail

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

cd /scene

# Codegen output lives under rbnx-build/codegen/. Build phase produces
# it on the host before this container runs; we just inject onto path.
export PYTHONPATH="/scene/rbnx-build/codegen/proto_gen:/scene/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"

# robonix-py lives in the workspace pylib dir, also bind-mounted.
if [ -d /robonix-py ]; then
    export PYTHONPATH="/robonix-py:${PYTHONPATH}"
fi

mkdir -p /scene/rbnx-build/data

exec python3 -m scene_service.service
