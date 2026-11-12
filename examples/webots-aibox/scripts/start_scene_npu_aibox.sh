#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Start native Scene with the AIBOX DualMap RKNPU3 model bundle.
set -euo pipefail

SCENE=/root/robonix/system/scene
export SCENE_DUALMAP_NPU_DIR=/root/robonix/dualmap/model/npu
export SCENE_DUALMAP_CLIP_WEIGHTS=/root/weights/clip/ViT-B-32.pt
export SCENE_DUALMAP_ROOT=/opt/dualmap
export SCENE_DUALMAP_CLASSES=/root/robonix/dualmap/config/class_list/scene_eval_classes.txt
export ROBONIX_SCENE_FORCE=native

exec "$SCENE/scripts/start.sh" "$@"
