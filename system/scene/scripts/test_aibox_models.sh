#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Push converted DualMap graphs to the AIBOX and smoke-test every one of them
# on the board with test_aibox_rknn_lite.py (one process per model).
#
# Usage: test_aibox_models.sh [DIR ...]
#
# Scans each DIR (default: the conversion output dirs under
# $AIBOX_NPU_WORKDIR) for <model>.rknn, pairs it with <model>.weight and the
# first reference found among <model>_sim.npz (toolkit simulator) and
# <model>_torch.npz (export reference). Graph input sizes: yoloworld 640, mobilesam 1024,
# clip_image 224. mobilesam_decoder has no image input — it replays the
# reference npz's in0.. arrays (--inputs) and is skipped without one.
#
# PASS means the graph loads, initializes and runs on the NPU. The reported
# cosine is only meaningful when IMAGE matches the one the reference was
# generated with (the build/fetch --image); point AIBOX_TEST_IMAGE at it for
# a numeric check, any other image still exercises the graph as a smoke test.
set -euo pipefail

WORK="${AIBOX_NPU_WORKDIR:-$HOME/aibox-npu}"
HOST="${AIBOX_SSH_TARGET:-aibox}"
IMAGE="${AIBOX_TEST_IMAGE:-$WORK/rknn-model-zoo/examples/yolo_world/model/bus.jpg}"
BOARD_DIR=/tmp/aibox-model-test
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

die() {
    printf '[aibox-test] error: %s\n' "$*" >&2
    exit 1
}

info() {
    printf '[aibox-test] %s\n' "$*"
}

model_size() {
    case "$1" in
        mobilesam) printf 1024 ;;
        clip_image) printf 224 ;;
        *) printf 640 ;;
    esac
}

# First reference npz for the graph, if any; the board run then reports the
# cosine against it instead of bare output statistics.
model_ref() {
    local dir="$1" name="$2" cand
    for cand in "$dir/$name"_sim.npz "$dir/$name"_torch.npz "$WORK/models/zoo/$name"_torch.npz; do
        if [[ -f "$cand" ]]; then
            printf '%s' "$cand"
            return
        fi
    done
}

dirs=("$@")
if ((${#dirs[@]} == 0)); then
    for d in "$WORK/rknn-clean" "$WORK/rknn-zoo"; do
        if [[ -d "$d" ]]; then
            dirs+=("$d")
        fi
    done
fi
((${#dirs[@]})) || die "no model directories found (pass DIR ... or populate $WORK)"
[[ -f "$IMAGE" ]] || die "test image not found: $IMAGE"
[[ -f "$HERE/test_aibox_rknn_lite.py" ]] || die 'test_aibox_rknn_lite.py not found next to this script'

files=("$HERE/test_aibox_rknn_lite.py" "$IMAGE")
names=()
runs=()
shopt -s nullglob
for dir in "${dirs[@]}"; do
    for rknn in "$dir"/*.rknn; do
        name="$(basename "$rknn" .rknn)"
        weight="${rknn%.rknn}.weight"
        if [[ ! -f "$weight" ]]; then
            info "skip $name: no weight file $weight"
            continue
        fi
        ref="$(model_ref "$dir" "$name" || true)"
        if [[ "$name" == mobilesam_decoder && -z "$ref" ]]; then
            info "skip $name: no reference npz with replayable inputs"
            continue
        fi
        ref_arg=""
        if [[ -n "$ref" ]]; then
            files+=("$ref")
            ref_arg="--ref $(basename "$ref")"
        else
            info "$name: no reference npz found; stats-only run"
        fi
        files+=("$rknn" "$weight")
        names+=("$name")
        if [[ "$name" == mobilesam_decoder ]]; then
            runs+=("python3 test_aibox_rknn_lite.py --rknn $name.rknn --weight $name.weight --inputs $(basename "$ref") $ref_arg")
        else
            runs+=("python3 test_aibox_rknn_lite.py --rknn $name.rknn --weight $name.weight --image $(basename "$IMAGE") --size $(model_size "$name") $ref_arg")
        fi
    done
done
shopt -u nullglob
((${#runs[@]})) || die "no models found in: ${dirs[*]}"

info "deploying ${#runs[@]} model(s) to $HOST:$BOARD_DIR"
info "cosine is meaningful only when the image matches the reference-generation image; pass AIBOX_TEST_IMAGE=<that image> for a numeric check"
ssh -o BatchMode=yes "$HOST" "mkdir -p '$BOARD_DIR'"
scp -q "${files[@]}" "$HOST:$BOARD_DIR/"

status=0
for i in "${!runs[@]}"; do
    info "testing ${names[$i]}: ${runs[$i]}"
    if ssh -o BatchMode=yes "$HOST" "cd '$BOARD_DIR' && ${runs[$i]}"; then
        info "PASS ${names[$i]}"
    else
        info "FAIL ${names[$i]}"
        status=1
    fi
done
((status == 0)) || die 'some models failed on the board'
info "all ${#runs[@]} model(s) passed"
