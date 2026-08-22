#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Run one immutable Scene review trial and retain protocol/resource evidence.

set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
OUTPUT_DIR="${1:?usage: $0 OUTPUT_DIR [WORLD] [DURATION_S]}"
WORLD="${2:-office}"
DURATION_S="${3:-120}"
RBNX_BIN="${ROBONIX_SCENE_BENCHMARK_RBNX:-$(command -v rbnx)}"
RBNX_ON_PATH="$(command -v rbnx || true)"
ROBONIX_SOURCE_PATH="$("$RBNX_BIN" path root)"
SCENE_IMAGE="${ROBONIX_SCENE_IMAGE:-robonix-scene}"
MOTION_MODE="${ROBONIX_SCENE_BENCHMARK_MOTION_MODE:-reactive}"
if [[ -n "${ROBONIX_SCENE_BENCHMARK_MANIFEST:-}" ]]; then
    BENCHMARK_MANIFEST="$ROBONIX_SCENE_BENCHMARK_MANIFEST"
    if [[ "$BENCHMARK_MANIFEST" != /* ]]; then
        BENCHMARK_MANIFEST="$REPO_ROOT/examples/webots/$BENCHMARK_MANIFEST"
    fi
elif [[ "$MOTION_MODE" == "explore" ]]; then
    BENCHMARK_MANIFEST="$REPO_ROOT/examples/webots/robonix_manifest.mapping-nav-eval.yaml"
else
    BENCHMARK_MANIFEST="$REPO_ROOT/examples/webots/robonix_manifest.scene-eval.yaml"
fi

if [[ -e "$OUTPUT_DIR" ]]; then
    echo "refusing to overwrite trial output: $OUTPUT_DIR" >&2
    exit 2
fi
mkdir -p "$OUTPUT_DIR"

date +%s.%N >"$OUTPUT_DIR/wrapper-start.txt"
docker image inspect "$SCENE_IMAGE" >"$OUTPUT_DIR/scene-image.json"
sha256sum "$RBNX_BIN" >"$OUTPUT_DIR/rbnx-sha256.txt"
sha256sum \
    "$REPO_ROOT/testing/fixtures/webots_scene_benchmark.json" \
    "$REPO_ROOT/testing/run_webots_scene_benchmark.sh" \
    "$REPO_ROOT/testing/run_webots_scene_sweep.py" \
    "$REPO_ROOT/testing/webots_scene_visibility.py" \
    "$REPO_ROOT/testing/webots_scene_motion.py" \
    "$REPO_ROOT/testing/scene_quality_ground_truth.py" \
    "$REPO_ROOT/testing/run_scene_review_trial.sh" \
    >"$OUTPUT_DIR/protocol-sha256.txt"
sha256sum "$BENCHMARK_MANIFEST" >"$OUTPUT_DIR/deployment-manifest-sha256.txt"
implementation_files=(
    system/scene/package_manifest.yaml
    system/scene/scene_service/ingest/cg_kernels.py
    system/scene/scene_service/ingest/model_storage.py
    system/scene/scene_service/ingest/perception_profiles.py
    system/scene/scene_service/ingest/tensorrt_cache.py
    system/scene/scene_service/ingest/perception_concept_graphs.py
    system/scene/scene_service/ingest/ros_subscribers.py
    system/scene/scene_service/service.py
    system/scene/scene_service/state/object_registry.py
    system/scene/rbnx-build/codegen/robonix_mcp_types/semantic_map_mcp.py
)
: >"$OUTPUT_DIR/implementation-sha256.txt"
for relative in "${implementation_files[@]}"; do
    if [[ -f "$REPO_ROOT/$relative" ]]; then
        (
            cd "$REPO_ROOT"
            sha256sum "$relative"
        ) >>"$OUTPUT_DIR/implementation-sha256.txt"
    else
        printf 'MISSING  %s\n' "$relative" \
            >>"$OUTPUT_DIR/implementation-sha256.txt"
    fi
done
printf '%s\n' \
    "world=$WORLD" \
    "duration_s=$DURATION_S" \
    "scene_image=$SCENE_IMAGE" \
    "scene_profile=${SCENE_PERCEPTION_PROFILE:-}" \
    "inference_precision=${SCENE_INFERENCE_PRECISION:-}" \
    "tensorrt=${SCENE_TENSORRT:-}" \
    "motion_mode=$MOTION_MODE" \
    "config_gate_mode=${ROBONIX_SCENE_BENCHMARK_CONFIG_GATE_MODE:-candidate}" \
    "alignment_timeout_s=${ROBONIX_SCENE_ALIGNMENT_TIMEOUT_S:-90}" \
    "post_acquisition_settle_s=${ROBONIX_SCENE_POST_ACQUISITION_SETTLE_S:-20}" \
    "benchmark_manifest=$BENCHMARK_MANIFEST" \
    "robonix_home=${ROBONIX_HOME:-}" \
    "robonix_source_path=$ROBONIX_SOURCE_PATH" \
    "rbnx=$RBNX_BIN" \
    "rbnx_on_path=$RBNX_ON_PATH" \
    >"$OUTPUT_DIR/trial-env.txt"

printf '%s\n' \
    'epoch_s,index,gpu_util_percent,memory_used_mib,memory_total_mib,power_w' \
    >"$OUTPUT_DIR/resource-gpu.csv"
printf '%s\n' \
    'epoch_s,name,cpu_percent,memory_usage,net_io,block_io,pids' \
    >"$OUTPUT_DIR/resource-docker.csv"

sample_resources() {
    while [[ ! -e "$OUTPUT_DIR/.sampling-done" ]]; do
        local now name
        now="$(date +%s.%N)"
        if command -v nvidia-smi >/dev/null 2>&1; then
            nvidia-smi \
                --query-gpu=index,utilization.gpu,memory.used,memory.total,power.draw \
                --format=csv,noheader,nounits 2>/dev/null \
                | sed "s/^/${now},/" \
                >>"$OUTPUT_DIR/resource-gpu.csv" || true
        fi
        while IFS= read -r name; do
            [[ -n "$name" ]] || continue
            docker stats --no-stream \
                --format "${now},{{.Name}},{{.CPUPerc}},{{.MemUsage}},{{.NetIO}},{{.BlockIO}},{{.PIDs}}" \
                "$name" 2>/dev/null \
                >>"$OUTPUT_DIR/resource-docker.csv" || true
        done < <(
            docker ps --format '{{.Names}}' 2>/dev/null \
                | grep -E '^(robonix_scene|robonix_mapping|robonix_tiago_sim_scene177)$' \
                || true
        )
        sleep 2
    done
}

sample_resources &
sampler_pid=$!
stop_sampler() {
    touch "$OUTPUT_DIR/.sampling-done"
    if kill -0 "$sampler_pid" 2>/dev/null; then
        wait "$sampler_pid" || true
    fi
}
trap stop_sampler EXIT INT TERM

set +e
ROBONIX_SCENE_BENCHMARK_RESULTS="$OUTPUT_DIR" \
ROBONIX_SCENE_BENCHMARK_DURATION_S="$DURATION_S" \
ROBONIX_SCENE_BENCHMARK_RBNX="$RBNX_BIN" \
    bash "$REPO_ROOT/testing/run_webots_scene_benchmark.sh" "$WORLD" \
    >"$OUTPUT_DIR/benchmark.log" 2>&1
status=$?
set -e

mapping_cache="$REPO_ROOT/examples/webots/rbnx-boot/cache/service-map-rbnx"
{
    printf 'mapping_source_path=%s\n' "$mapping_cache"
    if [[ -d "$mapping_cache/.git" ]]; then
        printf 'mapping_git_head=%s\n' "$(git -C "$mapping_cache" rev-parse HEAD)"
        printf 'mapping_git_dirty_files=%s\n' \
            "$(git -C "$mapping_cache" status --porcelain --untracked-files=no | wc -l | tr -d ' ')"
    else
        printf 'mapping_git_head=UNAVAILABLE\n'
        printf 'mapping_git_dirty_files=UNAVAILABLE\n'
    fi
    for relative in \
        package_manifest.yaml \
        scripts/start.sh \
        rbnx-build/proto-staging/robonix_contracts.proto \
        rbnx-build/codegen/mapping_proto_gen/robonix_contracts_pb2_grpc.py; do
        if [[ -f "$mapping_cache/$relative" ]]; then
            sha256sum "$mapping_cache/$relative"
        else
            printf 'MISSING  %s\n' "$mapping_cache/$relative"
        fi
    done
} >"$OUTPUT_DIR/dependency-evidence.txt"

stop_sampler
trap - EXIT INT TERM
date +%s.%N >"$OUTPUT_DIR/wrapper-end.txt"
printf '%s\n' "$status" >"$OUTPUT_DIR/wrapper-exit-code.txt"
cat "$OUTPUT_DIR/benchmark.log"
exit "$status"
