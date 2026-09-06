#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Run Scene's WBT-derived benchmark in fresh, isolated Webots sessions.
#
# This is intentionally serial: the Scene and Mapping packages use fixed local
# ports/container names.  Each world receives a newly created simulator
# container and a new Robonix boot, so odometry, map, and object state cannot
# leak between scores.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
WEBOTS_DIR="$REPO_ROOT/examples/webots"
MOTION_MODE="${ROBONIX_SCENE_BENCHMARK_MOTION_MODE:-reactive}"
CONFIG_GATE_MODE="${ROBONIX_SCENE_BENCHMARK_CONFIG_GATE_MODE:-candidate}"
ALIGNMENT_TIMEOUT_S="${ROBONIX_SCENE_ALIGNMENT_TIMEOUT_S:-90}"
POST_ACQUISITION_SETTLE_S="${ROBONIX_SCENE_POST_ACQUISITION_SETTLE_S:-20}"
if [[ "$MOTION_MODE" == "explore" ]]; then
    DEFAULT_MANIFEST="$WEBOTS_DIR/robonix_manifest.mapping-nav-eval.yaml"
else
    DEFAULT_MANIFEST="$WEBOTS_DIR/robonix_manifest.scene-eval.yaml"
fi
MANIFEST="${ROBONIX_SCENE_BENCHMARK_MANIFEST:-$DEFAULT_MANIFEST}"
if [[ "$MANIFEST" != /* ]]; then
    MANIFEST="$WEBOTS_DIR/$MANIFEST"
fi
EXPLORE_HELPER="$REPO_ROOT/testing/run_webots_explore.py"
EXPLORE_BUDGET_HELPER="$REPO_ROOT/testing/estimate_webots_explore_budget.py"
BENCHMARK_FILE="$REPO_ROOT/testing/fixtures/webots_scene_benchmark.json"
SIM_CONTAINER="${ROBONIX_SCENE_BENCHMARK_SIM_CONTAINER:-robonix_tiago_sim_scene177}"
RESULT_ROOT="${ROBONIX_SCENE_BENCHMARK_RESULTS:-/tmp/scene177-five-world-$(date +%Y%m%dT%H%M%S)}"
ROBONIX_HOME="${ROBONIX_HOME:-/tmp/robonix-scene-eval-home-3033b25}"
STREAM_PORT="${ROBONIX_SCENE_BENCHMARK_STREAM_PORT:-1245}"
VIEWER_PORT="${ROBONIX_SCENE_BENCHMARK_VIEWER_PORT:-8090}"
STREAM_ENABLED="${ROBONIX_SCENE_BENCHMARK_STREAM:-0}"
SCENE_URL="${ROBONIX_SCENE_BENCHMARK_SCENE_URL:-http://127.0.0.1:50107}"
LABEL_JUDGMENTS_FILE="${ROBONIX_SCENE_LABEL_JUDGMENTS_FILE:-}"
if [[ -n "$LABEL_JUDGMENTS_FILE" && "$LABEL_JUDGMENTS_FILE" != /* ]]; then
    LABEL_JUDGMENTS_FILE="$REPO_ROOT/$LABEL_JUDGMENTS_FILE"
fi
RBNX_BIN="${ROBONIX_SCENE_BENCHMARK_RBNX:-rbnx}"
RBNX_BIN="$(command -v "$RBNX_BIN")"
RBNX_BIN_DIR="$(dirname "$RBNX_BIN")"

ALL_WORLDS=(office break_room kitchen apartment complete_apartment)

world_duration_s() {
    case "$1" in
        office) echo 72 ;;
        break_room | kitchen) echo 96 ;;
        apartment) echo 150 ;;
        complete_apartment) echo 240 ;;
        *) return 1 ;;
    esac
}

benchmark_world_path() {
    python3 - "$BENCHMARK_FILE" "$1" "$REPO_ROOT" <<'PY'
import json
import pathlib
import sys

fixture = json.loads(pathlib.Path(sys.argv[1]).read_text(encoding="utf-8"))
world_id = sys.argv[2]
root = pathlib.Path(sys.argv[3])
for entry in fixture.get("worlds") or []:
    if entry.get("id") == world_id:
        path = root / str(entry["world"])
        if not path.is_file():
            raise SystemExit(f"benchmark world does not exist: {path}")
        print(path)
        break
else:
    raise SystemExit(f"benchmark world is not declared: {world_id}")
PY
}

usage() {
    cat <<'EOF'
Usage: testing/run_webots_scene_benchmark.sh [world ...]

Worlds: office break_room kitchen apartment complete_apartment
With no arguments, all five worlds are run serially.

Environment:
  ROBONIX_HOME
  ROBONIX_SCENE_BENCHMARK_RESULTS
  ROBONIX_SCENE_BENCHMARK_SIM_CONTAINER
  ROBONIX_SCENE_BENCHMARK_STREAM (default: 0)
  ROBONIX_SCENE_BENCHMARK_STREAM_PORT
  ROBONIX_SCENE_BENCHMARK_VIEWER_PORT
  ROBONIX_SCENE_BENCHMARK_RBNX (default: rbnx from PATH)
  ROBONIX_SCENE_BENCHMARK_MANIFEST (default: scene-only evaluation)
  ROBONIX_SCENE_BENCHMARK_MOTION_MODE (reactive | panorama | explore)
  ROBONIX_SCENE_BENCHMARK_CONFIG_GATE_MODE (candidate | baseline)
  ROBONIX_SCENE_ALIGNMENT_TIMEOUT_S (positive integer; default: 90)
  ROBONIX_SCENE_POST_ACQUISITION_SETTLE_S (positive integer; default: 20)
  ROBONIX_SCENE_BENCHMARK_EXPLORE_SPEED_MPS (default: 0.12)
  ROBONIX_SCENE_BENCHMARK_DURATION_S (fixed override; otherwise WBT-derived)
  ROBONIX_SCENE_BENCHMARK_MIN_DURATION_S (explore default: 180)
  ROBONIX_SCENE_BENCHMARK_MAX_DURATION_S (explore default: 600)
  ROBONIX_SCENE_LABEL_JUDGMENTS_FILE (recorded Codex/GPT semantic review)
  ROBONIX_SCENE_MIN_VISIBLE_RATIO (default: 0.25; diagnostic runs only)
  ROBONIX_SCENE_MAX_LOCALIZED_P95_ERROR_M (default: 0.30)
  ROBONIX_SCENE_MAX_LOCALIZED_FINAL_ERROR_M (default: 0.45)
  ROBONIX_SCENE_MAX_LOCALIZED_YAW_P95_RAD (default: 0.20)
EOF
}

if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
    usage
    exit 0
fi

WORLDS=("$@")
if [[ ${#WORLDS[@]} -eq 0 ]]; then
    WORLDS=("${ALL_WORLDS[@]}")
fi
for world in "${WORLDS[@]}"; do
    if ! world_duration_s "$world" >/dev/null; then
        echo "unknown benchmark world: $world" >&2
        usage >&2
        exit 2
    fi
done
if [[ ! -f "$MANIFEST" ]]; then
    echo "missing Scene benchmark manifest: $MANIFEST" >&2
    exit 1
fi
if [[ "$MOTION_MODE" != "reactive" && "$MOTION_MODE" != "panorama" \
    && "$MOTION_MODE" != "explore" ]]; then
    echo "invalid Scene benchmark motion mode: $MOTION_MODE" >&2
    exit 2
fi
if [[ "$CONFIG_GATE_MODE" != "candidate" \
    && "$CONFIG_GATE_MODE" != "baseline" ]]; then
    echo "invalid Scene benchmark config gate mode: $CONFIG_GATE_MODE" >&2
    exit 2
fi
if [[ ! "$ALIGNMENT_TIMEOUT_S" =~ ^[1-9][0-9]*$ ]]; then
    echo "ROBONIX_SCENE_ALIGNMENT_TIMEOUT_S must be a positive integer: $ALIGNMENT_TIMEOUT_S" >&2
    exit 2
fi
if [[ ! "$POST_ACQUISITION_SETTLE_S" =~ ^[1-9][0-9]*$ ]]; then
    echo "ROBONIX_SCENE_POST_ACQUISITION_SETTLE_S must be a positive integer: $POST_ACQUISITION_SETTLE_S" >&2
    exit 2
fi
if [[ "$MOTION_MODE" == "explore" && ! -f "$EXPLORE_HELPER" ]]; then
    echo "missing Explore benchmark helper: $EXPLORE_HELPER" >&2
    exit 1
fi
if [[ "$MOTION_MODE" == "explore" && ! -f "$EXPLORE_BUDGET_HELPER" ]]; then
    echo "missing Explore budget helper: $EXPLORE_BUDGET_HELPER" >&2
    exit 1
fi
if [[ -n "$LABEL_JUDGMENTS_FILE" && ! -f "$LABEL_JUDGMENTS_FILE" ]]; then
    echo "missing Scene label judgments: $LABEL_JUDGMENTS_FILE" >&2
    exit 1
fi

mkdir -p "$RESULT_ROOT"
printf '%s\n' "$RESULT_ROOT" >"$RESULT_ROOT/result-root.txt"

# Seed the isolated ROBONIX_HOME. Every package spawned under it reads
# `robonix_source_path` to resolve capability and IDL paths, and refuses to
# start without it. The default home lives in /tmp, so a reboot -- or a first
# run on a fresh machine -- leaves it absent or half-written by some earlier
# rbnx invocation. That surfaced only as three primitives exiting "before
# registering with atlas", several layers below the real cause.
mkdir -p "$ROBONIX_HOME/packages"
python3 - "$ROBONIX_HOME/config.yaml" "$ROBONIX_HOME/packages" "$REPO_ROOT" <<'SEED_PY'
import pathlib
import sys

import yaml

config_path = pathlib.Path(sys.argv[1])
packages, source = sys.argv[2], sys.argv[3]
config = {}
if config_path.is_file():
    loaded = yaml.safe_load(config_path.read_text(encoding='utf-8'))
    if isinstance(loaded, dict):
        config = loaded
# Rewrite both keys: an existing home may point at a different checkout, and
# scoring the tree we are running from is the whole point of the benchmark.
config['package_storage_path'] = packages
config['robonix_source_path'] = source
config_path.write_text(yaml.safe_dump(config, sort_keys=True), encoding='utf-8')
SEED_PY

boot_pid=""
sim_pid=""
sweep_pid=""
active_project=""

stop_one() {
    local status=0
    if [[ -n "$sweep_pid" ]] && kill -0 "$sweep_pid" 2>/dev/null; then
        kill -TERM "$sweep_pid" 2>/dev/null || true
        wait "$sweep_pid" 2>/dev/null || true
    fi
    sweep_pid=""
    if [[ -n "$boot_pid" ]] && kill -0 "$boot_pid" 2>/dev/null; then
        kill -INT "$boot_pid" 2>/dev/null || true
        for _ in $(seq 1 30); do
            kill -0 "$boot_pid" 2>/dev/null || break
            sleep 1
        done
        if kill -0 "$boot_pid" 2>/dev/null; then
            kill -TERM "$boot_pid" 2>/dev/null || true
            status=1
        fi
    fi
    boot_pid=""
    if [[ -n "$active_project" ]]; then
        (
            cd "$WEBOTS_DIR"
            ROBONIX_SIM_PROJECT="$active_project" \
            ROBONIX_SIM_CONTAINER="$SIM_CONTAINER" \
                bash sim/stop.sh
        ) || status=1
    fi
    active_project=""
    if [[ -n "$sim_pid" ]] && kill -0 "$sim_pid" 2>/dev/null; then
        kill -TERM "$sim_pid" 2>/dev/null || true
    fi
    sim_pid=""
    return "$status"
}
# Capture before tearing down. Component logs used to be copied only
# after a world scored, so every run that failed -- exactly the ones
# whose logs explain the failure -- discarded them when the trap
# stopped the deployment.
trap 'capture_component_logs "${world_dir:-}" || true; stop_one || true' EXIT INT TERM

wait_for_sim() {
    local log_file="$1"
    for _ in $(seq 1 180); do
        if [[ -n "$sim_pid" ]] && ! kill -0 "$sim_pid" 2>/dev/null; then
            echo "simulator launcher exited before readiness" >&2
            tail -120 "$log_file" >&2 || true
            return 1
        fi
        if [[ "$(docker inspect -f '{{.State.Running}}' "$SIM_CONTAINER" 2>/dev/null || true)" == "true" ]]; then
            local topic_count
            topic_count="$(
                docker exec "$SIM_CONTAINER" bash -lc \
                    'source /opt/ros/humble/setup.bash && ros2 topic list 2>/dev/null | wc -l' \
                    2>/dev/null || echo 0
            )"
            if [[ "${topic_count:-0}" -gt 20 ]]; then
                return 0
            fi
        fi
        sleep 1
    done
    echo "simulator did not expose ROS topics within 180 seconds" >&2
    tail -120 "$log_file" >&2 || true
    return 1
}

wait_for_scene() {
    local log_file="$1"
    for _ in $(seq 1 240); do
        if [[ -n "$boot_pid" ]] && ! kill -0 "$boot_pid" 2>/dev/null; then
            echo "rbnx boot exited before Scene readiness" >&2
            tail -160 "$log_file" >&2 || true
            return 1
        fi
        if curl -fsS --max-time 2 "$SCENE_URL/api/state" >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    echo "Scene API did not become ready within 240 seconds" >&2
    tail -160 "$log_file" >&2 || true
    return 1
}

wait_for_explore() {
    local log_file="$1"
    for _ in $(seq 1 240); do
        if [[ -n "$boot_pid" ]] && ! kill -0 "$boot_pid" 2>/dev/null; then
            echo "rbnx boot exited before Explore registration" >&2
            tail -160 "$log_file" >&2 || true
            return 1
        fi
        if python3 "$EXPLORE_HELPER" ready \
            --rbnx "$RBNX_BIN" \
            --deploy-dir "$WEBOTS_DIR" \
            >/dev/null 2>&1; then
            return 0
        fi
        sleep 1
    done
    echo "Explore did not register within 240 seconds" >&2
    tail -160 "$log_file" >&2 || true
    return 1
}

capture_component_logs() {
    # Called both on the success path and from the teardown trap, so it has to
    # tolerate being invoked before the first world has a directory.
    local output_dir="${1:-}"
    [[ -n "$output_dir" && -d "$output_dir" ]] || return 0
    local component
    for component in explore nav2 mapping scene; do
        local source_log="$WEBOTS_DIR/rbnx-boot/logs/${component}.log"
        if [[ -f "$source_log" ]]; then
            cp "$source_log" "$output_dir/${component}.log"
        fi
    done
}

for world in "${WORLDS[@]}"; do
    world_dir="$RESULT_ROOT/$world"
    mkdir -p "$world_dir"
    active_project="robonix_scene177_${world}"
    python3 "$REPO_ROOT/testing/export_webots_scene_truth.py" \
        --world-id "$world" \
        --output "$world_dir/truth.json"
    if [[ -n "${ROBONIX_SCENE_BENCHMARK_DURATION_S:-}" ]]; then
        duration="$ROBONIX_SCENE_BENCHMARK_DURATION_S"
        duration_source="explicit override"
    elif [[ "$MOTION_MODE" == "explore" ]]; then
        duration="$(
            python3 "$EXPLORE_BUDGET_HELPER" \
                --truth-file "$world_dir/truth.json" \
                --max-speed-m-s \
                    "${ROBONIX_SCENE_BENCHMARK_EXPLORE_SPEED_MPS:-0.12}" \
                --minimum-s \
                    "${ROBONIX_SCENE_BENCHMARK_MIN_DURATION_S:-180}" \
                --maximum-s \
                    "${ROBONIX_SCENE_BENCHMARK_MAX_DURATION_S:-600}"
        )"
        duration_source="WBT-derived hard limit"
    elif [[ "$MOTION_MODE" == "panorama" ]]; then
        duration="$(world_duration_s "$world")"
        duration_source="fixed-position panorama"
    else
        duration="$(world_duration_s "$world")"
        duration_source="reactive route"
    fi
    echo "=== Scene benchmark: $world (${duration}s, ${duration_source}) ==="

    (
        cd "$WEBOTS_DIR"
        export ROBONIX_HOME
        export ROBONIX_SIM_PROJECT="$active_project"
        export ROBONIX_SIM_CONTAINER="$SIM_CONTAINER"
        export ROBONIX_SIM_STREAM="$STREAM_ENABLED"
        export ROBONIX_SIM_STREAM_PORT="$STREAM_PORT"
        export ROBONIX_SIM_VIEWER_PORT="$VIEWER_PORT"
        export ROBONIX_GPU_ID="${ROBONIX_GPU_ID:-0}"
        export ROBONIX_WEBOTS_WORLD="${world}.wbt"
        export WEBOTS_HEADLESS_MODE="${WEBOTS_HEADLESS_MODE:-auto}"
        exec bash sim/start.sh
    ) >"$world_dir/sim.log" 2>&1 &
    sim_pid=$!
    wait_for_sim "$world_dir/sim.log"

    (
        cd "$WEBOTS_DIR"
        export ROBONIX_HOME
        # Each scored world must start with empty object memory and graph
        # caches. The Scene package default persists under its package build
        # tree, which would leak state between benchmark acquisitions.
        export SCENE_DATA_DIR="$world_dir/scene-data"
        export ROBONIX_SIM_PROJECT="$active_project"
        export ROBONIX_SIM_CONTAINER="$SIM_CONTAINER"
        export ROBONIX_SIM_STREAM="$STREAM_ENABLED"
        export ROBONIX_SIM_STREAM_PORT="$STREAM_PORT"
        export ROBONIX_SIM_VIEWER_PORT="$VIEWER_PORT"
        export ROBONIX_GPU_ID="${ROBONIX_GPU_ID:-0}"
        export ROBONIX_WEBOTS_WORLD="${world}.wbt"
        export WEBOTS_HEADLESS_MODE="${WEBOTS_HEADLESS_MODE:-auto}"
        # Package start scripts also call `rbnx path`; keep the exact tested
        # binary's directory first instead of falling back to a stale global
        # installation after this process launches.
        export PATH="$RBNX_BIN_DIR:$PATH"
        exec "$RBNX_BIN" boot -f "$MANIFEST" --no-update-check
    ) >"$world_dir/boot.log" 2>&1 &
    boot_pid=$!
    wait_for_scene "$world_dir/boot.log"
    curl -fsS --max-time 10 "$SCENE_URL/api/state" \
        >"$world_dir/startup-state.json"
    python3 - "$world_dir/startup-state.json" "$MANIFEST" \
        "$CONFIG_GATE_MODE" <<'PY'
import json
import math
import sys

import yaml

state = json.load(open(sys.argv[1], encoding="utf-8"))
manifest = yaml.safe_load(open(sys.argv[2], encoding="utf-8"))
gate_mode = sys.argv[3]
if gate_mode == "baseline":
    # Stable dev predates perception_quality and the shared Scene Driver
    # configuration channel.  It therefore cannot attest candidate-only
    # knobs.  Still require the public baseline state schema before acquiring
    # observations; readiness alone must not admit an unrelated HTTP server.
    required = {"objects", "robot", "map_binding"}
    missing = sorted(required - set(state))
    if missing:
        raise SystemExit(
            "Scene benchmark baseline state is incomplete: "
            + ", ".join(missing)
        )
    print(
        "Scene benchmark baseline config gate: candidate-only runtime "
        "diagnostics are not required",
        file=sys.stderr,
    )
    raise SystemExit(0)
quality = state.get("perception_quality") or {}
rerank = (
    manifest.get("system", {})
    .get("scene", {})
    .get("config", {})
    .get("perception", {})
    .get("label", {})
    .get("clip_rerank", {})
)
geometry = (
    manifest.get("system", {})
    .get("scene", {})
    .get("config", {})
    .get("perception", {})
    .get("geometry", {})
)
surface_snap = geometry.get("surface_snap", {})
groups = rerank.get("groups") or []
routes = rerank.get("routes") or {}
group_specs = [
    (
        list(group.get("labels") or ()),
        group.get("min_margin"),
    )
    if isinstance(group, dict)
    else (list(group or ()), None)
    for group in groups
]
route_specs = {
    str(source).strip().lower(): (
        list(route.get("labels") or ()),
        route.get("min_margin"),
    )
    if isinstance(route, dict)
    else (list(route or ()), None)
    for source, route in routes.items()
}
labels = {
    str(label).strip().lower()
    for group_labels, _ in group_specs
    for label in group_labels
}
labels.update(
    str(label).strip().lower()
    for route_labels, _ in route_specs.values()
    for label in route_labels
)
expected_group_count = len(groups)
expected_route_count = len(routes)
expected_label_count = len(labels)
expected_min_score = float(rerank.get("min_score", 0.0))
expected_min_margin = float(rerank.get("min_margin", 0.0))
expected_min_margin_by_label = {
    str(label).strip().lower(): float(group_margin)
    for group_labels, group_margin in group_specs
    if group_margin is not None
    for label in group_labels
}
expected_min_margin_by_label.update(
    {
        source: float(route_margin)
        for source, (_, route_margin) in route_specs.items()
        if route_margin is not None
    }
)
failures = []
if "rebase_map_corrections" in geometry:
    expected_rebase_map_corrections = bool(
        geometry["rebase_map_corrections"]
    )
    actual_rebase_map_corrections = bool(
        quality.get("rebase_map_corrections", False)
    )
    if actual_rebase_map_corrections != expected_rebase_map_corrections:
        failures.append(
            "rebase_map_corrections="
            f"{actual_rebase_map_corrections!r}, expected "
            f"{expected_rebase_map_corrections!r} from manifest"
        )
else:
    if "rebase_map_corrections" in quality:
        failures.append("runtime still exposes historical geometry rebase mode")
    transform_evidence_count = int(
        quality.get("observation_transform_evidence") or 0
    )
    last_transform_evidence = (
        quality.get("last_observation_transform_evidence") or {}
    )
    if int(quality.get("healthy_frames") or 0) > 0:
        if transform_evidence_count < 1:
            failures.append("observation transform evidence was not recorded")
        if not str(last_transform_evidence.get("source") or ""):
            failures.append("last observation transform has no source")
        if last_transform_evidence.get("historical_geometry_rebased") is not False:
            failures.append("historical geometry was not explicitly fixed")
if int(quality.get("clip_rerank_group_count") or 0) != expected_group_count:
    failures.append(
        "clip_rerank_group_count="
        f"{quality.get('clip_rerank_group_count')!r}, "
        f"expected {expected_group_count} from manifest"
    )
if int(quality.get("clip_rerank_route_count") or 0) != expected_route_count:
    failures.append(
        "clip_rerank_route_count="
        f"{quality.get('clip_rerank_route_count')!r}, "
        f"expected {expected_route_count} from manifest"
    )
if int(quality.get("clip_rerank_ready_label_count") or 0) != expected_label_count:
    failures.append(
        "clip_rerank_ready_label_count="
        f"{quality.get('clip_rerank_ready_label_count')!r}, "
        f"expected {expected_label_count} from manifest"
    )
if not math.isclose(
    float(quality.get("clip_rerank_min_score", -1.0)),
    expected_min_score,
    abs_tol=1e-9,
):
    failures.append(
        "clip_rerank_min_score="
        f"{quality.get('clip_rerank_min_score')!r}, "
        f"expected {expected_min_score} from manifest"
    )
if not math.isclose(
    float(quality.get("clip_rerank_min_margin", -1.0)),
    expected_min_margin,
    abs_tol=1e-9,
):
    failures.append(
        "clip_rerank_min_margin="
        f"{quality.get('clip_rerank_min_margin')!r}, "
        f"expected {expected_min_margin} from manifest"
    )
actual_min_margin_by_label = quality.get(
    "clip_rerank_min_margin_by_label"
) or {}
if actual_min_margin_by_label != expected_min_margin_by_label:
    failures.append(
        "clip_rerank_min_margin_by_label="
        f"{actual_min_margin_by_label!r}, "
        f"expected {expected_min_margin_by_label!r} from manifest"
    )
expected_surface_snap = {
    "labels": sorted(
        str(label).strip().lower()
        for label in surface_snap.get("labels") or ()
        if str(label).strip()
    ),
    "max_distance_m": float(surface_snap.get("max_distance_m", 0.60)),
    "tangent_padding_m": float(
        surface_snap.get("tangent_padding_m", 0.25)
    ),
    "min_shift_m": float(surface_snap.get("min_shift_m", 0.05)),
    "min_support_cells": int(surface_snap.get("min_support_cells", 30)),
    "min_dominant_share": float(
        surface_snap.get("min_dominant_share", 0.55)
    ),
    "min_tangent_coverage": float(
        surface_snap.get("min_tangent_coverage", 0.50)
    ),
    "occupancy_threshold": int(
        surface_snap.get("occupancy_threshold", 50)
    ),
}
actual_surface_snap = quality.get("surface_snap") or {}
for key, expected in expected_surface_snap.items():
    actual = actual_surface_snap.get(key)
    if isinstance(expected, float):
        matches = math.isclose(
            float(actual if actual is not None else -1.0),
            expected,
            abs_tol=1e-9,
        )
    else:
        matches = actual == expected
    if not matches:
        failures.append(
            f"surface_snap.{key}={actual!r}, expected {expected!r} "
            "from manifest"
        )
if failures:
    raise SystemExit(
        "Scene benchmark config was not applied: " + "; ".join(failures)
    )
PY

    docker cp "$REPO_ROOT/testing/run_webots_scene_sweep.py" \
        "$SIM_CONTAINER:/tmp/run_webots_scene_sweep.py"
    docker cp "$REPO_ROOT/testing/webots_scene_motion.py" \
        "$SIM_CONTAINER:/tmp/webots_scene_motion.py"
    docker cp "$REPO_ROOT/testing/webots_scene_visibility.py" \
        "$SIM_CONTAINER:/tmp/webots_scene_visibility.py"
    docker cp "$world_dir/truth.json" "$SIM_CONTAINER:/tmp/scene_truth.json"
    sweep_command="source /opt/ros/humble/setup.bash && python3 /tmp/run_webots_scene_sweep.py \
      --truth-file /tmp/scene_truth.json \
      --output /tmp/scene_visibility.json \
      --alignment-timeout-s $ALIGNMENT_TIMEOUT_S \
      --max-odom-step-m ${ROBONIX_SCENE_MAX_ODOM_STEP_M:-0.25} \
      --max-odom-speed-mps ${ROBONIX_SCENE_MAX_ODOM_SPEED_MPS:-0.60} \
      --duration-s $duration"
    if [[ "$MOTION_MODE" == "panorama" ]]; then
        sweep_command="$sweep_command --panorama"
    fi
    if [[ "$MOTION_MODE" == "explore" ]]; then
        wait_for_explore "$world_dir/boot.log"
        (
            timeout "$((duration + ALIGNMENT_TIMEOUT_S + 30))" \
                docker exec "$SIM_CONTAINER" bash -lc "$sweep_command --passive"
        ) >"$world_dir/sweep.log" 2>&1 &
        sweep_pid=$!
        for _ in $(seq 1 "$ALIGNMENT_TIMEOUT_S"); do
            if ! kill -0 "$sweep_pid" 2>/dev/null; then
                wait "$sweep_pid" || {
                    sweep_pid=""
                    tail -120 "$world_dir/sweep.log" >&2 || true
                    exit 1
                }
                sweep_pid=""
                break
            fi
            if grep -q '^truth alignment ' "$world_dir/sweep.log"; then
                break
            fi
            sleep 1
        done
        if ! grep -q '^truth alignment ' "$world_dir/sweep.log"; then
            echo "passive visibility sweep did not establish WBT alignment" >&2
            tail -120 "$world_dir/sweep.log" >&2 || true
            exit 1
        fi
        python3 "$EXPLORE_HELPER" start \
            --rbnx "$RBNX_BIN" \
            --deploy-dir "$WEBOTS_DIR" \
            --area-hint "$world" \
            --timeout-s "$duration" \
            --max-speed-m-s \
                "${ROBONIX_SCENE_BENCHMARK_EXPLORE_SPEED_MPS:-0.12}" \
            >"$world_dir/explore-start.json"
        explore_run_id="$(
            python3 - "$world_dir/explore-start.json" <<'PY'
import json
import sys
print(json.load(open(sys.argv[1], encoding="utf-8"))["run_id"])
PY
        )"
        if ! wait "$sweep_pid"; then
            sweep_pid=""
            python3 "$EXPLORE_HELPER" cancel \
                --rbnx "$RBNX_BIN" \
                --deploy-dir "$WEBOTS_DIR" \
                --run-id "$explore_run_id" \
                >"$world_dir/explore-cancel.json" 2>&1 || true
            exit 1
        fi
        sweep_pid=""
        python3 "$EXPLORE_HELPER" status \
            --rbnx "$RBNX_BIN" \
            --deploy-dir "$WEBOTS_DIR" \
            --run-id "$explore_run_id" \
            >"$world_dir/explore-status.json"
        python3 "$EXPLORE_HELPER" cancel \
            --rbnx "$RBNX_BIN" \
            --deploy-dir "$WEBOTS_DIR" \
            --run-id "$explore_run_id" \
            >"$world_dir/explore-cancel.json"
    else
        timeout "$((duration + ALIGNMENT_TIMEOUT_S + 30))" \
            docker exec "$SIM_CONTAINER" bash -lc "$sweep_command" \
            >"$world_dir/sweep.log" 2>&1
    fi
    docker cp "$SIM_CONTAINER:/tmp/scene_visibility.json" \
        "$world_dir/visibility.json"
    python3 - "$world_dir/visibility.json" \
        "${ROBONIX_SCENE_MIN_VISIBLE_RATIO:-0.25}" \
        "$REPO_ROOT/testing" \
        "${ROBONIX_SCENE_MAX_ODOM_PATH_REL_ERROR:-0.10}" \
        "${ROBONIX_SCENE_MAX_ODOM_PATH_ABS_ERROR_M:-0.50}" \
        "${ROBONIX_SCENE_MAX_LOCALIZED_P95_ERROR_M:-0.30}" \
        "${ROBONIX_SCENE_MAX_LOCALIZED_FINAL_ERROR_M:-0.45}" \
        "${ROBONIX_SCENE_MAX_LOCALIZED_YAW_P95_RAD:-0.20}" <<'PY'
import json
import sys
from pathlib import Path

path = sys.argv[1]
minimum_visible_ratio = float(sys.argv[2])
sys.path.insert(0, sys.argv[3])
max_odom_path_relative_error = float(sys.argv[4])
max_odom_path_absolute_error_m = float(sys.argv[5])
max_localized_p95_error_m = float(sys.argv[6])
max_localized_final_error_m = float(sys.argv[7])
max_localized_yaw_p95_rad = float(sys.argv[8])
from webots_scene_motion import odometry_path_agreement

if not 0.0 <= minimum_visible_ratio <= 1.0:
    raise SystemExit(
        "ROBONIX_SCENE_MIN_VISIBLE_RATIO must be in [0, 1], got "
        f"{minimum_visible_ratio}"
    )
for name, threshold in (
    ("ROBONIX_SCENE_MAX_LOCALIZED_P95_ERROR_M", max_localized_p95_error_m),
    ("ROBONIX_SCENE_MAX_LOCALIZED_FINAL_ERROR_M", max_localized_final_error_m),
    ("ROBONIX_SCENE_MAX_LOCALIZED_YAW_P95_RAD", max_localized_yaw_p95_rad),
):
    if threshold < 0.0:
        raise SystemExit(f"{name} must not be negative, got {threshold}")
value = json.load(open(path, encoding="utf-8"))
duration = float(value.get("duration_s") or 0.0)
samples = int(value.get("visibility_samples") or 0)
tf_failures = int(value.get("transform_failures") or 0)
depth_failures = int(value.get("depth_failures") or 0)
odom_discontinuities = int(value.get("odom_discontinuity_count") or 0)
aborted_reason = str(value.get("aborted_reason") or "").strip()
attempts = samples + tf_failures + depth_failures
minimum_samples = max(12, int(duration * 0.5))
failures = []
if samples < minimum_samples:
    failures.append(f"visibility_samples={samples} below {minimum_samples}")
if attempts and tf_failures / attempts > 0.25:
    failures.append(
        f"transform_failure_rate={tf_failures / attempts:.3f} above 0.25"
    )
if not value.get("visible_truth_ids"):
    failures.append("visible_truth_ids is empty")
truth_count = int(value.get("truth_count") or 0)
visible_count = len(value.get("visible_truth_ids") or ())
if truth_count <= 0:
    failures.append("truth_count is missing")
elif visible_count / truth_count < minimum_visible_ratio:
    failures.append(
        f"visible_truth_ratio={visible_count / truth_count:.3f} below "
        f"{minimum_visible_ratio:.3f}"
    )
if odom_discontinuities:
    failures.append(
        "odometry discontinuities detected: "
        f"count={odom_discontinuities}, "
        f"max_step_m={float(value.get('max_odom_step_m') or 0.0):.3f}"
    )
path_agreement = odometry_path_agreement(
    value.get("odometry_sources") or {},
    compared_sources=("localized", "wheel", "fused"),
    required_sources=(),
    max_relative_error=max_odom_path_relative_error,
    max_absolute_error_m=max_odom_path_absolute_error_m,
)
value["odometry_path_agreement"] = path_agreement
pose_agreement = value.get("localized_pose_agreement") or {}
pose_samples = int(pose_agreement.get("sample_count") or 0)
pose_rejected = int(pose_agreement.get("rejected_time_pairs") or 0)
pose_attempts = pose_samples + pose_rejected
pose_rejection_rate = pose_rejected / pose_attempts if pose_attempts else 1.0
translation_error = pose_agreement.get("translation_error_m") or {}
yaw_error = pose_agreement.get("yaw_error_rad") or {}
translation_p95_m = float(translation_error.get("p95") or 0.0)
translation_final_m = float(translation_error.get("final") or 0.0)
yaw_p95_rad = float(yaw_error.get("p95") or 0.0)
pose_failures = []
if pose_samples < minimum_samples:
    pose_failures.append(
        f"localized pose has {pose_samples} synchronized samples; "
        f"need at least {minimum_samples}"
    )
if pose_rejection_rate > 0.10:
    pose_failures.append(
        f"localized pose timestamp rejection rate {pose_rejection_rate:.1%} "
        "is above 10.0%"
    )
if translation_p95_m > max_localized_p95_error_m:
    pose_failures.append(
        f"localized translation P95 error {translation_p95_m:.3f} m exceeds "
        f"{max_localized_p95_error_m:.3f} m"
    )
if translation_final_m > max_localized_final_error_m:
    pose_failures.append(
        f"localized final translation error {translation_final_m:.3f} m "
        f"exceeds {max_localized_final_error_m:.3f} m"
    )
if yaw_p95_rad > max_localized_yaw_p95_rad:
    pose_failures.append(
        f"localized yaw P95 error {yaw_p95_rad:.3f} rad exceeds "
        f"{max_localized_yaw_p95_rad:.3f} rad"
    )
pose_agreement["gate"] = {
    "valid": not pose_failures,
    "max_translation_p95_error_m": max_localized_p95_error_m,
    "max_translation_final_error_m": max_localized_final_error_m,
    "max_yaw_p95_error_rad": max_localized_yaw_p95_rad,
    "timestamp_rejection_rate": round(pose_rejection_rate, 6),
    "failures": pose_failures,
}
value["localized_pose_agreement"] = pose_agreement
Path(path).write_text(
    json.dumps(value, indent=2, sort_keys=True),
    encoding="utf-8",
)
failures.extend(path_agreement["failures"])
failures.extend(pose_failures)
for warning in path_agreement["warnings"]:
    print(f"Scene benchmark odometry diagnostic: {warning}", file=sys.stderr)
if aborted_reason:
    failures.append(f"sweep aborted: {aborted_reason}")
if failures:
    raise SystemExit("invalid Scene benchmark sweep: " + "; ".join(failures))
PY

    # Score the converged persistent map, not an arbitrary point inside the
    # asynchronous cleanup cycle. With period_s=0.6 and a 10-tick cleanup
    # cadence, measured full-profile cleanup can take another 7.4 s; the old
    # fixed 8 s wait repeatedly captured canonical merge pairs immediately
    # before they were applied. Both A/B sides receive this same explicit
    # post-acquisition window and the trial manifest records its value.
    sleep "$POST_ACQUISITION_SETTLE_S"
    curl -fsS --max-time 10 "$SCENE_URL/api/state" >"$world_dir/state.json"
    curl -fsS --max-time 10 "$SCENE_URL/api/objects3d" \
        >"$world_dir/objects3d.json"
    curl -fsS --max-time 10 \
        "$SCENE_URL/api/objects3d?debug_clip_features=1" \
        >"$world_dir/objects3d-debug.json"
    python3 "$REPO_ROOT/testing/evaluate_webots_occupancy.py" \
        --world "$(benchmark_world_path "$world")" \
        --scene-state "$world_dir/state.json" \
        --json-out "$world_dir/occupancy-evaluation.json" \
        --overlay-out "$world_dir/occupancy-overlay.png" \
        >/dev/null
    python3 "$REPO_ROOT/testing/evaluate_webots_scene.py" \
        --world-id "$world" \
        --state-file "$world_dir/state.json" \
        --objects3d-file "$world_dir/objects3d.json" \
        --visibility-file "$world_dir/visibility.json" \
        >"$world_dir/evaluation.json"
    evaluation_file="$world_dir/evaluation.json"
    if [[ -n "$LABEL_JUDGMENTS_FILE" ]]; then
        python3 "$REPO_ROOT/testing/evaluate_webots_scene.py" \
            --world-id "$world" \
            --state-file "$world_dir/state.json" \
            --objects3d-file "$world_dir/objects3d.json" \
            --visibility-file "$world_dir/visibility.json" \
            --label-judgments-file "$LABEL_JUDGMENTS_FILE" \
            >"$world_dir/evaluation.semantic.json"
        evaluation_file="$world_dir/evaluation.semantic.json"
    fi

    python3 - "$evaluation_file" <<'PY'
import json
import sys
result = json.load(open(sys.argv[1], encoding="utf-8"))
print(
    "result "
    f"TP={result['tp']} FP={result['fp']} FN={result['fn']} "
    f"F1={result['f1']:.3f} label={result['label_accuracy']:.3f} "
    f"duplicate={result['duplicate_rate']:.3f} ghost={result['ghost_rate']:.3f}"
)
PY
    # Preserve the acquisition control evidence before `rbnx shutdown`
    # removes or replaces the per-component logs for the next world.
    capture_component_logs "$world_dir"
    stop_one || true
done

python3 - "$RESULT_ROOT" "${WORLDS[@]}" <<'PY'
import json
import pathlib
import sys

root = pathlib.Path(sys.argv[1])
worlds = sys.argv[2:]
def evaluation_path(world):
    semantic = root / world / "evaluation.semantic.json"
    return semantic if semantic.is_file() else root / world / "evaluation.json"

payload = {
    "result_root": str(root),
    "worlds": {
        world: (
            json.loads(evaluation_path(world).read_text())
            | {
                "occupancy_evaluation": json.loads(
                    (root / world / "occupancy-evaluation.json").read_text()
                )
            }
        )
        for world in worlds
    },
}
(root / "summary.json").write_text(
    json.dumps(payload, indent=2, sort_keys=True),
    encoding="utf-8",
)
print(root / "summary.json")
PY

review_args=()
for world in "${WORLDS[@]}"; do
    review_args+=(--run "$world=$RESULT_ROOT/$world")
done
python3 "$REPO_ROOT/testing/render_webots_scene_report.py" \
    "${review_args[@]}" \
    --output "$RESULT_ROOT/review.html"

trap - EXIT INT TERM
