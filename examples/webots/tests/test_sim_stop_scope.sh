#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
set -euo pipefail

TEST_DIR="$(cd "$(dirname "$0")" && pwd)"
SIM_DIR="$(cd "$TEST_DIR/../sim" && pwd)"
STOP_SCRIPT="$SIM_DIR/stop.sh"
TEMP_ROOT="$(mktemp -d)"
ACTIVE_PIDS=()

cleanup() {
    local pid
    for pid in "${ACTIVE_PIDS[@]:-}"; do
        kill "$pid" 2>/dev/null || true
        wait "$pid" 2>/dev/null || true
    done
    rm -rf "$TEMP_ROOT"
}
trap cleanup EXIT

fail() {
    echo "test_sim_stop_scope: $*" >&2
    exit 1
}

wait_until_stopped() {
    local pid="$1"
    local _
    for _ in $(seq 1 50); do
        kill -0 "$pid" 2>/dev/null || return 0
        sleep 0.02
    done
    return 1
}

MOCK_BIN="$TEMP_ROOT/mock-bin"
mkdir -p "$MOCK_BIN"
cat > "$MOCK_BIN/docker" <<'MOCK_DOCKER'
#!/usr/bin/env bash
set -euo pipefail
: "${MOCK_DOCKER_LOG:?}"
: "${MOCK_CONTAINER_DB:?}"
printf 'project_env=%s container_env=%s args=' \
    "${ROBONIX_SIM_PROJECT:-}" "${ROBONIX_SIM_CONTAINER:-}" >> "$MOCK_DOCKER_LOG"
printf ' %q' "$@" >> "$MOCK_DOCKER_LOG"
printf '\n' >> "$MOCK_DOCKER_LOG"

[[ "${1:-}" == "compose" ]] || exit 90
shift
project=""
while [[ $# -gt 0 ]]; do
    case "$1" in
        --project-name)
            project="$2"
            shift 2
            ;;
        -f)
            shift 2
            ;;
        down)
            shift
            [[ "${1:-}" == "--remove-orphans" ]] || exit 91
            [[ -n "$project" ]] || exit 92
            temporary="${MOCK_CONTAINER_DB}.tmp.$$"
            awk -F '|' -v project="$project" '$1 != project' \
                "$MOCK_CONTAINER_DB" > "$temporary"
            mv "$temporary" "$MOCK_CONTAINER_DB"
            exit 0
            ;;
        *) exit 93 ;;
    esac
done
exit 94
MOCK_DOCKER
chmod +x "$MOCK_BIN/docker"

spawn_named_process() {
    local marker="$1"
    python3 -c 'import time; time.sleep(300)' "$marker" >/dev/null 2>&1 &
    SPAWNED_PID=$!
    ACTIVE_PIDS+=("$SPAWNED_PID")
}

run_case() {
    local label="$1" project="$2" container="$3" use_defaults="$4"
    local case_dir="$TEMP_ROOT/$label"
    local state_dir="$case_dir/state"
    local container_db="$case_dir/containers"
    local docker_log="$case_dir/docker.log"
    mkdir -p "$state_dir"

    spawn_named_process "$SIM_DIR/start_rviz.sh"
    local rviz_pid="$SPAWNED_PID"
    spawn_named_process "robonix-atlas-decoy"
    local host_decoy_pid="$SPAWNED_PID"

    printf '%s\n' "$rviz_pid" > "$state_dir/${project}--${container}.rviz.pid"
    printf '%s|%s\n' "$project" "$container" > "$container_db"
    printf '%s|%s\n' "unrelated-project" "unrelated-container" >> "$container_db"

    if [[ "$use_defaults" == "1" ]]; then
        (
            unset ROBONIX_SIM_PROJECT ROBONIX_SIM_CONTAINER
            ROBONIX_SIM_STATE_DIR="$state_dir" \
            MOCK_DOCKER_LOG="$docker_log" \
            MOCK_CONTAINER_DB="$container_db" \
            PATH="$MOCK_BIN:$PATH" \
                "$STOP_SCRIPT"
        )
    else
        ROBONIX_SIM_PROJECT="$project" \
        ROBONIX_SIM_CONTAINER="$container" \
        ROBONIX_SIM_STATE_DIR="$state_dir" \
        MOCK_DOCKER_LOG="$docker_log" \
        MOCK_CONTAINER_DB="$container_db" \
        PATH="$MOCK_BIN:$PATH" \
            "$STOP_SCRIPT"
    fi

    wait_until_stopped "$rviz_pid" \
        || fail "$label: recorded RViz wrapper still running"
    kill -0 "$host_decoy_pid" 2>/dev/null \
        || fail "$label: unrelated host process was killed"
    [[ ! -e "$state_dir/${project}--${container}.rviz.pid" ]] \
        || fail "$label: scoped RViz PID file was not removed"
    ! grep -q "^${project}|${container}$" "$container_db" \
        || fail "$label: selected simulator project was not stopped"
    grep -q '^unrelated-project|unrelated-container$' "$container_db" \
        || fail "$label: unrelated simulator project was touched"
    grep -q "project_env=${project} container_env=${container}" "$docker_log" \
        || fail "$label: selected project/container were not exported to Compose"
    grep -q -- "--project-name ${project}" "$docker_log" \
        || fail "$label: Compose did not receive the selected project"
    ! grep -Eq '(^| )rm( |$)|(^| )exec( |$)' "$docker_log" \
        || fail "$label: teardown used an unscoped docker rm/exec operation"

    wait "$rviz_pid" 2>/dev/null || true
}

run_case default robonix_tiago_sim robonix_tiago_sim 1
run_case custom demo-project-42 demo-container-42 0

# A stale PID may have been reused by an unrelated process. Even if that PID is
# present in the selected state file, stop.sh must refuse to kill it unless its
# command line names this simulator's start_rviz.sh wrapper.
STALE_DIR="$TEMP_ROOT/stale"
mkdir -p "$STALE_DIR/state"
spawn_named_process "unrelated-rviz-looking-process"
stale_decoy_pid="$SPAWNED_PID"
printf '%s\n' "$stale_decoy_pid" > "$STALE_DIR/state/stale-project--stale-container.rviz.pid"
printf '%s|%s\n' stale-project stale-container > "$STALE_DIR/containers"
ROBONIX_SIM_PROJECT=stale-project \
ROBONIX_SIM_CONTAINER=stale-container \
ROBONIX_SIM_STATE_DIR="$STALE_DIR/state" \
MOCK_DOCKER_LOG="$STALE_DIR/docker.log" \
MOCK_CONTAINER_DB="$STALE_DIR/containers" \
PATH="$MOCK_BIN:$PATH" \
    "$STOP_SCRIPT"
kill -0 "$stale_decoy_pid" 2>/dev/null \
    || fail "stale PID reuse killed an unrelated process"

echo "test_sim_stop_scope: PASS"
