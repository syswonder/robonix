#!/usr/bin/env bash
set -euo pipefail

RIDLC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RUST_DIR="$(cd "$RIDLC_DIR/.." && pwd)"
ARTIFACT_ROOT="$RIDLC_DIR/tests/.artifacts/zenoh_rmw_e2e"
WORKSPACE_DIR="$ARTIFACT_ROOT/ws"
LOG_DIR="$ARTIFACT_ROOT/logs"
E2E_ASSET_DIR="$RIDLC_DIR/tests/assets/robonix_ridlc_e2e"
SERVER_LOG="$LOG_DIR/robonix-server.log"
COMBINED_RUNTIME_LOG="$LOG_DIR/combined-runtime.log"
ZENOH_ROUTER_LOG="$LOG_DIR/zenoh-router.log"
SERVER_TCP_WAIT_RETRIES="${RIDLC_E2E_SERVER_TCP_WAIT_RETRIES:-40}"
RUNTIME_PROBE_WAIT_RETRIES="${RIDLC_E2E_RUNTIME_PROBE_WAIT_RETRIES:-20}"
ZENOH_ROUTER_WAIT_RETRIES="${RIDLC_E2E_ZENOH_ROUTER_WAIT_RETRIES:-20}"

if [[ -t 1 ]]; then
    RIDLC_TEST_PREFIX=$'\033[1;38;5;214m[ridlc-tests]\033[0m'
else
    RIDLC_TEST_PREFIX='[ridlc-tests]'
fi

log_info() {
    printf '%s %s\n' "$RIDLC_TEST_PREFIX" "$*"
}

log_error() {
    printf '%s %s\n' "$RIDLC_TEST_PREFIX" "$*" >&2
}

mkdir -p "$LOG_DIR"

cleanup() {
    local exit_code=$?
    if [[ -n "${ZENOH_ROUTER_PID:-}" ]]; then
        kill "$ZENOH_ROUTER_PID" >/dev/null 2>&1 || true
        wait "$ZENOH_ROUTER_PID" >/dev/null 2>&1 || true
    fi
    if [[ -n "${COMBINED_RUNTIME_PID:-}" ]]; then
        kill "$COMBINED_RUNTIME_PID" >/dev/null 2>&1 || true
        wait "$COMBINED_RUNTIME_PID" >/dev/null 2>&1 || true
    fi
    if [[ -n "${RBONIX_SERVER_PID:-}" ]]; then
        kill "$RBONIX_SERVER_PID" >/dev/null 2>&1 || true
        wait "$RBONIX_SERVER_PID" >/dev/null 2>&1 || true
    fi
    return "$exit_code"
}
trap cleanup EXIT

source_ros_env() {
    local distro="${ROS_DISTRO:-humble}"
    local setup_file="/opt/ros/${distro}/setup.bash"
    if [[ ! -f "$setup_file" ]]; then
        log_error "ros2 not found and ${setup_file} is missing"
        exit 2
    fi

    # shellcheck disable=SC1090
    set +u
    source "$setup_file"
    set -u
}

require_python_modules() {
    python3 - <<'PY'
import importlib.util
missing = [name for name in ("grpc", "rclpy") if importlib.util.find_spec(name) is None]
if missing:
    raise SystemExit("missing Python modules: " + ", ".join(missing))
PY
}

kill_port_processes() {
    local port="$1"
    local killed=false

    log_info "freeing tcp port $port"

    if command -v fuser >/dev/null 2>&1; then
        if fuser -k "${port}/tcp" >/dev/null 2>&1; then
            killed=true
        fi
    fi

    if [[ "$killed" == false ]] && command -v ss >/dev/null 2>&1; then
        local pids
        pids="$(
            ss -lptn "sport = :${port}" 2>/dev/null \
                | sed -n 's/.*pid=\([0-9]\+\).*/\1/p' \
                | sort -u
        )"
        if [[ -n "$pids" ]]; then
            while IFS= read -r pid; do
                [[ -n "$pid" ]] || continue
                kill "$pid" >/dev/null 2>&1 || true
                killed=true
            done <<<"$pids"
        fi
    fi

    if [[ "$killed" == false ]] && [[ -r /proc/net/tcp ]]; then
        python3 - "$port" <<'PY'
from pathlib import Path
import os
import signal
import sys
import time

port = int(sys.argv[1])
port_hex = f"{port:04X}"
inodes = set()

for table in ("/proc/net/tcp", "/proc/net/tcp6"):
    try:
        with open(table, "r", encoding="utf-8") as fh:
            next(fh, None)
            for line in fh:
                parts = line.split()
                if len(parts) < 10:
                    continue
                local_addr = parts[1]
                state = parts[3]
                if local_addr.endswith(f":{port_hex}") and state == "0A":
                    inodes.add(parts[9])
    except FileNotFoundError:
        pass

for proc_dir in Path("/proc").iterdir():
    if not proc_dir.name.isdigit():
        continue
    fd_dir = proc_dir / "fd"
    if not fd_dir.is_dir():
        continue
    for fd in fd_dir.iterdir():
        try:
            target = os.readlink(fd)
        except OSError:
            continue
        if target.startswith("socket:[") and target[8:-1] in inodes:
            try:
                os.kill(int(proc_dir.name), signal.SIGTERM)
            except OSError:
                pass
            break

time.sleep(0.5)
PY
    fi

    sleep 1
}

check_rmw() {
    export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"
    RIDLC_TEST_PREFIX="$RIDLC_TEST_PREFIX" python3 - <<'PY'
import os
import rclpy
from rclpy.utilities import get_rmw_implementation_identifier

prefix = os.environ.get("RIDLC_TEST_PREFIX", "[ridlc-tests]")

rclpy.init()
try:
    ident = get_rmw_implementation_identifier()
    print(f"{prefix} active RMW implementation: {ident}")
    expected = os.environ["RMW_IMPLEMENTATION"]
    if ident != expected:
        raise SystemExit(f"expected RMW {expected!r}, got {ident!r}")
finally:
    rclpy.shutdown()
PY
}

wait_for_tcp() {
    local host="$1"
    local port="$2"
    local retries="${3:-50}"
    python3 - "$host" "$port" "$retries" <<'PY'
import socket
import sys
import time

host = sys.argv[1]
port = int(sys.argv[2])
retries = int(sys.argv[3])

for _ in range(retries):
    try:
        with socket.create_connection((host, port), timeout=0.5):
            sys.exit(0)
    except OSError:
        time.sleep(0.2)

raise SystemExit(f"timed out waiting for {host}:{port}")
PY
}

wait_for_runtime_probe() {
    local retries="${1:-40}"
    for _ in $(seq 1 "$retries"); do
        if ros2 run robonix_ridlc_e2e runtime_probe >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.5
    done
    log_error "timed out waiting for runtime channel registration"
    return 1
}

start_zenoh_router() {
    kill_port_processes 7447
    log_info "starting zenoh router"
    ros2 run rmw_zenoh_cpp rmw_zenohd >"$ZENOH_ROUTER_LOG" 2>&1 &
    ZENOH_ROUTER_PID=$!
    wait_for_tcp 127.0.0.1 7447 "$ZENOH_ROUTER_WAIT_RETRIES"
}

python3 - "$ARTIFACT_ROOT" <<'PY'
from pathlib import Path
import shutil
import sys

path = Path(sys.argv[1])
if path.exists():
    shutil.rmtree(path, ignore_errors=False)
PY
mkdir -p "$LOG_DIR"

source_ros_env
export PYTHONNOUSERSITE=1
require_python_modules
start_zenoh_router
check_rmw

log_info "generating output for end-to-end workspace"
cargo run --manifest-path "$RIDLC_DIR/Cargo.toml" -- --lang python \
    --layout workspace \
    -I "$RIDLC_DIR/../robonix-interfaces/lib/robonix_runtime_interfaces" \
    -I "$RIDLC_DIR/../robonix-interfaces/lib/rcl_interfaces" \
    -I "$RIDLC_DIR/../robonix-interfaces/lib/common_interfaces" \
    -o "$WORKSPACE_DIR" \
    -i "$RIDLC_DIR/../robonix-interfaces/ridl"

log_info "generated ROS workspace at $WORKSPACE_DIR"
if [[ ! -d "$E2E_ASSET_DIR" ]]; then
    log_error "missing e2e asset package: $E2E_ASSET_DIR"
    exit 2
fi
mkdir -p "$WORKSPACE_DIR/src/test"
rm -rf "$WORKSPACE_DIR/src/test/robonix_ridlc_e2e"
cp -r "$E2E_ASSET_DIR" "$WORKSPACE_DIR/src/test/robonix_ridlc_e2e"

log_info "building ROS workspace"
colcon build \
    --base-paths "$WORKSPACE_DIR/src/generated" "$WORKSPACE_DIR/src/vendor" "$WORKSPACE_DIR/src/app" "$WORKSPACE_DIR/src/test" \
    --build-base "$WORKSPACE_DIR/build" \
    --install-base "$WORKSPACE_DIR/install" \
    --packages-up-to robonix_interfaces_app robonix_ridlc_e2e

# shellcheck disable=SC1090
set +u
source "$WORKSPACE_DIR/install/setup.bash"
set -u

# Keep colcon isolated from user site-packages, but allow runtime helpers to
# use the newer protobuf/grpc Python packages when available.
unset PYTHONNOUSERSITE

kill_port_processes 50051
log_info "starting robonix-server"
(
    cd "$RUST_DIR/robonix-server"
    ROBONIX_META_GRPC_ADDR=127.0.0.1:50051 \
    ROBONIX_META_GRPC_ENDPOINT=127.0.0.1:50051 \
    RUST_LOG=robonix_server=info \
    cargo run >"$SERVER_LOG" 2>&1
) &
RBONIX_SERVER_PID=$!

wait_for_tcp 127.0.0.1 50051 "$SERVER_TCP_WAIT_RETRIES"

log_info "starting combined generated runtime over ${RMW_IMPLEMENTATION}"
ROBONIX_RUNTIME_ENDPOINT=127.0.0.1:50051 \
ROBONIX_TEST_SERVER_ID=status_server_1 \
ROBONIX_TEST_STREAM_SERVER_ID=odom_provider_1 \
ROBONIX_TEST_COMMAND_SERVER_ID=move_server_1 \
ros2 run robonix_ridlc_e2e combined_runtime >"$COMBINED_RUNTIME_LOG" 2>&1 &
COMBINED_RUNTIME_PID=$!

wait_for_runtime_probe "$RUNTIME_PROBE_WAIT_RETRIES"

CHANNEL_NAME="$(
    ROBONIX_RUNTIME_ENDPOINT=127.0.0.1:50051 \
    ROBONIX_TEST_SERVER_ID=status_server_1 \
    ros2 run robonix_ridlc_e2e runtime_probe
)"
log_info "resolved runtime channel: $CHANNEL_NAME"

CLIENT_OUTPUT="$(
    ROBONIX_RUNTIME_ENDPOINT=127.0.0.1:50051 \
    ROBONIX_TEST_SERVER_ID=status_server_1 \
    ROBONIX_TEST_REQUEST=ping \
    ROBONIX_TEST_EXPECTED=ok:ping \
    ros2 run robonix_ridlc_e2e query_client
)"

log_info "client response: $CLIENT_OUTPUT"
if [[ "$CLIENT_OUTPUT" != "ok:ping" ]]; then
    log_error "unexpected client output: $CLIENT_OUTPUT"
    exit 1
fi

STREAM_OUTPUT="$(
    ROBONIX_RUNTIME_ENDPOINT=127.0.0.1:50051 \
    ROBONIX_TEST_STREAM_SERVER_ID=odom_provider_1 \
    ros2 run robonix_ridlc_e2e stream_subscriber
)"

log_info "stream message: $STREAM_OUTPUT"
if [[ "$STREAM_OUTPUT" != "1.25,-0.50" ]]; then
    log_error "unexpected stream output: $STREAM_OUTPUT"
    exit 1
fi

COMMAND_OUTPUT="$(
    ROBONIX_RUNTIME_ENDPOINT=127.0.0.1:50051 \
    ROBONIX_TEST_COMMAND_SERVER_ID=move_server_1 \
    ROBONIX_TEST_COMMAND_EXPECTED=ok:1.25 \
    ros2 run robonix_ridlc_e2e command_client
)"

log_info "command result: $COMMAND_OUTPUT"
if [[ "$COMMAND_OUTPUT" != "ok:1.25" ]]; then
    log_error "unexpected command result: $COMMAND_OUTPUT"
    exit 1
fi

log_info "zenoh_rmw end-to-end test passed"
