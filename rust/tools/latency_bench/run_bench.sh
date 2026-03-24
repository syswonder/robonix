#!/usr/bin/env bash
# Run latency benchmark for all transports.
# Prerequisites:
#   - pip install -r requirements.txt
#   - For ROS2: build latency_bench_msgs and source install/setup.bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

ITERATIONS="${ITERATIONS:-10000}"
WARMUP="${WARMUP:-100}"
PAYLOAD_SIZE="${PAYLOAD_SIZE:-64}"
RESULTS_DIR="${RESULTS_DIR:-results}"
mkdir -p "$RESULTS_DIR"

echo "[latency_bench] iterations=$ITERATIONS warmup=$WARMUP payload_size=$PAYLOAD_SIZE"
echo "[latency_bench] results -> $RESULTS_DIR"
echo ""

# Clean up any leftover processes on benchmark ports (from previous runs)
kill_ports() {
  python3 - "$@" <<'PY'
import os, sys
for port in sys.argv[1:]:
  port = int(port)
  hex_port = format(port, "04X")
  inodes = set()
  for path in ("/proc/net/tcp", "/proc/net/tcp6"):
    try:
      with open(path) as f:
        next(f)
        for line in f:
          parts = line.split()
          local = parts[1]
          state = parts[3]
          inode = parts[9]
          _ip, port_hex = local.split(":")
          if port_hex.upper() == hex_port and state in ("0A", "01"):
            inodes.add(inode)
    except (FileNotFoundError, IndexError):
      pass
  pids = []
  for pid in os.listdir("/proc"):
    if not pid.isdigit(): continue
    try:
      for fd in os.listdir(f"/proc/{pid}/fd"):
        try:
          t = os.readlink(f"/proc/{pid}/fd/{fd}")
          if t.startswith("socket:[") and t[8:-1] in inodes:
            pids.append(int(pid))
            break
        except OSError: pass
    except OSError: pass
  for pid in sorted(set(pids)):
    print(f"[latency_bench] Killing leftover process {pid} on port {port}", flush=True)
    try: os.kill(pid, 15)
    except ProcessLookupError: pass
PY
}
kill_ports 50052 5555 18080
sleep 2

# Ensure proto generated
if [[ ! -f proto/echo_pb2.py ]]; then
  echo "[latency_bench] Generating gRPC stubs..."
  python3 -m grpc_tools.protoc -I. --python_out=. --grpc_python_out=. proto/echo.proto
fi

run_one() {
  local transport=$1
  local server_cmd=$2
  local server_pid=""
  local wait_sec=${3:-2}

  echo "[latency_bench] Starting $transport server..."
  eval "$server_cmd" &
  server_pid=$!
  sleep "$wait_sec"

  echo "[latency_bench] Running $transport benchmark..."
  if python3 -m latency_bench.benchmark \
    --transport "$transport" \
    --iterations "$ITERATIONS" \
    --warmup "$WARMUP" \
    --payload-size "$PAYLOAD_SIZE" \
    -o "$RESULTS_DIR/${transport}_$(date +%Y%m%d_%H%M%S).json"; then
    echo "[latency_bench] $transport OK"
  else
    echo "[latency_bench] $transport FAILED"
  fi

  kill "$server_pid" 2>/dev/null || true
  wait "$server_pid" 2>/dev/null || true
  sleep 2
}

# gRPC
run_one grpc "python3 -m latency_bench.servers.grpc_server" 2

# ZeroMQ
run_one zmq "python3 -m latency_bench.servers.zmq_server" 1

# HTTP
run_one http "python3 -m latency_bench.servers.http_server" 2

# ROS2: need built workspace
ROS2_WS="$SCRIPT_DIR/ros2_ws"
if [[ -d "$ROS2_WS" ]] && [[ -f "$ROS2_WS/install/setup.bash" ]]; then
  set +u
  source "$ROS2_WS/install/setup.bash"
  set -u

  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  echo "[latency_bench] Starting ROS2 ($RMW_IMPLEMENTATION) server..."
  python3 -m latency_bench.servers.ros2_server 2>/dev/null &
  ROS2_PID=$!
  sleep 3

  echo "[latency_bench] Running ROS2 benchmark..."
  if python3 -m latency_bench.benchmark \
    --transport ros2 \
    --iterations "$ITERATIONS" \
    --warmup "$WARMUP" \
    --payload-size "$PAYLOAD_SIZE" \
    -o "$RESULTS_DIR/ros2_$(date +%Y%m%d_%H%M%S).json"; then
    echo "[latency_bench] ROS2 OK"
  else
    echo "[latency_bench] ROS2 FAILED"
  fi

  kill $ROS2_PID 2>/dev/null || true
  wait $ROS2_PID 2>/dev/null || true
  sleep 3
else
  echo "[latency_bench] Skipping ROS2: build with ./build_ros2.sh first"
fi

echo ""
echo "[latency_bench] Done. Results in $RESULTS_DIR/"
