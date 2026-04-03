#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DEMO_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
RESULTS_DIR="$DEMO_DIR/results"
if [ -d "$DEMO_DIR/proto_gen" ]; then
  PROTO_GEN_DIR="$(cd "$DEMO_DIR/proto_gen" && pwd)"
else
  PROTO_GEN_DIR="$(cd "$DEMO_DIR/../../proto_gen" && pwd)"
fi
IMAGE_NAME="robonix-ros2-bench:latest"

WIDTH="${1:-1920}"
HEIGHT="${2:-1080}"
FRAMES="${3:-200}"
SERVER_ADDR="${ROBONIX_ATLAS:-127.0.0.1:50051}"

mkdir -p "$RESULTS_DIR"

echo "=================================================================="
echo "  Robonix vs ROS 2/FastDDS -- Full Pipeline Comparison"
echo "  Frame: ${WIDTH}x${HEIGHT} RGB8  |  ${FRAMES} frames"
echo "  Control plane: robonix-atlas @ ${SERVER_ADDR}"
echo "=================================================================="

# ── 0. Ensure robonix-atlas is running ──────────────────────────────
echo ""
echo "[Step 0] Check robonix-atlas"
echo "-----------------------------------------------------"

_server_started_by_us=false

if python3 -c "
import grpc, sys
sys.path.insert(0, '$PROTO_GEN_DIR')
import robonix_runtime_pb2_grpc as g
ch = grpc.insecure_channel('$SERVER_ADDR')
try:
    grpc.channel_ready_future(ch).result(timeout=2)
    print('[+] robonix-atlas is running')
except:
    sys.exit(1)
" 2>/dev/null; then
    echo "[+] Using existing robonix-atlas at $SERVER_ADDR"
else
    echo "[!] robonix-atlas not running. Starting it..."
    robonix-atlas &
    _SERVER_PID=$!
    _server_started_by_us=true
    sleep 2
    echo "[+] Started robonix-atlas (PID=$_SERVER_PID)"
fi

# Cleanup server on exit if we started it
cleanup() {
    if $_server_started_by_us && [ -n "${_SERVER_PID:-}" ]; then
        echo "[+] Stopping robonix-atlas (PID=$_SERVER_PID)"
        kill "$_SERVER_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# ── 1. Build ROS 2 benchmark image ──────────────────────────────────
echo ""
echo "[Step 1] Build ROS 2 / FastDDS benchmark container"
echo "-----------------------------------------------------"

# Copy shared files into docker build context
rm -rf "$SCRIPT_DIR/proto_gen"
cp -r "$PROTO_GEN_DIR" "$SCRIPT_DIR/proto_gen"
cp "$DEMO_DIR/zero_copy_demo/common.py" "$SCRIPT_DIR/common.py"

# Always rebuild to pick up code changes
docker build -t "$IMAGE_NAME" -f "$SCRIPT_DIR/Dockerfile.ros2_bench" "$SCRIPT_DIR"

rm -rf "$SCRIPT_DIR/proto_gen" "$SCRIPT_DIR/common.py"

# ── 2. Run ROS 2 benchmark inside container ─────────────────────────
echo ""
echo "[Step 2] Run TRADITIONAL path (ROS 2 / FastDDS in Docker)"
echo "-----------------------------------------------------"

docker run --rm --gpus all \
    --network=host \
    --ipc=host \
    -v "$RESULTS_DIR:/results" \
    -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e ROBONIX_ATLAS="$SERVER_ADDR" \
    "$IMAGE_NAME" \
    bash -c "source /opt/ros/humble/setup.bash && \
             python3 /bench/ros2_bench.py \
                --server $SERVER_ADDR \
                --width $WIDTH --height $HEIGHT --frames $FRAMES \
                --json --results-dir /results" \
    > "$RESULTS_DIR/ros2_raw.txt" \
    2> >(tee "$RESULTS_DIR/ros2_log.txt" >&2)

# Extract JSON (ultralytics may pollute stdout)
python3 -c "
import json, sys
raw = open('$RESULTS_DIR/ros2_raw.txt').read()
idx = 0
while True:
    pos = raw.find('{', idx)
    if pos < 0:
        print('ERROR: no JSON found in ros2 output', file=sys.stderr)
        sys.exit(1)
    try:
        obj = json.loads(raw[pos:])
        break
    except json.JSONDecodeError:
        idx = pos + 1
with open('$RESULTS_DIR/ros2_results.json', 'w') as f:
    json.dump(obj, f, indent=2)
"

echo "[+] ROS 2 results -> $RESULTS_DIR/ros2_results.json"

# ── 3. Run Robonix multi-process benchmark on host ───────────────────
echo ""
echo "[Step 3] Run ROBONIX path (multi-process, zero-copy)"
echo "-----------------------------------------------------"

cd "$DEMO_DIR"
python -m zero_copy_demo.benchmark \
    --server "$SERVER_ADDR" \
    --width "$WIDTH" --height "$HEIGHT" --frames "$FRAMES" \
    --json --results-dir "$RESULTS_DIR" \
    > "$RESULTS_DIR/robonix_raw.txt" \
    2> >(tee "$RESULTS_DIR/robonix_log.txt" >&2)

python3 -c "
import json, sys
raw = open('$RESULTS_DIR/robonix_raw.txt').read()
idx = 0
while True:
    pos = raw.find('{', idx)
    if pos < 0:
        print('ERROR: no JSON found in robonix output', file=sys.stderr)
        sys.exit(1)
    try:
        obj = json.loads(raw[pos:])
        break
    except json.JSONDecodeError:
        idx = pos + 1
with open('$RESULTS_DIR/robonix_results.json', 'w') as f:
    json.dump(obj, f, indent=2)
"

echo "[+] Robonix results -> $RESULTS_DIR/robonix_results.json"

# ── 4. Print combined comparison ────────────────────────────────────
echo ""
echo "[Step 4] Combined comparison"
echo "-----------------------------------------------------"

python3 -c "
import json, sys

ros2 = json.load(open('$RESULTS_DIR/ros2_results.json'))
rbnx = json.load(open('$RESULTS_DIR/robonix_results.json'))

W = ros2['config']['width']
H = ros2['config']['height']
mb = ros2['config']['frame_mb']
sep = '=' * 78

print(f'\n{sep}')
print(f'  COMPARISON: ROS 2/FastDDS vs Robonix')
print(f'  Frame: {W}x{H} RGB8 ({mb:.2f} MB)  |  Camera -> Node A (YOLO) + Node B (Edge)')
print(f'  Both paths coordinated by robonix-atlas')
print(sep)

def row(label, trad_us, rbnx_us, trad_tag='', rbnx_tag=''):
    sp = trad_us / rbnx_us if rbnx_us > 0.5 else float('inf')
    sp_str = f'{sp:.1f}x' if sp < 1000 else 'INF'
    tt = f'{trad_us:>8,.0f}'
    rt = f'{rbnx_us:>8,.0f}' if rbnx_us > 0.5 else '      ~0'
    print(f'  {label:<44s} {tt} us {trad_tag:<5s}  {rt} us {rbnx_tag:<5s}  {sp_str:>8s}')

print(f'  {\"Step\":<44s} {\"ROS 2\":>11s}       {\"Robonix\":>11s}       {\"Speedup\":>8s}')
print(f'  {\"-\"*76}')

row('Driver DMA -> buffer',
    ros2['driver_dma_us'], rbnx['driver_dma_us'], '', 'WRITE')

row('Serialize / Image.data (tobytes)',
    ros2['serialize_us'], 0, 'COPY', '')

row('Node A: deserialize + copy',
    ros2['node_a']['deser_us'], rbnx['node_a']['read_us'], 'COPY', 'FREE')

row('Node A: preprocess',
    ros2['node_a']['preprocess_us'], rbnx['node_a']['preprocess_gpu_us'], 'COPY', 'FREE')

row('Node A: CPU->GPU H2D',
    ros2['node_a']['h2d_us'], rbnx['node_a']['h2d_us'], 'COPY', 'DMA')

row('Node A: YOLO inference',
    ros2['node_a']['compute_us'], rbnx['node_a']['infer_us'])

row('Node B: deserialize + copy',
    ros2['node_b']['deser_us'], rbnx['node_b']['gpu_read_us'], 'COPY', 'FREE')

row('Node B: preprocess',
    ros2['node_b']['preprocess_us'], 0, 'COPY', '')

row('Node B: CPU->GPU H2D',
    ros2['node_b']['h2d_us'], 0, 'COPY', '')

row('Node B: compute',
    ros2['node_b']['compute_us'], rbnx['node_b']['compute_us'])

print(f'  {\"-\"*76}')
row('TOTAL', ros2['total_us'], rbnx['total_us'])

print()
print(f'  Copies:     ROS 2 = 7    Robonix = 1  (H2D DMA only)')
print(f'  CPU BW:     ROS 2 = ~{mb*7:.1f} MB/frame    Robonix = ~{mb:.1f} MB/frame')
print(f'  Jitter (s): ROS 2 = {ros2[\"total_std_us\"]:.0f} us    Robonix = {rbnx[\"total_std_us\"]:.0f} us')
print(f'  Node B:     ROS 2 = full chain ({ros2[\"node_b\"][\"total_us\"]:,.0f} us)    Robonix = GPU-only ({rbnx[\"node_b\"][\"compute_us\"]:.0f} us)')
print(f'{sep}\n')
"

# ── 5. Verify computation results match ──────────────────────────────
echo ""
echo "[Step 5] Result verification (same computation, different paths)"
echo "-----------------------------------------------------"

python3 "$SCRIPT_DIR/verify_results.py" "$RESULTS_DIR"

echo "  All results in: $RESULTS_DIR/"
