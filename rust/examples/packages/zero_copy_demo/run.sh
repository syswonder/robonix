#!/usr/bin/env bash
# Robonix Zero-Copy Demo launcher
#
# Usage:
#   ./run.sh setup              # build Rust + install Python deps
#   ./run.sh start [frames]     # launch 3-process pipeline
#   ./run.sh benchmark [opts]   # multi-process benchmark with timing
#   ./run.sh compare [W H N]    # full ROS 2 vs Robonix comparison
set -euo pipefail
cd "$(dirname "$0")"

RUST_ROOT="$(cd ../../.. && pwd)"
VENV=".venv"
SERVER_ADDR="${ROBONIX_SERVER:-127.0.0.1:50051}"
PROTO_GEN_DIR="$(cd ../../proto_gen 2>/dev/null && pwd || echo "")"

_build_rust() {
    echo "[+] Building robonix-buffer (Rust)..."
    (cd "$RUST_ROOT" && cargo build -p robonix-buffer --release)
    echo "    librobonix_buffer.so -> $RUST_ROOT/target/release/"
}

_activate_env() {
    if [ -n "${CONDA_PREFIX:-}" ]; then return 0; fi
    if [ -d "$VENV" ]; then source "$VENV/bin/activate"; return 0; fi
    return 1
}

_check_deps() {
    python -c "import numpy, torch, grpc" 2>/dev/null
}

_ensure_venv() {
    if [ ! -d "$VENV" ]; then
        if command -v uv &>/dev/null; then
            uv venv "$VENV" --python 3.12 2>/dev/null \
                || uv venv "$VENV" --python 3.11 2>/dev/null \
                || uv venv "$VENV"
        else
            echo "[!] uv not found: curl -LsSf https://astral.sh/uv/install.sh | sh"
            exit 1
        fi
    fi
}

_install_deps() {
    echo "[+] Installing Python dependencies..."
    if command -v uv &>/dev/null; then uv pip install -e . --quiet
    else pip install -e . --quiet; fi
    echo "[+] Done. PyTorch with CUDA must be installed separately (see README)."
}

_ensure_ready() {
    if ! _activate_env; then
        echo "[!] No Python environment. Run: ./run.sh setup"; exit 1
    fi
    if ! _check_deps; then
        echo "[!] Missing deps. Run: ./run.sh setup"; exit 1
    fi
}

_SERVER_PID=""
_ensure_server() {
    if [ -n "$PROTO_GEN_DIR" ] && python3 -c "
import grpc, sys
sys.path.insert(0, '$PROTO_GEN_DIR')
import robonix_runtime_pb2_grpc as g
ch = grpc.insecure_channel('$SERVER_ADDR')
grpc.channel_ready_future(ch).result(timeout=2)
" 2>/dev/null; then
        echo "[+] robonix-server already running at $SERVER_ADDR"
    else
        echo "[+] Starting robonix-server..."
        robonix-server &
        _SERVER_PID=$!
        sleep 2
        echo "[+] Started robonix-server (PID=$_SERVER_PID)"
    fi
}

_cleanup_server() {
    if [ -n "$_SERVER_PID" ]; then
        echo "[+] Stopping robonix-server (PID=$_SERVER_PID)"
        kill "$_SERVER_PID" 2>/dev/null || true
        wait "$_SERVER_PID" 2>/dev/null || true
    fi
}

CMD="${1:-help}"
case "$CMD" in
    setup)
        _build_rust
        if [ -n "${CONDA_PREFIX:-}" ]; then
            echo "[*] Using conda env: $(basename "$CONDA_PREFIX")"
            _install_deps
        else
            _ensure_venv; source "$VENV/bin/activate"; _install_deps
        fi
        echo ""; echo "[+] Ready. Run:  ./run.sh start 200"
        ;;
    start)
        shift || true; _ensure_ready
        _ensure_server; trap _cleanup_server EXIT
        FRAMES="${1:-200}"
        echo "[+] Multi-process zero-copy pipeline (${FRAMES} frames)"
        echo ""
        export RBNX_FRAMES="$FRAMES"
        echo "[1/3] camera_node..."
        python -m zero_copy_demo.nodes.camera_node &
        CAM_PID=$!
        sleep 1
        echo "[2/3] yolo_node..."
        python -m zero_copy_demo.nodes.yolo_node &
        YOLO_PID=$!
        sleep 2
        echo "[3/3] edge_node..."
        python -m zero_copy_demo.nodes.edge_node &
        EDGE_PID=$!
        echo "[+] All nodes running (cam=$CAM_PID yolo=$YOLO_PID edge=$EDGE_PID)"
        echo "    Ctrl+C to stop."
        trap "kill $CAM_PID $YOLO_PID $EDGE_PID 2>/dev/null; wait; _cleanup_server" INT TERM EXIT
        wait $CAM_PID $YOLO_PID $EDGE_PID 2>/dev/null || true
        echo "[+] Done."
        ;;
    benchmark)
        shift || true; _ensure_ready
        _ensure_server; trap _cleanup_server EXIT
        python -m zero_copy_demo.benchmark --server "$SERVER_ADDR" "$@"
        ;;
    compare)
        shift || true; _ensure_ready
        bash docker/run_comparison.sh "$@"
        ;;
    help|*)
        cat <<'EOF'
Robonix Zero-Copy Demo

Usage: ./run.sh <command> [options]

Commands:
  setup                  Build Rust lib + install Python deps
  start [N]              Launch 3-process pipeline (N frames, default 200)
  benchmark [opts]       Multi-process benchmark with timing report
  compare [W H N]        Full ROS 2 vs Robonix comparison (Docker + host)

Examples:
  ./run.sh setup
  ./run.sh start 200
  ./run.sh benchmark --frames 200 --width 1920 --height 1080
  ./run.sh compare 1920 1080 200
EOF
        ;;
esac
