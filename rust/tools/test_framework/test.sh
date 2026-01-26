#!/bin/bash
# Test Framework Main Script
# Manages test execution with process isolation and cleanup

set -e

# Set timezone to Asia/Shanghai (UTC+8)
export TZ=Asia/Shanghai

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
TEST_DIR="$SCRIPT_DIR"

# Test type: rust, python, or all
TEST_TYPE="${1:-all}"

# DDS type: rustdds or fastdds
DDS_TYPE="${2:-rustdds}"

# Concurrency level (number of concurrent clients)
CONCURRENCY="${3:-3}"

# Number of requests
REQUESTS="${4:-1000}"

# Request rate
RATE="${5:-100}"

# Test duration
DURATION="${6:-0}"

# Process group for isolation
TEST_PGID=$$

# Log directory (can be overridden by environment variable)
LOG_DIR_BASE="${LOG_DIR_BASE:-$TEST_DIR/logs}"
LOG_DIR="$LOG_DIR_BASE"
mkdir -p "$LOG_DIR"

# Create test-specific log directory with timestamp and concurrency level
# Include test type in directory name to distinguish rust/python tests
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
if [ "$TEST_TYPE" = "all" ]; then
    # For "all" tests, use DDS type
    LOG_DIR_NAME="${TIMESTAMP}_${DDS_TYPE}_c${CONCURRENCY}"
else
    # For specific test types, use test type name
    LOG_DIR_NAME="${TIMESTAMP}_${TEST_TYPE}_${DDS_TYPE}_c${CONCURRENCY}"
fi
DDS_LOG_DIR="$LOG_DIR/$LOG_DIR_NAME"
mkdir -p "$DDS_LOG_DIR"

# PID file for tracking processes
PID_FILE="$DDS_LOG_DIR/test_pids.txt"
> "$PID_FILE"

# Cleanup function
cleanup() {
    echo ""
    echo "=== Cleaning up test processes ==="
    
    # Kill all processes in our process group
    if [ -n "$TEST_PGID" ]; then
        kill -TERM -$TEST_PGID 2>/dev/null || true
        sleep 2
        kill -KILL -$TEST_PGID 2>/dev/null || true
    fi
    
    # Kill robonix-core if running
    pkill -9 -f "robonix-core" 2>/dev/null || true
    
    # Kill any test processes
    if [ -f "$PID_FILE" ]; then
        while read pid; do
            [ -n "$pid" ] && kill -9 "$pid" 2>/dev/null || true
        done < "$PID_FILE"
    fi
    
    # Clean up ROS2 daemon if needed
    # ros2 daemon stop 2>/dev/null || true
    
    echo "✓ Cleanup complete"
}

# Set up signal handlers
trap cleanup EXIT INT TERM

# Source ROS2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

# Source robonix-sdk
if [ -f "$ROOT_DIR/robonix-sdk/install/setup.bash" ]; then
    source "$ROOT_DIR/robonix-sdk/install/setup.bash"
fi

# Ensure ROS2 environment is properly set
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Start robonix-core
start_core() {
    echo "=== Starting robonix-core (RustDDS) ==="
    
    # Kill any existing core
    pkill -9 -f "robonix-core" 2>/dev/null || true
    sleep 1
    
    # Free port 8000
    PORT=8000
    if command -v fuser >/dev/null 2>&1; then
        fuser -k ${PORT}/tcp 2>/dev/null || true
    fi
    sleep 1
    
    # Start core in background
    cd "$ROOT_DIR"
    ROBONIX_WEB_ASSETS_DIR="$ROOT_DIR/robonix-core/web" \
    ROBONIX_WEB_PORT=8000 \
    RUST_LOG=robonix_core=info \
    robonix-core > "$DDS_LOG_DIR/core.log" 2>&1 &
    CORE_PID=$!
    echo "$CORE_PID" >> "$PID_FILE"
    
    # Wait for core to be ready
    echo "Waiting for robonix-core to start..."
    
    # First, wait a bit for the process to actually start
    sleep 2
    
    # Check if process is still running
    if ! kill -0 "$CORE_PID" 2>/dev/null; then
        echo "✗ robonix-core process died immediately"
        echo "Check log: $DDS_LOG_DIR/core.log"
        if [ -f "$DDS_LOG_DIR/core.log" ]; then
            echo "Last 10 lines of log:"
            tail -10 "$DDS_LOG_DIR/core.log"
        fi
        return 1
    fi
    
    # Check log for successful startup message
    if [ -f "$DDS_LOG_DIR/core.log" ]; then
        if grep -q "robonix core ready" "$DDS_LOG_DIR/core.log"; then
            echo "✓ robonix-core started successfully (from log)"
            # Give it a moment for services to be discoverable
            sleep 2
        fi
    fi
    
    # Ensure ROS2 environment is available for service discovery
    # Re-source in case environment wasn't properly set
    if [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash >/dev/null 2>&1
    fi
    if [ -f "$ROOT_DIR/robonix-sdk/install/setup.bash" ]; then
        source "$ROOT_DIR/robonix-sdk/install/setup.bash" >/dev/null 2>&1
    fi
    
    # Wait for service to appear (with retries)
    for i in {1..30}; do
        # Check if process is still running
        if ! kill -0 "$CORE_PID" 2>/dev/null; then
            echo "✗ robonix-core process died after ${i}s"
            echo "Check log: $DDS_LOG_DIR/core.log"
            if [ -f "$DDS_LOG_DIR/core.log" ]; then
                echo "Last 10 lines of log:"
                tail -10 "$DDS_LOG_DIR/core.log"
            fi
            return 1
        fi
        
        # Method 1: Check log for ready message (most reliable)
        if [ -f "$DDS_LOG_DIR/core.log" ]; then
            if grep -q "robonix core ready\|all robonix modules initialized" "$DDS_LOG_DIR/core.log"; then
                # Core is ready according to log, but DDS discovery may take time
                # Try to verify service is available
                SERVICE_LIST_OUTPUT=$(ros2 service list 2>&1 | grep -v "ERROR\|Error\|error" || true)
                if echo "$SERVICE_LIST_OUTPUT" | grep -q "/rbnx/ping"; then
                    echo "✓ robonix-core is ready (service discovered after ${i}s)"
                    return 0
                fi
                # If log says ready but service not found, give it more time (DDS discovery)
                if [ $i -ge 5 ]; then
                    echo "✓ robonix-core is ready (from log), DDS discovery may take time"
                    echo "  Proceeding with tests (services should be available)"
                    return 0
                fi
            fi
        fi
        
        # Method 2: Check web GUI port (quick check)
        if command -v ss >/dev/null 2>&1; then
            if ss -ltn 2>/dev/null | grep -q ":8000"; then
                # Port is listening, core is likely ready
                if [ $i -ge 3 ]; then
                    echo "✓ robonix-core web GUI is listening, assuming ready"
                    echo "  (DDS service discovery may take additional time)"
                    return 0
                fi
            fi
        fi
        
        # Method 3: Try ros2 service list (may fail due to RMW mismatch, but worth trying)
        SERVICE_LIST_OUTPUT=$(ros2 service list 2>&1 | grep -v "ERROR\|Error\|error" || true)
        if echo "$SERVICE_LIST_OUTPUT" | grep -q "/rbnx/ping"; then
            echo "✓ robonix-core is ready (service discovered after ${i}s)"
            return 0
        fi
        
        # Show progress every 5 seconds
        if [ $((i % 5)) -eq 0 ]; then
            echo "  Still waiting... (${i}/30s)"
        fi
        
        sleep 1
    done
    
    echo "✗ robonix-core failed to start or services not discovered after 30s"
    echo ""
    echo "Diagnostics:"
    echo "  Process status:"
    if kill -0 "$CORE_PID" 2>/dev/null; then
        echo "    ✓ Process is running (PID: $CORE_PID)"
    else
        echo "    ✗ Process is not running"
    fi
    echo ""
    echo "  ROS2 Environment:"
    echo "    ROS_DOMAIN_ID: ${ROS_DOMAIN_ID:-not set}"
    echo "    RMW_IMPLEMENTATION: ${RMW_IMPLEMENTATION:-not set}"
    echo ""
    echo "  Available nodes:"
    NODE_LIST=$(ros2 node list 2>&1)
    if echo "$NODE_LIST" | grep -q "/rbnx/core"; then
        echo "$NODE_LIST" | grep "/rbnx" | sed 's/^/    /'
    else
        echo "$NODE_LIST" | grep -v "ERROR\|Error\|error" | head -5 | sed 's/^/    /' || echo "    (no nodes found or RMW error)"
    fi
    echo ""
    echo "  Available services:"
    SERVICE_LIST=$(ros2 service list 2>&1)
    if echo "$SERVICE_LIST" | grep -q "/rbnx"; then
        echo "$SERVICE_LIST" | grep "/rbnx" | sed 's/^/    /'
    else
        echo "$SERVICE_LIST" | grep -v "ERROR\|Error\|error" | head -10 | sed 's/^/    /' || echo "    (no services found or RMW error)"
    fi
    echo ""
    echo "  Note: If you see RMW errors above, it's because:"
    echo "    - robonix-core uses RustDDS (rustdds crate) via ros2-client"
    echo "    - ROS2 CLI tools use RMW (rmw_fastrtps_cpp)"
    echo "    - They can still communicate via DDS discovery protocol"
    echo ""
    echo "  Check log: $DDS_LOG_DIR/core.log"
    if [ -f "$DDS_LOG_DIR/core.log" ]; then
        echo "  Last 20 lines of log:"
        tail -20 "$DDS_LOG_DIR/core.log" | sed 's/^/    /'
    fi
    return 1
}

# Run Rust tests
run_rust_tests() {
    echo ""
    echo "=== Running Rust CLI Stress Tests ==="
    
    # Always rebuild to ensure latest code
    echo "Building Rust stress test tool..."
    cd "$TEST_DIR/rust_tests"
    cargo build --release > "$DDS_LOG_DIR/rust_build.log" 2>&1
    
    # Check for binary
    STRESS_TEST_BIN="target/release/robonix-stress-test"
    [ -f "$STRESS_TEST_BIN" ] || STRESS_TEST_BIN=$(find target/release -maxdepth 1 -type f -executable | head -1)
    
    if [ -n "$STRESS_TEST_BIN" ]; then
        echo "✓ Found stress test binary: $STRESS_TEST_BIN"
        
        # Run multiple Rust test clients concurrently
        RUST_PIDS=()
        for i in $(seq 1 $CONCURRENCY); do
            $STRESS_TEST_BIN --client-id $i --requests $REQUESTS --rate $RATE --duration $DURATION > "$DDS_LOG_DIR/rust_test_$i.log" 2>&1 &
            RUST_PID=$!
            RUST_PIDS+=($RUST_PID)
            echo "$RUST_PID" >> "$PID_FILE"
            # sleep 0.5
        done
        
        # Wait for all Rust tests
        for pid in "${RUST_PIDS[@]}"; do
            wait $pid 2>/dev/null || true
        done
        echo "✓ Rust tests completed"
    else
        echo "✗ Rust stress test binary not found"
        return 1
    fi
}

# Run Python tests
run_python_tests() {
    echo ""
    echo "=== Running Python Stress Tests ==="
    
    cd "$TEST_DIR/python_tests"
    
    # Run multiple Python test clients concurrently
    PYTHON_PIDS=()
    for i in $(seq 1 $CONCURRENCY); do
        python3 stress_test.py --client-id $i --requests $REQUESTS --rate $RATE --duration $DURATION > "$DDS_LOG_DIR/python_test_$i.log" 2>&1 &
        PYTHON_PID=$!
        PYTHON_PIDS+=($PYTHON_PID)
        echo "$PYTHON_PID" >> "$PID_FILE"
        # sleep 0.5
    done
    
    # Wait for all Python tests (with timeout)
    for pid in "${PYTHON_PIDS[@]}"; do
        # Wait with timeout (max 5 minutes per client)
        timeout=300
        start_time=$(date +%s)
        while kill -0 $pid 2>/dev/null; do
            current_time=$(date +%s)
            elapsed=$((current_time - start_time))
            if [ $elapsed -gt $timeout ]; then
                echo "Warning: Python test $pid timed out, killing..."
                kill -9 $pid 2>/dev/null || true
                break
            fi
            sleep 1
        done
        wait $pid 2>/dev/null || true
    done
    echo "✓ Python tests completed"
}

# Run C++ tests
run_cpp_tests() {
    echo ""
    echo "=== Running C++ Stress Tests ==="
    
    cd "$TEST_DIR/cpp_tests"
    
    # Build if needed
    if [ ! -f "build/stress_test" ]; then
        echo "Building C++ stress test..."
        mkdir -p build
        cd build
        cmake .. > "$DDS_LOG_DIR/cpp_build.log" 2>&1
        make -j$(nproc) >> "$DDS_LOG_DIR/cpp_build.log" 2>&1
        cd ..
    fi
    
    if [ -f "build/stress_test" ]; then
        echo "✓ Found C++ stress test binary"
        # Run multiple C++ test clients concurrently
        CPP_PIDS=()
        for i in $(seq 1 $CONCURRENCY); do
            ./build/stress_test --client-id $i --requests $REQUESTS --rate $RATE --duration $DURATION > "$DDS_LOG_DIR/cpp_test_$i.log" 2>&1 &
            CPP_PID=$!
            CPP_PIDS+=($CPP_PID)
            echo "$CPP_PID" >> "$PID_FILE"
            # sleep 0.5
        done
        
        # Wait for all C++ tests
        for pid in "${CPP_PIDS[@]}"; do
            wait $pid 2>/dev/null || true
        done
        echo "✓ C++ tests completed"
    else
        echo "✗ C++ stress test binary not found"
        return 1
    fi
}

# Main execution
main() {
    START_TIME_RUN=$(date +%s)
    echo "=========================================="
    echo "Robonix Test Framework"
    echo "=========================================="
    echo "Test Type: $TEST_TYPE"
    echo "DDS Type: $DDS_TYPE"
    echo "Concurrency: $CONCURRENCY clients"
    echo "Log Directory: $DDS_LOG_DIR"
    echo "Process Group ID: $TEST_PGID"
    echo "=========================================="
    
    # Set DDS implementation for ROS2 CLI tools
    export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
    if [ "$DDS_TYPE" = "fastdds" ]; then
        echo "Testing FastDDS: robonix-core and CLI tools both use FastDDS (rmw_fastrtps_cpp)"
    else
        echo "Testing RustDDS: robonix-core uses RustDDS (rustdds crate via ros2-client)"
        echo "  CLI tools use fastrtps (rmw_fastrtps_cpp) for service discovery"
    fi
    
    # Ensure ROS2 daemon is running
    if ! ros2 daemon status >/dev/null 2>&1; then
        echo "Starting ROS2 daemon..."
        ros2 daemon start >/dev/null 2>&1 || true
        sleep 1
    fi
    
    # Start core first
    if ! start_core; then
        echo "Failed to start robonix-core"
        exit 1
    fi
    
    # Run tests based on type
    case "$TEST_TYPE" in
        rust)
            run_rust_tests
            ;;
        python)
            run_python_tests
            ;;
        cpp)
            run_cpp_tests
            ;;
        all)
            run_rust_tests || true
            run_cpp_tests || true
            run_python_tests || true
            ;;
        *)
            echo "Unknown test type: $TEST_TYPE"
            echo "Valid types: rust, python, cpp, all"
            exit 1
            ;;
    esac
    
    END_TIME_RUN=$(date +%s)
    DURATION_RUN=$((END_TIME_RUN - START_TIME_RUN))

    echo ""
    echo "=========================================="
    echo "✓ Test run completed in ${DURATION_RUN}s"
    echo "Check logs in: $DDS_LOG_DIR"
    echo ""
    echo "Generating benchmark report..."
    python3 "$TEST_DIR/report.py" \
        --log-dir "$DDS_LOG_DIR" \
        --output "$DDS_LOG_DIR/benchmark_report.json" \
        --requests "$REQUESTS" \
        --rate "$RATE" \
        --duration "$DURATION" \
        --concurrency "$CONCURRENCY"
    echo ""
    echo "Benchmark report saved to: $DDS_LOG_DIR/benchmark_report.json"
    echo "=========================================="
}

# Run main function
main
