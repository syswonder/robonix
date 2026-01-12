#!/bin/bash
# Run Benchmark Suite
# Tests RustDDS with different concurrency levels

set -e

# Set timezone to Asia/Shanghai (UTC+8)
export TZ=Asia/Shanghai

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
TEST_DIR="$SCRIPT_DIR"

# Create benchmark-specific log directory with timestamp
BENCHMARK_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BENCHMARK_LOG_DIR="$TEST_DIR/logs/benchmark_${BENCHMARK_TIMESTAMP}"
mkdir -p "$BENCHMARK_LOG_DIR"

# Concurrency levels to test
CONCURRENCY_LEVELS=(1 2 4 8)
REPEATS=3

echo "=========================================="
echo "Robonix Benchmark Suite (RustDDS)"
echo "=========================================="
echo "Concurrency Levels: ${CONCURRENCY_LEVELS[@]}"
echo "Repeats per Level:  $REPEATS"
echo "Benchmark Timestamp: $BENCHMARK_TIMESTAMP"
echo "Benchmark Log Directory: $BENCHMARK_LOG_DIR"
echo "=========================================="
echo ""

START_TIME_TOTAL=$(date +%s)
TOTAL_STEPS=$((${#CONCURRENCY_LEVELS[@]} * REPEATS))
CURRENT_STEP=0

# Run benchmarks for each concurrency level
for concurrency in "${CONCURRENCY_LEVELS[@]}"; do
    for repeat in $(seq 1 $REPEATS); do
        CURRENT_STEP=$((CURRENT_STEP + 1))
        PERCENT=$((CURRENT_STEP * 100 / TOTAL_STEPS))
        
        echo "------------------------------------------"
        echo "[${PERCENT}%] Progress: ${CURRENT_STEP}/${TOTAL_STEPS}"
        echo "Testing Concurrency: $concurrency (Repeat $repeat/$REPEATS)"
        echo "------------------------------------------"
        
        # Run tests with specified concurrency
        cd "$TEST_DIR"
        LOG_DIR_BASE="$BENCHMARK_LOG_DIR" bash test.sh all "rustdds" "$concurrency" 500 100 0 || true
    done
done

END_TIME_TOTAL=$(date +%s)
DURATION_TOTAL=$((END_TIME_TOTAL - START_TIME_TOTAL))

echo ""
echo "=========================================="
echo "✓ All Experiments Finished in ${DURATION_TOTAL}s"
echo "Generating comprehensive benchmark report..."
python3 "$TEST_DIR/analyze.py" "$BENCHMARK_LOG_DIR"

echo ""
echo "=========================================="
echo "✓ Benchmark suite finished"
echo "Benchmark results: $BENCHMARK_LOG_DIR"
echo "Summary report: $BENCHMARK_LOG_DIR/benchmark_summary.txt"
echo "Rust Latency curves: $BENCHMARK_LOG_DIR/latency_curves_rust.png"
echo "Python Latency curves: $BENCHMARK_LOG_DIR/latency_curves_python.png"
echo "Failure comparison: $BENCHMARK_LOG_DIR/failure_rate_comparison.png"
echo "=========================================="
