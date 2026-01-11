#!/bin/bash
# Run Benchmark Suite
# Tests RustDDS with different concurrency levels

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
TEST_DIR="$SCRIPT_DIR"

# Create benchmark-specific log directory with timestamp
BENCHMARK_TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BENCHMARK_LOG_DIR="$TEST_DIR/logs/benchmark_${BENCHMARK_TIMESTAMP}"
mkdir -p "$BENCHMARK_LOG_DIR"

# Concurrency levels to test
CONCURRENCY_LEVELS=(1 3 5 10)

echo "=========================================="
echo "Robonix Benchmark Suite (RustDDS)"
echo "=========================================="
echo "Concurrency Levels: ${CONCURRENCY_LEVELS[@]}"
echo "Benchmark Timestamp: $BENCHMARK_TIMESTAMP"
echo "Benchmark Log Directory: $BENCHMARK_LOG_DIR"
echo "=========================================="
echo ""

# Run benchmarks for each concurrency level
for concurrency in "${CONCURRENCY_LEVELS[@]}"; do
    echo ""
    echo "=========================================="
    echo "Testing Concurrency Level: $concurrency"
    echo "=========================================="
    
    # Run tests with specified concurrency
    # Use benchmark directory as base for logs
    cd "$TEST_DIR"
    LOG_DIR_BASE="$BENCHMARK_LOG_DIR" bash test.sh all "rustdds" "$concurrency" 1000 100 0 || true
    
    echo "✓ Benchmark completed: Concurrency $concurrency"
done

echo ""
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
