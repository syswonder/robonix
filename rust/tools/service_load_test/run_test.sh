#!/bin/bash
# Run both ping clients simultaneously to test concurrent service access

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# source robonix sdk
ROBONIX_SDK_DIR=$(realpath "../../robonix-sdk")

(
  cd $ROBONIX_SDK_DIR
  ./build_ros2.sh
)

source $ROBONIX_SDK_DIR/install/setup.bash

echo "Starting ping client 1..."
python3 ping_client_1.py &
CLIENT1_PID=$!

sleep 2

echo "Starting ping client 2..."
python3 ping_client_2.py &
CLIENT2_PID=$!

echo "Both clients started. PIDs: $CLIENT1_PID, $CLIENT2_PID"
echo "Press Ctrl+C to stop both clients"

# Wait for user interrupt
trap "kill $CLIENT1_PID $CLIENT2_PID 2>/dev/null; exit" INT TERM

wait

