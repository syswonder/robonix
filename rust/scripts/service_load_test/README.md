# Service Load Test

This directory contains high-intensity load testing scripts for robonix-core services.

## Purpose

Test concurrent access to robonix-core services, particularly the ping pong service, to identify any issues with multiple processes making high-frequency requests.

## Files

- `ping_client_1.py`: First test client node that continuously calls the ping pong service
- `ping_client_2.py`: Second test client node that continuously calls the ping pong service
- `run_test.sh`: Script to run both test clients simultaneously

## Usage

1. Build robonix-sdk to generate service types:
```bash
cd /root/workspace/rust/robonix-sdk
colcon build --packages-select robonix_sdk
source install/setup.bash
```

2. Start robonix-core:
```bash
cd /root/workspace/rust/robonix-core
cargo run --release
```

3. In separate terminals, run the test clients:
```bash
# Terminal 1
cd /root/workspace/rust/scripts/service_load_test
source /root/workspace/rust/robonix-sdk/install/setup.bash
python3 ping_client_1.py

# Terminal 2
cd /root/workspace/rust/scripts/service_load_test
source /root/workspace/rust/robonix-sdk/install/setup.bash
python3 ping_client_2.py
```

Or use the test script:
```bash
./run_test.sh
```

Note: The test clients will automatically fall back to using `ros2 service call` if the service types are not available, but native ROS2 client is preferred for better performance.

## Expected Behavior

Both clients should be able to successfully call the ping pong service concurrently without one blocking the other. The clients will print statistics every 5 seconds showing:
- Total requests
- Successful requests
- Failed requests
- Request rate (req/s)
- Success rate (%)

## Troubleshooting

If you see high failure rates or one client blocking the other, this indicates a concurrency issue in robonix-core that needs to be fixed.

