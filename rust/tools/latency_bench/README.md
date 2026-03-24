# Latency benchmark

Compares RTT across **gRPC**, **ZeroMQ**, **HTTP**, and **ROS 2 services** (default RMW `rmw_fastrtps_cpp`).

## Run

```bash
cd rust/tools/latency_bench
pip install -r requirements.txt
./build_ros2.sh
source ros2_ws/install/setup.bash
./run_bench.sh
```

See `PAYLOAD_SPEC.md` for payload rules.
