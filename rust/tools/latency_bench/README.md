# Latency Benchmark: Communication Transport Comparison

本实验对比不同通信方式下的 **RTT (Round-Trip Time)** 延迟，在控制变量的前提下统计各传输层的 latency。

## 通信方式

| 传输层 | 说明 |
|--------|------|
| **ROS2 Service (FastDDS)** | `RMW_IMPLEMENTATION=rmw_fastrtps_cpp` |
| **ROS2 Service (Zenoh)** | `RMW_IMPLEMENTATION=rmw_zenoh_cpp` |
| **gRPC** | tonic/protobuf |
| **ZeroMQ** | REQ/REP 模式 |
| **HTTP** | POST 原始 binary body |

## 控制变量

- **数据格式**：所有传输使用相同的 binary payload（见 `PAYLOAD_SPEC.md`）
- **Payload 大小**：默认 64 字节，可配置 64/256/1024
- **迭代次数**：默认 10000 次
- **预热**：每次测试前 100 次 warmup
- **环境**：localhost，同一进程或同机不同进程

## 目录结构

```
latency_bench/
├── README.md
├── PAYLOAD_SPEC.md
├── requirements.txt
├── proto/echo.proto          # gRPC 定义
├── payload.py                 # 共享 payload 生成
├── servers/                   # 各传输 server
├── clients/                   # 各传输 client
├── benchmark.py               # 主 runner + 统计
├── run_bench.sh               # 一键运行
└── latency_bench_msgs/        # ROS2 srv 定义
```

## 运行

```bash
cd rust/tools/latency_bench
pip install -r requirements.txt
./run_bench.sh
```

**ROS2 (FastDDS/Zenoh)** 需先构建：

```bash
./build_ros2.sh
source ros2_ws/install/setup.bash
# Zenoh 需先启动 rmw_zenohd
./run_bench.sh
```

或单独运行某传输：

```bash
export PYTHONPATH="$PWD:$PYTHONPATH"

# 启动 server（另开终端）
python3 -m latency_bench.servers.grpc_server
python3 -m latency_bench.servers.zmq_server
python3 -m latency_bench.servers.http_server
python3 -m latency_bench.servers.ros2_server   # 需 source ros2_ws + RMW_IMPLEMENTATION

# 运行 client 并统计
python3 -m latency_bench.benchmark --transport grpc --iterations 10000
```

## 输出

- 控制台打印：min, max, mean, median, p50, p95, p99 (μs)
- JSON 输出到 `results/`
- 汇总对比（自动生成 summary.txt 和 latency_plot.png）：`python3 summarize_results.py results`
- **startup**：从 client 初始化到首次成功响应（含 rclpy.init、DDS 发现等，ROS2 明显慢于 gRPC/ZMQ）
- **稳定度**：std（标准差）、cv%（变异系数，越低越稳定）
