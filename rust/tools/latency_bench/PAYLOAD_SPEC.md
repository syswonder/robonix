# Payload 规范（控制变量）

为保证各传输层 latency 测试的可比性，所有实现必须遵循以下规范。

## 1. 数据格式

- **类型**：原始 binary (bytes)
- **内容**：可复现的伪随机序列，基于固定 seed
- **大小**：支持 64 / 256 / 1024 字节（默认 64）

## 2. 语义

- **Request**：client 发送 N 字节 payload
- **Response**：server 原样回显 (echo) N 字节
- **校验**：client 必须验证 response == request

## 3. 实现要求

| 传输层 | Request 编码 | Response 编码 |
|--------|-------------|---------------|
| gRPC | `bytes data` (proto) | `bytes data` |
| ZeroMQ | raw bytes | raw bytes |
| HTTP | POST body (application/octet-stream) | response body |
| ROS2 | `uint8[] data` (srv) | `uint8[] data` |

## 4. 时间测量

- **startup (启动时间)**：从 client 初始化到首次成功响应的总时间，含：
  - gRPC: channel + stub + 首 RPC
  - ZMQ: context + socket + connect + 首 send/recv
  - HTTP: 首 request（含 TCP 连接）
  - ROS2: **rclpy.init()** + node + client + wait_for_service + 首 call（rclpy 初始化与 DDS 发现较慢）
- **steady-state RTT**：warmup 后的请求，`t0` 发送前 → `t1` 收到完整 response，`(t1 - t0)` 微秒

单次测量不包含序列化/反序列化外的业务逻辑。
