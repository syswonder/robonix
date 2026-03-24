# Payload specification (controlled variables)

To keep latency measurements comparable across transports, all implementations must follow this spec.

## 1. Data format

- **Type**: Raw binary (`bytes`)
- **Content**: Reproducible pseudo-random sequence from a fixed seed
- **Size**: 64 / 256 / 1024 bytes supported (default 64)

## 2. Semantics

- **Request**: Client sends an N-byte payload
- **Response**: Server echoes the same N bytes
- **Verification**: Client must assert `response == request`

## 3. Implementation requirements

| Transport | Request encoding | Response encoding |
|-----------|------------------|-------------------|
| gRPC | `bytes data` (proto) | `bytes data` |
| ZeroMQ | raw bytes | raw bytes |
| HTTP | POST body (`application/octet-stream`) | response body |
| ROS2 | `uint8[] data` (srv) | `uint8[] data` |

## 4. Timing

- **startup**: Wall time from client initialization through first successful response, including:
  - gRPC: channel + stub + first RPC
  - ZMQ: context + socket + connect + first send/recv
  - HTTP: first request (including TCP connection)
  - ROS2: **`rclpy.init()`** + node + client + `wait_for_service` + first call (`rclpy` init and DDS discovery are relatively slow)
- **steady-state RTT**: After warmup, from `t0` immediately before send to `t1` when the full response is received; `(t1 - t0)` in microseconds

Each sample excludes business logic beyond serialization/deserialization.
