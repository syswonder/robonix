# SPDX-License-Identifier: MulanPSL-2.0
"""Latency benchmark runner: measures RTT for each transport and computes statistics."""

import argparse
import json
import os
import sys
import time
from pathlib import Path

import numpy as np


from latency_bench.payload import get_payload, validate_response


def run_grpc_bench(addr: str, payload: bytes, warmup: int, iterations: int) -> list[float]:
    import grpc
    from proto import echo_pb2, echo_pb2_grpc

    channel = grpc.insecure_channel(addr)
    stub = echo_pb2_grpc.EchoStub(channel)

    def one_call():
        t0 = time.perf_counter()
        resp = stub.Echo(echo_pb2.EchoRequest(data=payload))
        t1 = time.perf_counter()
        if not validate_response(payload, resp.data):
            raise ValueError("Response mismatch")
        return (t1 - t0) * 1e6

    for _ in range(warmup):
        one_call()

    latencies = []
    for _ in range(iterations):
        latencies.append(one_call())
    return latencies


def run_zmq_bench(addr: str, payload: bytes, warmup: int, iterations: int) -> list[float]:
    import zmq

    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.connect(addr)

    def one_call():
        t0 = time.perf_counter()
        sock.send(payload)
        resp = sock.recv()
        t1 = time.perf_counter()
        if not validate_response(payload, resp):
            raise ValueError("Response mismatch")
        return (t1 - t0) * 1e6

    for _ in range(warmup):
        one_call()

    latencies = []
    for _ in range(iterations):
        latencies.append(one_call())
    return latencies


def run_http_bench(url: str, payload: bytes, warmup: int, iterations: int) -> list[float]:
    import urllib.request

    def one_call():
        t0 = time.perf_counter()
        req = urllib.request.Request(url, data=payload, method="POST")
        req.add_header("Content-Type", "application/octet-stream")
        with urllib.request.urlopen(req) as resp:
            resp_data = resp.read()
        t1 = time.perf_counter()
        if not validate_response(payload, resp_data):
            raise ValueError("Response mismatch")
        return (t1 - t0) * 1e6

    for _ in range(warmup):
        one_call()

    latencies = []
    for _ in range(iterations):
        latencies.append(one_call())
    return latencies


def run_ros2_bench(payload: bytes, warmup: int, iterations: int) -> list[float]:
    import rclpy
    from rclpy.node import Node
    from latency_bench_msgs.srv import Echo

    rclpy.init()
    node = Node("bench_client")
    client = node.create_client(Echo, "/latency_bench/echo")
    if not client.wait_for_service(timeout_sec=10.0):
        raise RuntimeError("ROS2 echo service not available")

    def one_call():
        req = Echo.Request()
        req.data = list(payload)
        t0 = time.perf_counter()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)
        result = future.result()
        t1 = time.perf_counter()
        if result is None:
            raise RuntimeError("ROS2 call failed")
        if not validate_response(payload, bytes(result.data)):
            raise ValueError("Response mismatch")
        return (t1 - t0) * 1e6

    for _ in range(warmup):
        one_call()

    latencies = []
    for _ in range(iterations):
        latencies.append(one_call())

    node.destroy_node()
    rclpy.shutdown()
    return latencies


def compute_stats(latencies: list[float]) -> dict:
    arr = np.array(latencies)
    return {
        "min_us": float(np.min(arr)),
        "max_us": float(np.max(arr)),
        "mean_us": float(np.mean(arr)),
        "median_us": float(np.median(arr)),
        "p50_us": float(np.percentile(arr, 50)),
        "p95_us": float(np.percentile(arr, 95)),
        "p99_us": float(np.percentile(arr, 99)),
        "std_us": float(np.std(arr)),
        "count": len(latencies),
    }


def print_stats(name: str, stats: dict):
    print(f"\n=== {name} ===")
    print(f"  min:    {stats['min_us']:.2f} μs")
    print(f"  max:    {stats['max_us']:.2f} μs")
    print(f"  mean:   {stats['mean_us']:.2f} μs")
    print(f"  median: {stats['median_us']:.2f} μs")
    print(f"  p50:    {stats['p50_us']:.2f} μs")
    print(f"  p95:    {stats['p95_us']:.2f} μs")
    print(f"  p99:    {stats['p99_us']:.2f} μs")
    print(f"  std:    {stats['std_us']:.2f} μs")
    print(f"  count:  {stats['count']}")


def main():
    parser = argparse.ArgumentParser(description="Latency benchmark for communication transports")
    parser.add_argument("--transport", choices=["grpc", "zmq", "http", "ros2"], required=True)
    parser.add_argument("--iterations", type=int, default=10000)
    parser.add_argument("--warmup", type=int, default=100)
    parser.add_argument("--payload-size", type=int, default=64, choices=[64, 256, 1024])
    parser.add_argument("--grpc-addr", default="127.0.0.1:50052")
    parser.add_argument("--zmq-addr", default="tcp://127.0.0.1:5555")
    parser.add_argument("--http-url", default="http://127.0.0.1:18080/echo")
    parser.add_argument("--output", "-o", help="Write JSON results to file")
    args = parser.parse_args()

    payload = get_payload(args.payload_size)

    try:
        if args.transport == "grpc":
            latencies = run_grpc_bench(args.grpc_addr, payload, args.warmup, args.iterations)
        elif args.transport == "zmq":
            latencies = run_zmq_bench(args.zmq_addr, payload, args.warmup, args.iterations)
        elif args.transport == "http":
            latencies = run_http_bench(args.http_url, payload, args.warmup, args.iterations)
        elif args.transport == "ros2":
            latencies = run_ros2_bench(payload, args.warmup, args.iterations)
        else:
            raise ValueError(f"Unknown transport: {args.transport}")
    except Exception as e:
        print(f"Benchmark failed: {e}", file=sys.stderr)
        sys.exit(1)

    stats = compute_stats(latencies)
    rmw = os.environ.get("RMW_IMPLEMENTATION", "default")
    name = f"{args.transport} (RMW={rmw})" if args.transport == "ros2" else args.transport
    print_stats(name, stats)

    if args.output:
        out = {
            "transport": args.transport,
            "rmw": rmw if args.transport == "ros2" else None,
            "payload_size": args.payload_size,
            "iterations": args.iterations,
            "warmup": args.warmup,
            "stats": stats,
        }
        Path(args.output).parent.mkdir(parents=True, exist_ok=True)
        with open(args.output, "w") as f:
            json.dump(out, f, indent=2)
        print(f"\nResults written to {args.output}")


if __name__ == "__main__":
    main()
