#!/usr/bin/env python3
"""Minimal check: gRPC QueryNodes against a running robonix-server (no VLM, no ROS)."""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

_RUST = Path(__file__).resolve().parents[2]
_PG = _RUST / "examples" / "proto_gen"
sys.path.insert(0, str(_PG))

import grpc  # noqa: E402

import robonix_runtime_pb2 as pb  # noqa: E402
import robonix_runtime_pb2_grpc as pb_grpc  # noqa: E402


def _target() -> str:
    s = os.environ.get("ROBONIX_SERVER", "127.0.0.1:50051")
    if s.startswith("http://"):
        s = s[len("http://") :]
    if s.startswith("https://"):
        s = s[len("https://") :]
    return s


def main() -> int:
    t = _target()
    deadline = time.time() + float(os.environ.get("SMOKE_DEADLINE_SEC", "15"))
    last = None
    while time.time() < deadline:
        try:
            ch = grpc.insecure_channel(t)
            grpc.channel_ready_future(ch).result(timeout=5.0)
            stub = pb_grpc.RobonixRuntimeStub(ch)
            r = stub.QueryNodes(pb.QueryNodesRequest())
            print(f"[smoke] OK QueryNodes count={len(r.nodes)}")
            for n in r.nodes:
                print(f"  - {n.node_id} ({n.namespace})")
            return 0
        except Exception as e:
            last = e
            time.sleep(0.5)
    print(f"[smoke] FAIL: {last}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
