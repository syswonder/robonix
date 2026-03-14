# SPDX-License-Identifier: MulanPSL-2.0
"""gRPC echo client for latency benchmark."""

import grpc
from proto import echo_pb2, echo_pb2_grpc


def echo(channel: grpc.Channel, data: bytes) -> bytes:
    """Send data, receive echo. Returns response data."""
    stub = echo_pb2_grpc.EchoStub(channel)
    resp = stub.Echo(echo_pb2.EchoRequest(data=data))
    return resp.data


def create_channel(addr: str = "127.0.0.1:50052") -> grpc.Channel:
    return grpc.insecure_channel(addr)
