# SPDX-License-Identifier: MulanPSL-2.0
"""ZeroMQ REQ/REP echo client for latency benchmark."""

import zmq


def echo(sock: zmq.Socket, data: bytes) -> bytes:
    """Send data, receive echo. Returns response data."""
    sock.send(data)
    return sock.recv()


def create_socket(addr: str = "tcp://127.0.0.1:5555") -> zmq.Socket:
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REQ)
    sock.connect(addr)
    return sock
