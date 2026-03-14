# SPDX-License-Identifier: MulanPSL-2.0
"""ZeroMQ REQ/REP echo server for latency benchmark."""

import zmq


def main():
    ctx = zmq.Context()
    sock = ctx.socket(zmq.REP)
    sock.bind("tcp://*:5555")
    print("ZeroMQ echo server listening on tcp://*:5555", flush=True)
    while True:
        data = sock.recv()
        sock.send(data)


if __name__ == "__main__":
    main()
